"""
AEB Threshold and Braking Parameter Sensitivity Study Module.

Performs controlled, multi-seed sensitivity analysis of AEB controller parameters:
1. Time-To-Collision (TTC) threshold variations:
   - Warning TTC: [3.50, 4.00, 4.50] s
   - Level 1 TTC: [2.50, 3.00, 3.50] s
   - Level 2 TTC: [2.00, 2.25, 2.50] s
   - Level 3 TTC: [1.50, 1.75, 2.00] s
   - Combined configurations (Early/Conservative, Late/Aggressive)
   Enforcing strict ordering: Warning > L1 > L2 > L3 > 0

2. Commanded Braking Deceleration variations:
   - Level 1 Decel: [-1.50, -2.00, -2.50] m/s^2
   - Level 2 Decel: [-3.00, -4.00, -5.00] m/s^2
   - Level 3 Decel: [-5.00, -6.00, -7.00] m/s^2
   - Combined configurations (Mild/Gentle, Firm/Aggressive)
   Enforcing strict ordering: |L1| < |L2| < |L3| with negative deceleration demands.

3. Scenarios:
   - CCR (Car-to-Car Rear Stationary)
   - CCRm (Car-to-Car Rear Moving)
   - CCRb (Car-to-Car Rear Braking)

4. Robustness:
   - Multi-seed pseudo-random sequences (seeds: 42 to 51)
   - Nominal sensor noise (distance_noise_std = 0.05 m)
   - Nominal actuator dynamics (actuator_tau = 0.10 s)

Note:
This is a software sensitivity study in a discrete-time simulation environment.
It does not claim Hardware-in-the-Loop (HIL), VIRYA platform validation, or regulatory certification.
"Number of 3-sample sliding FSM reversal patterns" (triplet_reversal_count)
measures structural sliding-window reversal patterns, not physical chattering cycles.
"""

import os
import csv
import math
from dataclasses import dataclass, field
from typing import List, Dict, Any, Tuple, Optional
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.simulation import SimulationEngine, SimulationResult
from src.metrics import PerformanceEvaluator
from scenarios.scenarios import (
    CCRScenario,
    CCRmScenario,
    CCRbScenario,
)
from analysis.sensitivity_analysis import (
    count_fsm_state_transitions,
    compute_active_braking_metrics,
    check_boundary_chattering,
    count_triplet_reversal_patterns,
)


DEFAULT_SEEDS = [42, 43, 44, 45, 46, 47, 48, 49, 50, 51]
DEFAULT_DISTANCE_NOISE_STD = 0.05
DEFAULT_ACTUATOR_TAU = 0.10
DEFAULT_SENSOR_LATENCY = 0.00


class ConfigurableAEBController:
    """
    Parameterizable Finite State Machine (FSM) AEB Controller.

    Replicates the deterministic 5-priority FSM logic of AEBController
    while permitting parameterized TTC thresholds and deceleration demands.
    """

    def __init__(
        self,
        ttc_warning: float = 4.00,
        ttc_level_1: float = 3.00,
        ttc_level_2: float = 2.25,
        ttc_level_3: float = 1.75,
        a_level_1: float = -2.00,
        a_level_2: float = -4.00,
        a_level_3: float = -6.00,
        min_brake_duration: float = 0.20,
    ):
        for val, name in [
            (ttc_warning, "ttc_warning"),
            (ttc_level_1, "ttc_level_1"),
            (ttc_level_2, "ttc_level_2"),
            (ttc_level_3, "ttc_level_3"),
            (a_level_1, "a_level_1"),
            (a_level_2, "a_level_2"),
            (a_level_3, "a_level_3"),
            (min_brake_duration, "min_brake_duration"),
        ]:
            if not math.isfinite(val):
                raise ValueError(f"Parameter {name}={val} must be finite.")

        if not (ttc_warning > ttc_level_1 > ttc_level_2 > ttc_level_3 > 0.0):
            raise ValueError(
                f"Invalid TTC ordering: {ttc_warning} > {ttc_level_1} > "
                f"{ttc_level_2} > {ttc_level_3} > 0 required."
            )

        if not (0.0 > a_level_1 > a_level_2 > a_level_3):
            raise ValueError(
                f"Invalid deceleration ordering: 0 > {a_level_1} > "
                f"{a_level_2} > {a_level_3} required."
            )

        if min_brake_duration < 0.0:
            raise ValueError(
                f"Invalid min_brake_duration={min_brake_duration}. Must be non-negative."
            )

        self.ttc_warning = float(ttc_warning)
        self.ttc_level_1 = float(ttc_level_1)
        self.ttc_level_2 = float(ttc_level_2)
        self.ttc_level_3 = float(ttc_level_3)
        self.a_level_1 = float(a_level_1)
        self.a_level_2 = float(a_level_2)
        self.a_level_3 = float(a_level_3)
        self.min_brake_duration = float(min_brake_duration)

        self.current_state: str = "SAFE"
        self.state_time: float = 0.0

    def reset(self) -> None:
        """Reset controller state memory to SAFE."""
        self.current_state = "SAFE"
        self.state_time = 0.0

    @staticmethod
    def calculate_relative_velocity(ego_speed: float, target_speed: float) -> float:
        return ego_speed - target_speed

    @staticmethod
    def calculate_ttc(ego_speed: float, target_speed: float, distance: float) -> float:
        v_rel = ego_speed - target_speed
        if v_rel <= 0.0:
            return float("inf")
        return distance / v_rel

    def update_state(
        self, ttc: float, v_rel: float, dt: float = 0.05
    ) -> Tuple[str, float]:
        """Advance FSM state following strict 5-priority precedence."""
        if not math.isfinite(ttc) or not math.isfinite(v_rel):
            self.current_state = "SAFE"
            self.state_time = 0.0
            return self.current_state, 0.0

        self.state_time += dt

        # Priority 1: Safety-Critical Release / Threat Clearance
        # If target moving away or equal speed (v_rel <= 0), release immediately.
        # Normal release on TTC > ttc_warning is subject to minimum dwell retention.
        if v_rel <= 0.0:
            if self.current_state != "SAFE":
                self.current_state = "SAFE"
                self.state_time = 0.0
            return self.current_state, 0.0

        state = self.current_state

        # STATE 1: SAFE
        if state == "SAFE":
            if ttc <= self.ttc_level_3:
                self.current_state = "AEB_LEVEL_3"
                self.state_time = 0.0
            elif ttc <= self.ttc_level_2:
                self.current_state = "AEB_LEVEL_2"
                self.state_time = 0.0
            elif ttc <= self.ttc_level_1:
                self.current_state = "AEB_LEVEL_1"
                self.state_time = 0.0
            elif ttc <= self.ttc_warning:
                self.current_state = "WARNING"
                self.state_time = 0.0

        # STATE 2: WARNING
        elif state == "WARNING":
            if ttc <= self.ttc_level_3:
                self.current_state = "AEB_LEVEL_3"
                self.state_time = 0.0
            elif ttc <= self.ttc_level_2:
                self.current_state = "AEB_LEVEL_2"
                self.state_time = 0.0
            elif ttc <= self.ttc_level_1:
                self.current_state = "AEB_LEVEL_1"
                self.state_time = 0.0
            elif ttc > self.ttc_warning:
                self.current_state = "SAFE"
                self.state_time = 0.0

        # STATE 3: AEB_LEVEL_1
        elif state == "AEB_LEVEL_1":
            if ttc <= self.ttc_level_3:
                self.current_state = "AEB_LEVEL_3"
                self.state_time = 0.0
            elif ttc <= self.ttc_level_2:
                self.current_state = "AEB_LEVEL_2"
                self.state_time = 0.0
            elif self.state_time < self.min_brake_duration:
                pass  # Retain dwell
            elif ttc > self.ttc_level_1:
                if ttc <= self.ttc_warning:
                    self.current_state = "WARNING"
                else:
                    self.current_state = "SAFE"
                self.state_time = 0.0

        # STATE 4: AEB_LEVEL_2
        elif state == "AEB_LEVEL_2":
            if ttc <= self.ttc_level_3:
                self.current_state = "AEB_LEVEL_3"
                self.state_time = 0.0
            elif self.state_time < self.min_brake_duration:
                pass  # Retain dwell
            elif ttc > self.ttc_level_2:
                if ttc <= self.ttc_level_1:
                    self.current_state = "AEB_LEVEL_1"
                elif ttc <= self.ttc_warning:
                    self.current_state = "WARNING"
                else:
                    self.current_state = "SAFE"
                self.state_time = 0.0

        # STATE 5: AEB_LEVEL_3
        elif state == "AEB_LEVEL_3":
            if self.state_time < self.min_brake_duration:
                pass  # Retain dwell
            elif ttc > self.ttc_level_3:
                if ttc <= self.ttc_level_2:
                    self.current_state = "AEB_LEVEL_2"
                elif ttc <= self.ttc_level_1:
                    self.current_state = "AEB_LEVEL_1"
                elif ttc <= self.ttc_warning:
                    self.current_state = "WARNING"
                else:
                    self.current_state = "SAFE"
                self.state_time = 0.0

        decel_map = {
            "SAFE": 0.0,
            "WARNING": 0.0,
            "AEB_LEVEL_1": self.a_level_1,
            "AEB_LEVEL_2": self.a_level_2,
            "AEB_LEVEL_3": self.a_level_3,
        }
        return self.current_state, decel_map[self.current_state]


@dataclass
class SensitivityConfig:
    """Configuration descriptor for a sensitivity test case."""

    name: str
    category: str  # "baseline", "oat_ttc", "comb_ttc", "oat_brake", "comb_brake", "joint"
    param_changed: str
    baseline_value: str
    new_value: str
    ttc_warning: float = 4.00
    ttc_level_1: float = 3.00
    ttc_level_2: float = 2.25
    ttc_level_3: float = 1.75
    a_level_1: float = -2.00
    a_level_2: float = -4.00
    a_level_3: float = -6.00

    def create_controller(self) -> ConfigurableAEBController:
        return ConfigurableAEBController(
            ttc_warning=self.ttc_warning,
            ttc_level_1=self.ttc_level_1,
            ttc_level_2=self.ttc_level_2,
            ttc_level_3=self.ttc_level_3,
            a_level_1=self.a_level_1,
            a_level_2=self.a_level_2,
            a_level_3=self.a_level_3,
        )


def get_default_scenarios():
    """Return default CCR, CCRm, and CCRb scenarios."""
    return [
        CCRScenario(
            name="CCR_Stationary_Target",
            initial_ego_speed=15.0,
            initial_distance=30.0,
        ),
        CCRmScenario(
            name="CCRm_Moving_Target",
            initial_ego_speed=15.0,
            initial_target_speed=5.0,
            initial_distance=25.0,
        ),
        CCRbScenario(
            name="CCRb_Braking_Target",
            initial_ego_speed=15.0,
            initial_target_speed=10.0,
            initial_distance=30.0,
            target_decel=-2.0,
            brake_start_time=1.0,
        ),
    ]


def get_sensitivity_configurations() -> List[SensitivityConfig]:
    """
    Define systematic parameter sensitivity grid:
    - 1 Baseline configuration
    - 8 OAT TTC configurations (Warning, L1, L2, L3)
    - 2 Combined TTC configurations (Early, Late)
    - 6 OAT Braking configurations (L1, L2, L3)
    - 2 Combined Braking configurations (Mild, Firm)
    - 2 Joint Combined configurations (Early-Firm, Late-Mild)
    Total: 21 unique configurations.
    """
    configs: List[SensitivityConfig] = []

    # 1. Baseline
    configs.append(
        SensitivityConfig(
            name="Baseline",
            category="baseline",
            param_changed="None",
            baseline_value="4.00/3.00/2.25/1.75s, -2/-4/-6m/s²",
            new_value="Baseline",
            ttc_warning=4.00,
            ttc_level_1=3.00,
            ttc_level_2=2.25,
            ttc_level_3=1.75,
            a_level_1=-2.00,
            a_level_2=-4.00,
            a_level_3=-6.00,
        )
    )

    # 2. OAT TTC - Warning
    configs.append(
        SensitivityConfig(
            name="TTC_Warn_3.5s",
            category="oat_ttc",
            param_changed="TTC_warning",
            baseline_value="4.00 s",
            new_value="3.50 s",
            ttc_warning=3.50,
            ttc_level_1=3.00,
            ttc_level_2=2.25,
            ttc_level_3=1.75,
        )
    )
    configs.append(
        SensitivityConfig(
            name="TTC_Warn_4.5s",
            category="oat_ttc",
            param_changed="TTC_warning",
            baseline_value="4.00 s",
            new_value="4.50 s",
            ttc_warning=4.50,
            ttc_level_1=3.00,
            ttc_level_2=2.25,
            ttc_level_3=1.75,
        )
    )

    # 3. OAT TTC - Level 1
    configs.append(
        SensitivityConfig(
            name="TTC_L1_2.5s",
            category="oat_ttc",
            param_changed="TTC_level_1",
            baseline_value="3.00 s",
            new_value="2.50 s",
            ttc_warning=4.00,
            ttc_level_1=2.50,
            ttc_level_2=2.25,
            ttc_level_3=1.75,
        )
    )
    configs.append(
        SensitivityConfig(
            name="TTC_L1_3.5s",
            category="oat_ttc",
            param_changed="TTC_level_1",
            baseline_value="3.00 s",
            new_value="3.50 s",
            ttc_warning=4.00,
            ttc_level_1=3.50,
            ttc_level_2=2.25,
            ttc_level_3=1.75,
        )
    )

    # 4. OAT TTC - Level 2
    configs.append(
        SensitivityConfig(
            name="TTC_L2_2.0s",
            category="oat_ttc",
            param_changed="TTC_level_2",
            baseline_value="2.25 s",
            new_value="2.00 s",
            ttc_warning=4.00,
            ttc_level_1=3.00,
            ttc_level_2=2.00,
            ttc_level_3=1.75,
        )
    )
    configs.append(
        SensitivityConfig(
            name="TTC_L2_2.5s",
            category="oat_ttc",
            param_changed="TTC_level_2",
            baseline_value="2.25 s",
            new_value="2.50 s",
            ttc_warning=4.00,
            ttc_level_1=3.00,
            ttc_level_2=2.50,
            ttc_level_3=1.75,
        )
    )

    # 5. OAT TTC - Level 3
    configs.append(
        SensitivityConfig(
            name="TTC_L3_1.5s",
            category="oat_ttc",
            param_changed="TTC_level_3",
            baseline_value="1.75 s",
            new_value="1.50 s",
            ttc_warning=4.00,
            ttc_level_1=3.00,
            ttc_level_2=2.25,
            ttc_level_3=1.50,
        )
    )
    configs.append(
        SensitivityConfig(
            name="TTC_L3_2.0s",
            category="oat_ttc",
            param_changed="TTC_level_3",
            baseline_value="1.75 s",
            new_value="2.00 s",
            ttc_warning=4.00,
            ttc_level_1=3.00,
            ttc_level_2=2.25,
            ttc_level_3=2.00,
        )
    )

    # 6. Combined TTC configurations
    configs.append(
        SensitivityConfig(
            name="TTC_Early_Conservative",
            category="comb_ttc",
            param_changed="TTC_all",
            baseline_value="4.00/3.00/2.25/1.75 s",
            new_value="4.50/3.50/2.50/2.00 s",
            ttc_warning=4.50,
            ttc_level_1=3.50,
            ttc_level_2=2.50,
            ttc_level_3=2.00,
        )
    )
    configs.append(
        SensitivityConfig(
            name="TTC_Late_Aggressive",
            category="comb_ttc",
            param_changed="TTC_all",
            baseline_value="4.00/3.00/2.25/1.75 s",
            new_value="3.50/2.50/2.00/1.50 s",
            ttc_warning=3.50,
            ttc_level_1=2.50,
            ttc_level_2=2.00,
            ttc_level_3=1.50,
        )
    )

    # 7. OAT Braking - Level 1
    configs.append(
        SensitivityConfig(
            name="Brake_L1_-1.5",
            category="oat_brake",
            param_changed="A_level_1",
            baseline_value="-2.00 m/s²",
            new_value="-1.50 m/s²",
            a_level_1=-1.50,
            a_level_2=-4.00,
            a_level_3=-6.00,
        )
    )
    configs.append(
        SensitivityConfig(
            name="Brake_L1_-2.5",
            category="oat_brake",
            param_changed="A_level_1",
            baseline_value="-2.00 m/s²",
            new_value="-2.50 m/s²",
            a_level_1=-2.50,
            a_level_2=-4.00,
            a_level_3=-6.00,
        )
    )

    # 8. OAT Braking - Level 2
    configs.append(
        SensitivityConfig(
            name="Brake_L2_-3.0",
            category="oat_brake",
            param_changed="A_level_2",
            baseline_value="-4.00 m/s²",
            new_value="-3.00 m/s²",
            a_level_1=-2.00,
            a_level_2=-3.00,
            a_level_3=-6.00,
        )
    )
    configs.append(
        SensitivityConfig(
            name="Brake_L2_-5.0",
            category="oat_brake",
            param_changed="A_level_2",
            baseline_value="-4.00 m/s²",
            new_value="-5.00 m/s²",
            a_level_1=-2.00,
            a_level_2=-5.00,
            a_level_3=-6.00,
        )
    )

    # 9. OAT Braking - Level 3
    configs.append(
        SensitivityConfig(
            name="Brake_L3_-5.0",
            category="oat_brake",
            param_changed="A_level_3",
            baseline_value="-6.00 m/s²",
            new_value="-5.00 m/s²",
            a_level_1=-2.00,
            a_level_2=-4.00,
            a_level_3=-5.00,
        )
    )
    configs.append(
        SensitivityConfig(
            name="Brake_L3_-7.0",
            category="oat_brake",
            param_changed="A_level_3",
            baseline_value="-6.00 m/s²",
            new_value="-7.00 m/s²",
            a_level_1=-2.00,
            a_level_2=-4.00,
            a_level_3=-7.00,
        )
    )

    # 10. Combined Braking configurations
    configs.append(
        SensitivityConfig(
            name="Brake_Mild_Gentle",
            category="comb_brake",
            param_changed="A_all",
            baseline_value="-2.0/-4.0/-6.0 m/s²",
            new_value="-1.5/-3.0/-5.0 m/s²",
            a_level_1=-1.50,
            a_level_2=-3.00,
            a_level_3=-5.00,
        )
    )
    configs.append(
        SensitivityConfig(
            name="Brake_Firm_Aggressive",
            category="comb_brake",
            param_changed="A_all",
            baseline_value="-2.0/-4.0/-6.0 m/s²",
            new_value="-2.5/-5.0/-7.0 m/s²",
            a_level_1=-2.50,
            a_level_2=-5.00,
            a_level_3=-7.00,
        )
    )

    # 11. Joint Combined configurations
    configs.append(
        SensitivityConfig(
            name="Joint_Early_Firm",
            category="joint",
            param_changed="TTC_and_A_all",
            baseline_value="Baseline (4.0s / -6m/s²)",
            new_value="Early TTC (4.5s) + Firm Brake (-7.0m/s²)",
            ttc_warning=4.50,
            ttc_level_1=3.50,
            ttc_level_2=2.50,
            ttc_level_3=2.00,
            a_level_1=-2.50,
            a_level_2=-5.00,
            a_level_3=-7.00,
        )
    )
    configs.append(
        SensitivityConfig(
            name="Joint_Late_Mild",
            category="joint",
            param_changed="TTC_and_A_all",
            baseline_value="Baseline (4.0s / -6m/s²)",
            new_value="Late TTC (3.5s) + Mild Brake (-5.0m/s²)",
            ttc_warning=3.50,
            ttc_level_1=2.50,
            ttc_level_2=2.00,
            ttc_level_3=1.50,
            a_level_1=-1.50,
            a_level_2=-3.00,
            a_level_3=-5.00,
        )
    )

    return configs


def aggregate_sensitivity_results(
    raw_records: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """
    Aggregate raw records over seeds by (config_name, scenario).

    Computes statistical moments (mean, std, min, max, p05, p95) and
    calculates comparative delta vs the 'Baseline' configuration for each scenario.
    """
    groups: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    for r in raw_records:
        key = (r["config_name"], r["scenario"])
        groups.setdefault(key, []).append(r)

    # Pre-extract baseline means per scenario for comparative delta calculation
    baseline_stats: Dict[str, Dict[str, float]] = {}
    for (cfg_name, scenario), records in groups.items():
        if cfg_name == "Baseline":
            baseline_stats[scenario] = {
                "min_distance_mean": float(np.mean([r["minimum_distance"] for r in records])),
                "aeb_activation_time_mean": float(
                    np.mean([r["aeb_activation_time"] for r in records if r["aeb_activation_time"] is not None])
                ) if any(r["aeb_activation_time"] is not None for r in records) else 0.0,
                "stopping_distance_mean": float(
                    np.mean([r["stopping_distance"] for r in records if r["stopping_distance"] is not None])
                ) if any(r["stopping_distance"] is not None for r in records) else 0.0,
                "max_actual_decel_mean": float(np.mean([r["max_actual_deceleration"] for r in records])),
                "state_transitions_mean": float(np.mean([r["state_transition_count"] for r in records])),
                "triplet_reversals_mean": float(np.mean([r["triplet_reversal_count"] for r in records])),
            }

    numeric_metrics = [
        "minimum_distance",
        "aeb_activation_time",
        "ttc_at_activation",
        "max_braking_deceleration",
        "max_actual_deceleration",
        "ninety_percent_braking_response_time",
        "stopping_time",
        "stopping_distance",
        "final_relative_velocity",
        "minimum_relative_velocity",
        "state_transition_count",
        "triplet_reversal_count",
    ]

    summary_records: List[Dict[str, Any]] = []

    for (cfg_name, scenario), records in sorted(groups.items()):
        first = records[0]
        n_samples = len(records)
        collision_count = sum(1 for r in records if r["collision_status"] is True)
        collision_rate = collision_count / n_samples

        stopped_count = sum(1 for r in records if r["vehicle_stop_status"] == "STOPPED")
        stopped_rate = stopped_count / n_samples

        row: Dict[str, Any] = {
            "config_name": cfg_name,
            "category": first["category"],
            "param_changed": first["param_changed"],
            "baseline_value": first["baseline_value"],
            "new_value": first["new_value"],
            "ttc_warning": first["ttc_warning"],
            "ttc_level_1": first["ttc_level_1"],
            "ttc_level_2": first["ttc_level_2"],
            "ttc_level_3": first["ttc_level_3"],
            "a_level_1": first["a_level_1"],
            "a_level_2": first["a_level_2"],
            "a_level_3": first["a_level_3"],
            "scenario": scenario,
            "sample_size": n_samples,
            "collision_count": collision_count,
            "collision_rate": round(collision_rate, 4),
            "stopped_count": stopped_count,
            "stopped_rate": round(stopped_rate, 4),
        }

        for metric in numeric_metrics:
            vals = [r[metric] for r in records if r[metric] is not None]
            if vals:
                arr = np.array(vals, dtype=float)
                row[f"{metric}_mean"] = round(float(np.mean(arr)), 4)
                row[f"{metric}_std"] = round(float(np.std(arr)), 4)
                row[f"{metric}_min"] = round(float(np.min(arr)), 4)
                row[f"{metric}_max"] = round(float(np.max(arr)), 4)
                row[f"{metric}_p05"] = round(float(np.percentile(arr, 5)), 4)
                row[f"{metric}_p95"] = round(float(np.percentile(arr, 95)), 4)
            else:
                row[f"{metric}_mean"] = None
                row[f"{metric}_std"] = None
                row[f"{metric}_min"] = None
                row[f"{metric}_max"] = None
                row[f"{metric}_p05"] = None
                row[f"{metric}_p95"] = None

        # Calculate comparative deltas against baseline for this scenario
        if cfg_name == "Baseline":
            row["delta_min_distance_mean"] = 0.0
            row["pct_min_distance_mean"] = 0.0
            row["delta_activation_time_mean"] = 0.0
            row["delta_stopping_distance_mean"] = 0.0 if row.get("stopping_distance_mean") is not None else None
            row["pct_stopping_distance_mean"] = 0.0 if row.get("stopping_distance_mean") is not None else None
            row["delta_max_actual_decel_mean"] = 0.0
            row["delta_state_transitions_mean"] = 0.0
            row["delta_triplet_reversals_mean"] = 0.0
        else:
            b_stats = baseline_stats.get(scenario, {})
            cur_min_dist = row.get("minimum_distance_mean")
            base_min_dist = b_stats.get("min_distance_mean")
            if cur_min_dist is not None and base_min_dist is not None:
                row["delta_min_distance_mean"] = round(cur_min_dist - base_min_dist, 4)
                row["pct_min_distance_mean"] = round(
                    ((cur_min_dist - base_min_dist) / base_min_dist) * 100.0 if base_min_dist != 0 else 0.0,
                    2,
                )
            else:
                row["delta_min_distance_mean"] = None
                row["pct_min_distance_mean"] = None

            cur_act_time = row.get("aeb_activation_time_mean")
            base_act_time = b_stats.get("aeb_activation_time_mean")
            if cur_act_time is not None and base_act_time is not None:
                row["delta_activation_time_mean"] = round(cur_act_time - base_act_time, 4)
            else:
                row["delta_activation_time_mean"] = None

            cur_stop_dist = row.get("stopping_distance_mean")
            base_stop_dist = b_stats.get("stopping_distance_mean")
            if cur_stop_dist is not None and base_stop_dist is not None and base_stop_dist > 0:
                row["delta_stopping_distance_mean"] = round(cur_stop_dist - base_stop_dist, 4)
                row["pct_stopping_distance_mean"] = round(
                    ((cur_stop_dist - base_stop_dist) / base_stop_dist) * 100.0,
                    2,
                )
            else:
                row["delta_stopping_distance_mean"] = None
                row["pct_stopping_distance_mean"] = None

            cur_actual_decel = row.get("max_actual_deceleration_mean")
            base_actual_decel = b_stats.get("max_actual_decel_mean")
            if cur_actual_decel is not None and base_actual_decel is not None:
                row["delta_max_actual_decel_mean"] = round(cur_actual_decel - base_actual_decel, 4)
            else:
                row["delta_max_actual_decel_mean"] = None

            cur_trans = row.get("state_transition_count_mean")
            base_trans = b_stats.get("state_transitions_mean")
            if cur_trans is not None and base_trans is not None:
                row["delta_state_transitions_mean"] = round(cur_trans - base_trans, 2)
            else:
                row["delta_state_transitions_mean"] = None

            cur_rev = row.get("triplet_reversal_count_mean")
            base_rev = b_stats.get("triplet_reversals_mean")
            if cur_rev is not None and base_rev is not None:
                row["delta_triplet_reversals_mean"] = round(cur_rev - base_rev, 2)
            else:
                row["delta_triplet_reversals_mean"] = None

        summary_records.append(row)

    return summary_records


def run_aeb_parameter_sensitivity(
    configs: Optional[List[SensitivityConfig]] = None,
    scenarios: Optional[List[Any]] = None,
    seeds: Optional[List[int]] = None,
    noise_std: float = DEFAULT_DISTANCE_NOISE_STD,
    actuator_tau: float = DEFAULT_ACTUATOR_TAU,
    sensor_latency: float = DEFAULT_SENSOR_LATENCY,
    results_dir: str = "results/aeb_parameter_sensitivity",
    plots_dir: str = "plots/aeb_parameter_sensitivity",
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    Execute full AEB parameter sensitivity sweep across configurations, scenarios, and seeds.

    Args:
        configs: List of SensitivityConfig objects.
        scenarios: List of Scenario instances.
        seeds: List of random seeds.
        noise_std: Gaussian distance sensor noise std (m).
        actuator_tau: Actuator first-order lag time constant (s).
        sensor_latency: Sensor latency (s).
        results_dir: Directory for CSV outputs.
        plots_dir: Directory for generated plots.

    Returns:
        Tuple of (raw_records, summary_records).
    """
    if configs is None:
        configs = get_sensitivity_configurations()
    if scenarios is None:
        scenarios = get_default_scenarios()
    if seeds is None:
        seeds = DEFAULT_SEEDS

    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

    raw_records: List[Dict[str, Any]] = []

    for cfg in configs:
        for scenario in scenarios:
            for seed in seeds:
                engine = SimulationEngine(
                    sensor_latency=sensor_latency,
                    actuator_tau=actuator_tau,
                    distance_noise_std=noise_std,
                    distance_seed=seed,
                )
                # Assign the parameter-configured controller
                engine.controller = cfg.create_controller()

                result: SimulationResult = engine.run(scenario=scenario)
                metrics = PerformanceEvaluator.evaluate(result)

                state_transitions = count_fsm_state_transitions(result)
                braking_episodes, active_duration = compute_active_braking_metrics(result)
                chattering_detected, _ = check_boundary_chattering(result)
                triplet_reversals = count_triplet_reversal_patterns(result)

                act_time = (
                    round(metrics.aeb_activation_time, 4)
                    if metrics.aeb_activation_time is not None
                    else None
                )
                resp_time = (
                    round(metrics.ninety_percent_braking_response_time, 4)
                    if metrics.ninety_percent_braking_response_time is not None
                    else None
                )
                stop_time = (
                    round(metrics.stopping_time, 4)
                    if metrics.stopping_time is not None
                    else None
                )
                stop_dist = (
                    round(metrics.stopping_distance, 4)
                    if metrics.stopping_distance is not None
                    else None
                )
                ttc_act = (
                    round(metrics.ttc_at_activation, 4)
                    if metrics.ttc_at_activation is not None
                    else None
                )

                record: Dict[str, Any] = {
                    "config_name": cfg.name,
                    "category": cfg.category,
                    "param_changed": cfg.param_changed,
                    "baseline_value": cfg.baseline_value,
                    "new_value": cfg.new_value,
                    "ttc_warning": cfg.ttc_warning,
                    "ttc_level_1": cfg.ttc_level_1,
                    "ttc_level_2": cfg.ttc_level_2,
                    "ttc_level_3": cfg.ttc_level_3,
                    "a_level_1": cfg.a_level_1,
                    "a_level_2": cfg.a_level_2,
                    "a_level_3": cfg.a_level_3,
                    "scenario": scenario.name,
                    "seed": seed,
                    "collision_status": metrics.collision_status,
                    "minimum_distance": round(metrics.min_distance, 4),
                    "aeb_activation_time": act_time,
                    "ttc_at_activation": ttc_act,
                    "max_braking_deceleration": round(metrics.max_braking_deceleration, 4),
                    "max_actual_deceleration": round(metrics.max_actual_deceleration, 4),
                    "ninety_percent_braking_response_time": resp_time,
                    "stopping_time": stop_time,
                    "stopping_distance": stop_dist,
                    "vehicle_stop_status": metrics.vehicle_stop_status,
                    "final_relative_velocity": round(metrics.final_relative_velocity, 4),
                    "minimum_relative_velocity": round(metrics.min_relative_velocity, 4),
                    "final_ego_speed": round(metrics.final_ego_speed, 4),
                    "state_transition_count": state_transitions,
                    "triplet_reversal_count": triplet_reversals,
                    "braking_episode_count": braking_episodes,
                    "total_braking_duration": round(active_duration, 4),
                    "boundary_chattering_detected": chattering_detected,
                }
                raw_records.append(record)

    # Save raw records
    raw_csv_path = os.path.join(results_dir, "aeb_parameter_sensitivity_raw.csv")
    if raw_records:
        fieldnames = list(raw_records[0].keys())
        with open(raw_csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(raw_records)

    # Aggregate and save summary records
    summary_records = aggregate_sensitivity_results(raw_records)
    summary_csv_path = os.path.join(results_dir, "aeb_parameter_sensitivity_summary.csv")
    if summary_records:
        fieldnames = list(summary_records[0].keys())
        with open(summary_csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(summary_records)

    # Generate all plots
    generate_all_sensitivity_plots(summary_records, plots_dir)

    return raw_records, summary_records


# ==============================================================================
# Plotting Functions
# ==============================================================================

def plot_min_distance_vs_ttc(summary_records: List[Dict[str, Any]], plots_dir: str) -> None:
    """Plot 1: Minimum Separation Distance vs TTC Threshold Variations."""
    ttc_configs = [
        "Baseline",
        "TTC_Warn_3.5s", "TTC_Warn_4.5s",
        "TTC_L1_2.5s", "TTC_L1_3.5s",
        "TTC_L2_2.0s", "TTC_L2_2.5s",
        "TTC_L3_1.5s", "TTC_L3_2.0s",
        "TTC_Early_Conservative", "TTC_Late_Aggressive"
    ]
    scenarios = ["CCR_Stationary_Target", "CCRm_Moving_Target", "CCRb_Braking_Target"]
    sc_labels = {"CCR_Stationary_Target": "CCR (Stationary)", "CCRm_Moving_Target": "CCRm (Moving)", "CCRb_Braking_Target": "CCRb (Braking)"}
    colors = {"CCR_Stationary_Target": "#1f77b4", "CCRm_Moving_Target": "#2ca02c", "CCRb_Braking_Target": "#d62728"}

    fig, ax = plt.subplots(figsize=(12, 6))
    x = np.arange(len(ttc_configs))
    width = 0.25

    for idx, sc in enumerate(scenarios):
        sc_recs = {r["config_name"]: r for r in summary_records if r["scenario"] == sc}
        means = [sc_recs[c]["minimum_distance_mean"] if c in sc_recs else 0.0 for c in ttc_configs]
        stds = [sc_recs[c]["minimum_distance_std"] if c in sc_recs else 0.0 for c in ttc_configs]

        ax.bar(
            x + (idx - 1) * width,
            means,
            width=width,
            yerr=stds,
            capsize=3,
            label=sc_labels[sc],
            color=colors[sc],
            alpha=0.85,
            edgecolor="black",
            linewidth=0.5,
        )

    ax.set_xticks(x)
    ax.set_xticklabels(ttc_configs, rotation=35, ha="right", fontsize=9)
    ax.set_ylabel("Minimum Separation Distance (m)", fontsize=11, fontweight="bold")
    ax.set_title("Minimum Separation Distance vs. TTC Threshold Variations (10 Seeds Mean ± Std)", fontsize=12, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.5, axis="y")
    ax.axhline(0.0, color="black", linestyle="-", linewidth=1.0)
    ax.legend(framealpha=0.9, fontsize=10)

    fig.tight_layout()
    fig.savefig(os.path.join(plots_dir, "min_distance_vs_ttc_threshold.png"), dpi=300)
    plt.close(fig)


def plot_activation_time_vs_ttc(summary_records: List[Dict[str, Any]], plots_dir: str) -> None:
    """Plot 2: AEB Activation Time vs TTC Threshold Variations."""
    ttc_configs = [
        "Baseline",
        "TTC_Warn_3.5s", "TTC_Warn_4.5s",
        "TTC_L1_2.5s", "TTC_L1_3.5s",
        "TTC_L2_2.0s", "TTC_L2_2.5s",
        "TTC_L3_1.5s", "TTC_L3_2.0s",
        "TTC_Early_Conservative", "TTC_Late_Aggressive"
    ]
    scenarios = ["CCR_Stationary_Target", "CCRm_Moving_Target", "CCRb_Braking_Target"]
    sc_labels = {"CCR_Stationary_Target": "CCR", "CCRm_Moving_Target": "CCRm", "CCRb_Braking_Target": "CCRb"}
    markers = {"CCR_Stationary_Target": "o", "CCRm_Moving_Target": "s", "CCRb_Braking_Target": "^"}
    colors = {"CCR_Stationary_Target": "#1f77b4", "CCRm_Moving_Target": "#2ca02c", "CCRb_Braking_Target": "#d62728"}

    fig, ax = plt.subplots(figsize=(11, 5.5))

    for sc in scenarios:
        sc_recs = {r["config_name"]: r for r in summary_records if r["scenario"] == sc}
        times = [sc_recs[c]["aeb_activation_time_mean"] if c in sc_recs else 0.0 for c in ttc_configs]
        ax.plot(
            ttc_configs,
            times,
            marker=markers[sc],
            linewidth=2,
            markersize=7,
            label=sc_labels[sc],
            color=colors[sc],
        )

    ax.set_xticks(range(len(ttc_configs)))
    ax.set_xticklabels(ttc_configs, rotation=35, ha="right", fontsize=9)
    ax.set_ylabel("AEB Activation Time (s)", fontsize=11, fontweight="bold")
    ax.set_title("AEB Activation Time Across TTC Configurations", fontsize=12, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend(framealpha=0.9, fontsize=10)

    fig.tight_layout()
    fig.savefig(os.path.join(plots_dir, "activation_time_vs_ttc_threshold.png"), dpi=300)
    plt.close(fig)


def plot_max_actual_decel_vs_braking(summary_records: List[Dict[str, Any]], plots_dir: str) -> None:
    """Plot 3: Maximum Delivered Actual Deceleration vs Commanded Braking Level Variations."""
    brake_configs = [
        "Baseline",
        "Brake_L1_-1.5", "Brake_L1_-2.5",
        "Brake_L2_-3.0", "Brake_L2_-5.0",
        "Brake_L3_-5.0", "Brake_L3_-7.0",
        "Brake_Mild_Gentle", "Brake_Firm_Aggressive"
    ]
    scenarios = ["CCR_Stationary_Target", "CCRm_Moving_Target", "CCRb_Braking_Target"]
    sc_labels = {"CCR_Stationary_Target": "CCR", "CCRm_Moving_Target": "CCRm", "CCRb_Braking_Target": "CCRb"}
    colors = {"CCR_Stationary_Target": "#1f77b4", "CCRm_Moving_Target": "#2ca02c", "CCRb_Braking_Target": "#d62728"}

    fig, ax = plt.subplots(figsize=(11, 5.5))
    x = np.arange(len(brake_configs))
    width = 0.26

    for idx, sc in enumerate(scenarios):
        sc_recs = {r["config_name"]: r for r in summary_records if r["scenario"] == sc}
        decels = [sc_recs[c]["max_actual_deceleration_mean"] if c in sc_recs else 0.0 for c in brake_configs]
        ax.bar(
            x + (idx - 1) * width,
            decels,
            width=width,
            label=sc_labels[sc],
            color=colors[sc],
            alpha=0.85,
            edgecolor="black",
            linewidth=0.5,
        )

    ax.set_xticks(x)
    ax.set_xticklabels(brake_configs, rotation=35, ha="right", fontsize=9)
    ax.set_ylabel("Max Actual Deceleration Delivered (m/s²)", fontsize=11, fontweight="bold")
    ax.set_title("Delivered Deceleration Under Brake Actuator Lag (tau = 0.10 s)", fontsize=12, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.5, axis="y")
    ax.legend(framealpha=0.9, fontsize=10)

    fig.tight_layout()
    fig.savefig(os.path.join(plots_dir, "max_actual_decel_vs_braking_level.png"), dpi=300)
    plt.close(fig)


def plot_stopping_distance_vs_braking(summary_records: List[Dict[str, Any]], plots_dir: str) -> None:
    """Plot 4: Stopping Distance vs Commanded Braking Level Variations (CCR only, where stop occurs)."""
    brake_configs = [
        "Baseline",
        "Brake_L1_-1.5", "Brake_L1_-2.5",
        "Brake_L2_-3.0", "Brake_L2_-5.0",
        "Brake_L3_-5.0", "Brake_L3_-7.0",
        "Brake_Mild_Gentle", "Brake_Firm_Aggressive"
    ]

    ccr_recs = {r["config_name"]: r for r in summary_records if r["scenario"] == "CCR_Stationary_Target"}
    distances = [ccr_recs[c]["stopping_distance_mean"] if c in ccr_recs and ccr_recs[c]["stopping_distance_mean"] is not None else 0.0 for c in brake_configs]
    stds = [ccr_recs[c]["stopping_distance_std"] if c in ccr_recs and ccr_recs[c]["stopping_distance_std"] is not None else 0.0 for c in brake_configs]

    fig, ax = plt.subplots(figsize=(11, 5.5))
    x = np.arange(len(brake_configs))
    bars = ax.bar(x, distances, yerr=stds, capsize=4, color="#3b528b", alpha=0.85, edgecolor="black", linewidth=0.6)

    # Highlight baseline bar
    bars[0].set_color("#d95f02")

    ax.set_xticks(x)
    ax.set_xticklabels(brake_configs, rotation=35, ha="right", fontsize=9)
    ax.set_ylabel("Ego Stopping Distance (m)", fontsize=11, fontweight="bold")
    ax.set_title("CCR Scenario Ego Stopping Distance vs. Braking Demands", fontsize=12, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.5, axis="y")
    if distances and max(distances) > 0:
        ax.set_ylim(0, max(distances) * 1.25)

    for bar, d in zip(bars, distances):
        ax.text(bar.get_x() + bar.get_width() / 2.0, d + 0.3, f"{d:.2f}m", ha="center", va="bottom", fontsize=8)

    fig.subplots_adjust(bottom=0.22, top=0.92)
    fig.savefig(os.path.join(plots_dir, "stopping_distance_vs_braking_level.png"), dpi=300)
    plt.close(fig)


def plot_state_transitions_vs_ttc(summary_records: List[Dict[str, Any]], plots_dir: str) -> None:
    """Plot 5: FSM State Transition Count Across TTC Threshold Variations."""
    ttc_configs = [
        "Baseline",
        "TTC_Warn_3.5s", "TTC_Warn_4.5s",
        "TTC_L1_2.5s", "TTC_L1_3.5s",
        "TTC_L2_2.0s", "TTC_L2_2.5s",
        "TTC_L3_1.5s", "TTC_L3_2.0s",
        "TTC_Early_Conservative", "TTC_Late_Aggressive"
    ]
    scenarios = ["CCR_Stationary_Target", "CCRm_Moving_Target", "CCRb_Braking_Target"]
    sc_labels = {"CCR_Stationary_Target": "CCR", "CCRm_Moving_Target": "CCRm", "CCRb_Braking_Target": "CCRb"}
    colors = {"CCR_Stationary_Target": "#1f77b4", "CCRm_Moving_Target": "#2ca02c", "CCRb_Braking_Target": "#d62728"}

    fig, ax = plt.subplots(figsize=(11, 5.5))
    x = np.arange(len(ttc_configs))
    width = 0.26

    for idx, sc in enumerate(scenarios):
        sc_recs = {r["config_name"]: r for r in summary_records if r["scenario"] == sc}
        transitions = [sc_recs[c]["state_transition_count_mean"] if c in sc_recs else 0.0 for c in ttc_configs]
        ax.bar(
            x + (idx - 1) * width,
            transitions,
            width=width,
            label=sc_labels[sc],
            color=colors[sc],
            alpha=0.85,
            edgecolor="black",
            linewidth=0.5,
        )

    ax.set_xticks(x)
    ax.set_xticklabels(ttc_configs, rotation=35, ha="right", fontsize=9)
    ax.set_ylabel("FSM State Transitions (Count)", fontsize=11, fontweight="bold")
    ax.set_title("FSM Switching Activity vs. TTC Threshold Sets (Noise sigma = 0.05 m)", fontsize=12, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.5, axis="y")
    ax.legend(framealpha=0.9, fontsize=10)

    fig.tight_layout()
    fig.savefig(os.path.join(plots_dir, "state_transitions_vs_ttc_threshold.png"), dpi=300)
    plt.close(fig)


def plot_triplet_reversals_vs_ttc(summary_records: List[Dict[str, Any]], plots_dir: str) -> None:
    """Plot 6: Triplet Reversal Patterns vs TTC Threshold Variations."""
    ttc_configs = [
        "Baseline",
        "TTC_Warn_3.5s", "TTC_Warn_4.5s",
        "TTC_L1_2.5s", "TTC_L1_3.5s",
        "TTC_L2_2.0s", "TTC_L2_2.5s",
        "TTC_L3_1.5s", "TTC_L3_2.0s",
        "TTC_Early_Conservative", "TTC_Late_Aggressive"
    ]
    scenarios = ["CCR_Stationary_Target", "CCRm_Moving_Target", "CCRb_Braking_Target"]
    sc_labels = {"CCR_Stationary_Target": "CCR", "CCRm_Moving_Target": "CCRm", "CCRb_Braking_Target": "CCRb"}
    colors = {"CCR_Stationary_Target": "#1f77b4", "CCRm_Moving_Target": "#2ca02c", "CCRb_Braking_Target": "#d62728"}

    fig, ax = plt.subplots(figsize=(11, 5.5))
    x = np.arange(len(ttc_configs))
    width = 0.26

    for idx, sc in enumerate(scenarios):
        sc_recs = {r["config_name"]: r for r in summary_records if r["scenario"] == sc}
        reversals = [sc_recs[c]["triplet_reversal_count_mean"] if c in sc_recs else 0.0 for c in ttc_configs]
        ax.bar(
            x + (idx - 1) * width,
            reversals,
            width=width,
            label=sc_labels[sc],
            color=colors[sc],
            alpha=0.85,
            edgecolor="black",
            linewidth=0.5,
        )

    ax.set_xticks(x)
    ax.set_xticklabels(ttc_configs, rotation=35, ha="right", fontsize=9)
    ax.set_ylabel("Triplet Reversals (3-sample sliding count)", fontsize=11, fontweight="bold")
    ax.set_title("Triplet Reversal Count vs. TTC Threshold Sets (Excl. SAFE->X->SAFE)", fontsize=12, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.5, axis="y")
    ax.legend(framealpha=0.9, fontsize=10)

    fig.tight_layout()
    fig.savefig(os.path.join(plots_dir, "triplet_reversal_count_vs_ttc_threshold.png"), dpi=300)
    plt.close(fig)


def plot_baseline_summary_comparison(summary_records: List[Dict[str, Any]], plots_dir: str) -> None:
    """Plot 7: Comprehensive Summary Comparison of Baseline vs Key Tested Parameter Configurations."""
    key_configs = [
        "Baseline",
        "TTC_Early_Conservative",
        "TTC_Late_Aggressive",
        "Brake_Mild_Gentle",
        "Brake_Firm_Aggressive",
        "Joint_Early_Firm",
        "Joint_Late_Mild",
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    # Panel A: Minimum Distance (CCR)
    ccr_recs = {r["config_name"]: r for r in summary_records if r["scenario"] == "CCR_Stationary_Target"}
    d_means = [ccr_recs[c]["minimum_distance_mean"] if c in ccr_recs else 0.0 for c in key_configs]
    bars_a = axes[0, 0].barh(key_configs, d_means, color="#1f77b4", alpha=0.85, edgecolor="black", linewidth=0.5)
    bars_a[0].set_color("#d95f02")  # Highlight baseline
    axes[0, 0].set_xlabel("Minimum Separation Distance (m)", fontweight="bold")
    axes[0, 0].set_title("(A) Safety Margin: Min Distance (CCR)", fontweight="bold")
    axes[0, 0].grid(True, linestyle="--", alpha=0.5, axis="x")

    # Panel B: Stopping Distance (CCR)
    s_means = [ccr_recs[c]["stopping_distance_mean"] if c in ccr_recs and ccr_recs[c]["stopping_distance_mean"] is not None else 0.0 for c in key_configs]
    bars_b = axes[0, 1].barh(key_configs, s_means, color="#2ca02c", alpha=0.85, edgecolor="black", linewidth=0.5)
    bars_b[0].set_color("#d95f02")
    axes[0, 1].set_xlabel("Ego Stopping Distance (m)", fontweight="bold")
    axes[0, 1].set_title("(B) Braking Authority: Stopping Distance (CCR)", fontweight="bold")
    axes[0, 1].grid(True, linestyle="--", alpha=0.5, axis="x")

    # Panel C: Delivered Deceleration (CCR)
    a_means = [ccr_recs[c]["max_actual_deceleration_mean"] if c in ccr_recs else 0.0 for c in key_configs]
    bars_c = axes[1, 0].barh(key_configs, a_means, color="#9467bd", alpha=0.85, edgecolor="black", linewidth=0.5)
    bars_c[0].set_color("#d95f02")
    axes[1, 0].set_xlabel("Max Delivered Deceleration (m/s²)", fontweight="bold")
    axes[1, 0].set_title("(C) Braking Severity: Peak Deceleration (CCR)", fontweight="bold")
    axes[1, 0].grid(True, linestyle="--", alpha=0.5, axis="x")

    # Panel D: Triplet Reversal Patterns (CCR)
    r_means = [ccr_recs[c]["triplet_reversal_count_mean"] if c in ccr_recs else 0.0 for c in key_configs]
    bars_d = axes[1, 1].barh(key_configs, r_means, color="#e377c2", alpha=0.85, edgecolor="black", linewidth=0.5)
    bars_d[0].set_color("#d95f02")
    axes[1, 1].set_xlabel("Triplet Reversal Count", fontweight="bold")
    axes[1, 1].set_title("(D) Controller Stability: Triplet Reversals (CCR)", fontweight="bold")
    axes[1, 1].grid(True, linestyle="--", alpha=0.5, axis="x")

    fig.suptitle("AEB Parameter Sensitivity: Comprehensive Baseline Comparison", fontsize=14, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(plots_dir, "baseline_vs_parameters_summary.png"), dpi=300)
    plt.close(fig)


def generate_all_sensitivity_plots(summary_records: List[Dict[str, Any]], plots_dir: str) -> None:
    """Generate all required parameter sensitivity figures."""
    os.makedirs(plots_dir, exist_ok=True)
    plot_min_distance_vs_ttc(summary_records, plots_dir)
    plot_activation_time_vs_ttc(summary_records, plots_dir)
    plot_max_actual_decel_vs_braking(summary_records, plots_dir)
    plot_stopping_distance_vs_braking(summary_records, plots_dir)
    plot_state_transitions_vs_ttc(summary_records, plots_dir)
    plot_triplet_reversals_vs_ttc(summary_records, plots_dir)
    plot_baseline_summary_comparison(summary_records, plots_dir)


if __name__ == "__main__":
    print("=" * 80)
    print("STEP 9 — AEB THRESHOLD AND BRAKING PARAMETER SENSITIVITY STUDY")
    print("=" * 80)
    raw, summary = run_aeb_parameter_sensitivity()
    print(f"Completed {len(raw)} simulation runs across {len(summary)} aggregated configurations.")
    print("Results written to results/aeb_parameter_sensitivity/")
    print("Plots generated in plots/aeb_parameter_sensitivity/")
