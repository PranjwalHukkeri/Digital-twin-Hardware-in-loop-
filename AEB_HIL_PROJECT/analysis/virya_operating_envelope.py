"""
VIRYA Operating-Envelope Validation Module (Step 10).

Evaluates the provisional AEB software baseline:
- TTC thresholds: Warning = 4.00 s, Level 1 = 3.00 s, Level 2 = 2.25 s, Level 3 = 1.75 s
- Deceleration demands: Level 1 = -2.0 m/s², Level 2 = -4.0 m/s², Level 3 = -6.0 m/s²
- Minimum brake duration: 0.20 s

Across the low-speed operating envelope relevant to the VIRYA DBW skateboard platform:
- Speeds: [0.50, 1.00, 1.39] m/s (approx 1.8, 3.6, 5.0 km/h)
- Initial distances: [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0] m
- Scenarios: CCR (stationary), CCRm (moving), CCRb (braking target)
- Seeds: [42, 43, 44, 45, 46, 47, 48, 49, 50, 51]
- Nominal sensor noise: sigma = 0.05 m
- Nominal actuator lag: tau = 0.10 s
- Sensor latency: 0.0 s
- Timestep: dt = 0.05 s

Total experiment size: 3 speeds × 8 distances × 3 scenarios × 10 seeds = 720 runs.

Note:
This is a discrete-time Python software simulation study characterizing operating-envelope behavior.
It is NOT Hardware-in-the-Loop (HIL) validation, physical VIRYA testing, brake actuator characterization,
or regulatory certification.
"""

import os
import csv
import math
from dataclasses import dataclass, field
from typing import List, Dict, Any, Tuple, Optional
from collections import Counter
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.simulation import SimulationEngine, SimulationResult
from src.metrics import PerformanceEvaluator, AEBPerformanceMetrics
from scenarios.scenarios import (
    CCRScenario,
    CCRmScenario,
    CCRbScenario,
    Scenario,
)
from analysis.sensitivity_analysis import (
    count_fsm_state_transitions,
    count_triplet_reversal_patterns,
)


# ==============================================================================
# OPERATING ENVELOPE CONFIGURATION CONSTANTS
# ==============================================================================

VIRYA_SPEEDS = [0.50, 1.00, 1.39]  # m/s (~1.8, 3.6, 5.0 km/h)
INITIAL_DISTANCES = [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0]  # m
SCENARIO_TYPES = ["CCR", "CCRm", "CCRb"]
SEEDS = [42, 43, 44, 45, 46, 47, 48, 49, 50, 51]

NOMINAL_DISTANCE_NOISE_STD = 0.05  # m
NOMINAL_ACTUATOR_TAU = 0.10        # s
NOMINAL_SENSOR_LATENCY = 0.00      # s
SIMULATION_DT = 0.05               # s

# CCRb target braking specification
CCRB_TARGET_DECEL = -2.0           # m/s²
CCRB_BRAKE_START_TIME = 1.0        # s

FSM_LEVEL_ORDER = {
    "SAFE": 0,
    "WARNING": 1,
    "AEB_LEVEL_1": 2,
    "AEB_LEVEL_2": 3,
    "AEB_LEVEL_3": 4,
}
FSM_LEVEL_NAMES = {v: k for k, v in FSM_LEVEL_ORDER.items()}


def create_scenario(scenario_type: str, ego_speed: float, distance: float) -> Scenario:
    """
    Factory creating standardized low-speed scenarios.

    Args:
        scenario_type: One of 'CCR', 'CCRm', 'CCRb'.
        ego_speed: Initial ego vehicle speed (m/s).
        distance: Initial separation distance (m).

    Returns:
        Scenario instance.
    """
    if scenario_type == "CCR":
        return CCRScenario(
            name=f"CCR_{ego_speed:.2f}mps_{distance:.1f}m",
            initial_ego_speed=ego_speed,
            initial_distance=distance,
            simulation_duration=10.0,
        )
    elif scenario_type == "CCRm":
        target_speed = 0.5 * ego_speed
        return CCRmScenario(
            name=f"CCRm_{ego_speed:.2f}mps_{distance:.1f}m",
            initial_ego_speed=ego_speed,
            initial_target_speed=target_speed,
            initial_distance=distance,
            simulation_duration=10.0,
        )
    elif scenario_type == "CCRb":
        target_speed = 0.5 * ego_speed
        return CCRbScenario(
            name=f"CCRb_{ego_speed:.2f}mps_{distance:.1f}m",
            initial_ego_speed=ego_speed,
            initial_target_speed=target_speed,
            initial_distance=distance,
            target_decel=CCRB_TARGET_DECEL,
            brake_start_time=CCRB_BRAKE_START_TIME,
            simulation_duration=10.0,
        )
    else:
        raise ValueError(f"Unknown scenario type: {scenario_type}")


def compute_theoretical_initial_ttc(scenario_type: str, ego_speed: float, distance: float) -> float:
    """
    Compute theoretical initial Time-To-Collision (TTC) for kinematic baseline check.

    CCR: TTC = d0 / v_ego
    CCRm: TTC = d0 / (v_ego - v_target) = d0 / (0.5 * v_ego)
    CCRb: Prior to target braking (t < 1.0s), closing speed is 0.5 * v_ego.
    """
    if scenario_type == "CCR":
        v_rel = ego_speed
    else:
        v_rel = 0.5 * ego_speed

    if v_rel <= 1e-6:
        return float("inf")
    return float(distance / v_rel)


def get_max_fsm_level_reached(result: SimulationResult) -> str:
    """Determine highest FSM threat/braking level reached during simulation."""
    if not result.samples:
        return "SAFE"
    max_rank = 0
    for s in result.samples:
        rank = FSM_LEVEL_ORDER.get(s.aeb_state, 0)
        if rank > max_rank:
            max_rank = rank
    return FSM_LEVEL_NAMES.get(max_rank, "SAFE")


@dataclass
class OperatingEnvelopeRunResult:
    """Container for a single simulation run within the operating envelope."""
    scenario_type: str
    scenario_name: str
    ego_speed: float
    initial_distance: float
    target_speed_initial: float
    seed: int
    theoretical_initial_ttc: float
    min_distance: float
    collision_status: bool
    aeb_activation_time: Optional[float]
    ttc_at_activation: Optional[float]
    max_commanded_decel: float
    max_actual_decel: float
    response_time_90: Optional[float]
    stopping_time: Optional[float]
    stopping_distance: Optional[float]
    final_ego_speed: float
    final_relative_velocity: float
    min_relative_velocity: float
    vehicle_stop_status: str
    fsm_state_transitions: int
    triplet_reversal_count: int
    max_fsm_level_reached: str
    activated_braking: bool
    stopping_feasible: bool

    def to_dict(self) -> Dict[str, Any]:
        return {
            "scenario_type": self.scenario_type,
            "scenario_name": self.scenario_name,
            "ego_speed": self.ego_speed,
            "initial_distance": self.initial_distance,
            "target_speed_initial": self.target_speed_initial,
            "seed": self.seed,
            "theoretical_initial_ttc": self.theoretical_initial_ttc,
            "min_distance": self.min_distance,
            "collision_status": self.collision_status,
            "aeb_activation_time": self.aeb_activation_time if self.aeb_activation_time is not None else "",
            "ttc_at_activation": self.ttc_at_activation if self.ttc_at_activation is not None else "",
            "max_commanded_decel": self.max_commanded_decel,
            "max_actual_decel": self.max_actual_decel,
            "response_time_90": self.response_time_90 if self.response_time_90 is not None else "",
            "stopping_time": self.stopping_time if self.stopping_time is not None else "",
            "stopping_distance": self.stopping_distance if self.stopping_distance is not None else "",
            "final_ego_speed": self.final_ego_speed,
            "final_relative_velocity": self.final_relative_velocity,
            "min_relative_velocity": self.min_relative_velocity,
            "vehicle_stop_status": self.vehicle_stop_status,
            "fsm_state_transitions": self.fsm_state_transitions,
            "triplet_reversal_count": self.triplet_reversal_count,
            "max_fsm_level_reached": self.max_fsm_level_reached,
            "activated_braking": self.activated_braking,
            "stopping_feasible": self.stopping_feasible,
        }


def run_single_simulation(
    scenario_type: str,
    ego_speed: float,
    distance: float,
    seed: int,
    distance_noise_std: float = NOMINAL_DISTANCE_NOISE_STD,
    actuator_tau: float = NOMINAL_ACTUATOR_TAU,
    sensor_latency: float = NOMINAL_SENSOR_LATENCY,
    dt: float = SIMULATION_DT,
) -> OperatingEnvelopeRunResult:
    """Execute a single closed-loop simulation under the production AEB baseline."""
    scenario = create_scenario(scenario_type, ego_speed, distance)
    engine = SimulationEngine(
        dt=dt,
        distance_noise_std=distance_noise_std,
        distance_seed=seed,
        actuator_tau=actuator_tau,
        sensor_latency=sensor_latency,
    )
    result = engine.run(scenario=scenario)
    metrics: AEBPerformanceMetrics = PerformanceEvaluator.evaluate(result)

    transitions = count_fsm_state_transitions(result)
    triplets = count_triplet_reversal_patterns(result)
    max_level = get_max_fsm_level_reached(result)
    activated_braking = max_level in {"AEB_LEVEL_1", "AEB_LEVEL_2", "AEB_LEVEL_3"}

    # Scenario-aware stopping feasibility:
    # CCR and CCRb require full vehicle standstill behind stationary obstacle without collision.
    # CCRm requires safe speed matching or falling below moving target speed (v_rel <= 0.0) without collision.
    if scenario_type in {"CCR", "CCRb"}:
        stopping_feasible = (metrics.vehicle_stop_status == "STOPPED") and (not metrics.collision_status)
    elif scenario_type == "CCRm":
        stopping_feasible = (metrics.final_relative_velocity <= 0.0) and (not metrics.collision_status)
    else:
        stopping_feasible = (metrics.vehicle_stop_status == "STOPPED") and (not metrics.collision_status)

    theoretical_ttc = compute_theoretical_initial_ttc(scenario_type, ego_speed, distance)

    target_v_init = 0.0 if scenario_type == "CCR" else 0.5 * ego_speed

    return OperatingEnvelopeRunResult(
        scenario_type=scenario_type,
        scenario_name=scenario.name,
        ego_speed=ego_speed,
        initial_distance=distance,
        target_speed_initial=target_v_init,
        seed=seed,
        theoretical_initial_ttc=round(theoretical_ttc, 4),
        min_distance=round(metrics.min_distance, 4),
        collision_status=metrics.collision_status,
        aeb_activation_time=round(metrics.aeb_activation_time, 4) if metrics.aeb_activation_time is not None else None,
        ttc_at_activation=round(metrics.ttc_at_activation, 4) if metrics.ttc_at_activation is not None else None,
        max_commanded_decel=round(metrics.max_braking_deceleration, 4),
        max_actual_decel=round(metrics.max_actual_deceleration, 4),
        response_time_90=round(metrics.ninety_percent_braking_response_time, 4) if metrics.ninety_percent_braking_response_time is not None else None,
        stopping_time=round(metrics.stopping_time, 4) if metrics.stopping_time is not None else None,
        stopping_distance=round(metrics.stopping_distance, 4) if metrics.stopping_distance is not None else None,
        final_ego_speed=round(metrics.final_ego_speed, 4),
        final_relative_velocity=round(metrics.final_relative_velocity, 4),
        min_relative_velocity=round(metrics.min_relative_velocity, 4),
        vehicle_stop_status=metrics.vehicle_stop_status,
        fsm_state_transitions=transitions,
        triplet_reversal_count=triplets,
        max_fsm_level_reached=max_level,
        activated_braking=activated_braking,
        stopping_feasible=stopping_feasible,
    )


def run_operating_envelope_study(
    speeds: List[float] = VIRYA_SPEEDS,
    distances: List[float] = INITIAL_DISTANCES,
    scenarios: List[str] = SCENARIO_TYPES,
    seeds: List[int] = SEEDS,
) -> Tuple[List[OperatingEnvelopeRunResult], List[Dict[str, Any]]]:
    """
    Execute full grid: len(speeds) × len(distances) × len(scenarios) × len(seeds).

    Returns:
        (raw_results, summary_results)
    """
    raw_results: List[OperatingEnvelopeRunResult] = []

    print(f"Starting VIRYA Operating-Envelope Study...")
    print(f"  Speeds: {speeds} m/s")
    print(f"  Distances: {distances} m")
    print(f"  Scenarios: {scenarios}")
    print(f"  Seeds: {len(seeds)} seeds ({seeds[0]}..{seeds[-1]})")
    total_expected = len(speeds) * len(distances) * len(scenarios) * len(seeds)
    print(f"  Total planned runs: {total_expected}")

    for scen in scenarios:
        for v in speeds:
            for d in distances:
                for seed in seeds:
                    run_res = run_single_simulation(scen, v, d, seed)
                    raw_results.append(run_res)

    print(f"Completed {len(raw_results)} runs.")

    # Aggregate summary per (scenario, speed, distance)
    summary_results: List[Dict[str, Any]] = []

    # Grouping
    groups: Dict[Tuple[str, float, float], List[OperatingEnvelopeRunResult]] = {}
    for r in raw_results:
        key = (r.scenario_type, r.ego_speed, r.initial_distance)
        if key not in groups:
            groups[key] = []
        groups[key].append(r)

    def stats(vals: List[float]) -> Tuple[float, float, float, float]:
        if not vals:
            return 0.0, 0.0, 0.0, 0.0
        arr = np.array(vals, dtype=float)
        return float(np.mean(arr)), float(np.std(arr, ddof=0)), float(np.min(arr)), float(np.max(arr))

    for (scen, v, d), runs in groups.items():
        n = len(runs)
        collisions = sum(1 for r in runs if r.collision_status)
        stopping_feas = sum(1 for r in runs if r.stopping_feasible)
        act_braking = sum(1 for r in runs if r.activated_braking)

        min_dist_mean, min_dist_std, min_dist_min, min_dist_max = stats([r.min_distance for r in runs])

        act_times = [r.aeb_activation_time for r in runs if r.aeb_activation_time is not None]
        act_t_mean, act_t_std, act_t_min, act_t_max = stats(act_times) if act_times else (None, None, None, None)

        ttc_acts = [r.ttc_at_activation for r in runs if r.ttc_at_activation is not None]
        ttc_mean, ttc_std, ttc_min, ttc_max = stats(ttc_acts) if ttc_acts else (None, None, None, None)

        cmd_decel_mean, cmd_decel_std, cmd_decel_min, cmd_decel_max = stats([r.max_commanded_decel for r in runs])
        act_decel_mean, act_decel_std, act_decel_min, act_decel_max = stats([r.max_actual_decel for r in runs])

        resp_90s = [r.response_time_90 for r in runs if r.response_time_90 is not None]
        resp_mean, resp_std, resp_min, resp_max = stats(resp_90s) if resp_90s else (None, None, None, None)

        stop_times = [r.stopping_time for r in runs if r.stopping_time is not None]
        stop_t_mean, stop_t_std, stop_t_min, stop_t_max = stats(stop_times) if stop_times else (None, None, None, None)

        stop_dists = [r.stopping_distance for r in runs if r.stopping_distance is not None]
        stop_d_mean, stop_d_std, stop_d_min, stop_d_max = stats(stop_dists) if stop_dists else (None, None, None, None)

        fsm_trans_mean, fsm_trans_std, fsm_trans_min, fsm_trans_max = stats([float(r.fsm_state_transitions) for r in runs])
        triplet_mean, triplet_std, triplet_min, triplet_max = stats([float(r.triplet_reversal_count) for r in runs])

        v_ego_final_mean, v_ego_final_std, _, _ = stats([r.final_ego_speed for r in runs])
        v_rel_final_mean, v_rel_final_std, _, _ = stats([r.final_relative_velocity for r in runs])
        v_rel_min_mean, v_rel_min_std, _, _ = stats([r.min_relative_velocity for r in runs])

        # Most frequent FSM level
        levels = [r.max_fsm_level_reached for r in runs]
        most_freq_level = Counter(levels).most_common(1)[0][0]

        summary_results.append({
            "scenario_type": scen,
            "ego_speed": v,
            "initial_distance": d,
            "theoretical_initial_ttc": runs[0].theoretical_initial_ttc,
            "runs_count": n,
            "collision_count": collisions,
            "collision_rate": round(collisions / n, 4),
            "stopping_feasible_count": stopping_feas,
            "stopping_feasible_rate": round(stopping_feas / n, 4),
            "activated_braking_count": act_braking,
            "activated_braking_rate": round(act_braking / n, 4),
            "most_frequent_max_fsm_level": most_freq_level,
            "min_distance_mean": round(min_dist_mean, 4),
            "min_distance_std": round(min_dist_std, 4),
            "min_distance_min": round(min_dist_min, 4),
            "min_distance_max": round(min_dist_max, 4),
            "activation_time_mean": round(act_t_mean, 4) if act_t_mean is not None else "",
            "activation_time_std": round(act_t_std, 4) if act_t_std is not None else "",
            "activation_time_min": round(act_t_min, 4) if act_t_min is not None else "",
            "activation_time_max": round(act_t_max, 4) if act_t_max is not None else "",
            "ttc_at_activation_mean": round(ttc_mean, 4) if ttc_mean is not None else "",
            "ttc_at_activation_std": round(ttc_std, 4) if ttc_std is not None else "",
            "ttc_at_activation_min": round(ttc_min, 4) if ttc_min is not None else "",
            "ttc_at_activation_max": round(ttc_max, 4) if ttc_max is not None else "",
            "max_commanded_decel_mean": round(cmd_decel_mean, 4),
            "max_commanded_decel_std": round(cmd_decel_std, 4),
            "max_actual_decel_mean": round(act_decel_mean, 4),
            "max_actual_decel_std": round(act_decel_std, 4),
            "response_time_90_mean": round(resp_mean, 4) if resp_mean is not None else "",
            "response_time_90_std": round(resp_std, 4) if resp_std is not None else "",
            "stopping_time_mean": round(stop_t_mean, 4) if stop_t_mean is not None else "",
            "stopping_time_std": round(stop_t_std, 4) if stop_t_std is not None else "",
            "stopping_distance_mean": round(stop_d_mean, 4) if stop_d_mean is not None else "",
            "stopping_distance_std": round(stop_d_std, 4) if stop_d_std is not None else "",
            "final_ego_speed_mean": round(v_ego_final_mean, 4),
            "final_relative_velocity_mean": round(v_rel_final_mean, 4),
            "min_relative_velocity_mean": round(v_rel_min_mean, 4),
            "fsm_state_transitions_mean": round(fsm_trans_mean, 4),
            "fsm_state_transitions_std": round(fsm_trans_std, 4),
            "triplet_reversal_count_mean": round(triplet_mean, 4),
            "triplet_reversal_count_std": round(triplet_std, 4),
            "triplet_reversal_count_max": round(triplet_max, 4),
        })

    return raw_results, summary_results


def export_csvs(
    raw_results: List[OperatingEnvelopeRunResult],
    summary_results: List[Dict[str, Any]],
    output_dir: str = "results/virya_operating_envelope",
) -> Tuple[str, str]:
    """Export raw and summary results to CSV files."""
    os.makedirs(output_dir, exist_ok=True)
    raw_path = os.path.join(output_dir, "virya_operating_envelope_raw.csv")
    summary_path = os.path.join(output_dir, "virya_operating_envelope_summary.csv")

    if raw_results:
        fieldnames = list(raw_results[0].to_dict().keys())
        with open(raw_path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in raw_results:
                writer.writerow(r.to_dict())

    if summary_results:
        fieldnames_sum = list(summary_results[0].keys())
        with open(summary_path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames_sum)
            writer.writeheader()
            for s in summary_results:
                writer.writerow(s)

    print(f"Exported raw results: {raw_path} ({len(raw_results)} rows)")
    print(f"Exported summary results: {summary_path} ({len(summary_results)} rows)")
    return raw_path, summary_path


# ==============================================================================
# PLOTTING FUNCTIONS
# ==============================================================================

SCENARIO_COLORS = {
    "CCR": "#1f77b4",   # Blue
    "CCRm": "#2ca02c",  # Green
    "CCRb": "#d62728",  # Red
}

SPEED_COLORS = {
    0.50: "#1f77b4",
    1.00: "#ff7f0e",
    1.39: "#2ca02c",
}


def plot_all_figures(
    summary_results: List[Dict[str, Any]],
    raw_results: List[OperatingEnvelopeRunResult],
    output_dir: str = "plots/virya_operating_envelope",
) -> List[str]:
    """Generate all 8 required publication-quality plots."""
    os.makedirs(output_dir, exist_ok=True)
    created_plots: List[str] = []

    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
    plt.rcParams.update({
        "font.size": 11,
        "axes.labelsize": 12,
        "axes.titlesize": 13,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "legend.fontsize": 10,
        "figure.titlesize": 14,
    })

    # Convert summary to indexed lookup
    sum_map = {(s["scenario_type"], s["ego_speed"], s["initial_distance"]): s for s in summary_results}

    # 1. min_distance_vs_speed.png
    p1 = os.path.join(output_dir, "min_distance_vs_speed.png")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    fig.suptitle("Minimum Separation Distance vs Ego Approach Speed across Low-Speed Envelope", fontsize=14, y=0.98)
    for idx, scen in enumerate(SCENARIO_TYPES):
        ax = axes[idx]
        ax.set_title(f"Scenario: {scen}")
        ax.set_xlabel("Ego Speed (m/s)")
        if idx == 0:
            ax.set_ylabel("Minimum Distance (m)")
        for d in INITIAL_DISTANCES:
            speeds_plot = []
            min_dists = []
            stds = []
            for v in VIRYA_SPEEDS:
                entry = sum_map.get((scen, v, d))
                if entry:
                    speeds_plot.append(v)
                    min_dists.append(entry["min_distance_mean"])
                    stds.append(entry["min_distance_std"])
            ax.errorbar(speeds_plot, min_dists, yerr=stds, marker="o", capsize=3, label=f"d0={d:.1f}m")
        ax.set_xticks(VIRYA_SPEEDS)
        ax.set_xticklabels(["0.50 m/s\n(1.8 km/h)", "1.00 m/s\n(3.6 km/h)", "1.39 m/s\n(5.0 km/h)"])
        ax.grid(True, alpha=0.3)
        if idx == 2:
            ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left", title="Initial Distance")
    plt.tight_layout()
    plt.savefig(p1, dpi=300, bbox_inches="tight")
    plt.close()
    created_plots.append(p1)

    # 2. min_distance_vs_initial_distance.png
    p2 = os.path.join(output_dir, "min_distance_vs_initial_distance.png")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    fig.suptitle("Minimum Separation Distance vs Initial Distance", fontsize=14, y=0.98)
    for idx, scen in enumerate(SCENARIO_TYPES):
        ax = axes[idx]
        ax.set_title(f"Scenario: {scen}")
        ax.set_xlabel("Initial Separation Distance (m)")
        if idx == 0:
            ax.set_ylabel("Minimum Distance (m)")
        for v in VIRYA_SPEEDS:
            dists_plot = []
            min_dists = []
            stds = []
            for d in INITIAL_DISTANCES:
                entry = sum_map.get((scen, v, d))
                if entry:
                    dists_plot.append(d)
                    min_dists.append(entry["min_distance_mean"])
                    stds.append(entry["min_distance_std"])
            ax.errorbar(dists_plot, min_dists, yerr=stds, marker="s", capsize=3, color=SPEED_COLORS[v], label=f"v={v:.2f} m/s")
        ax.set_xticks(INITIAL_DISTANCES)
        ax.grid(True, alpha=0.3)
        if idx == 2:
            ax.legend(loc="lower right", title="Approach Speed")
    plt.tight_layout()
    plt.savefig(p2, dpi=300, bbox_inches="tight")
    plt.close()
    created_plots.append(p2)

    # 3. activation_time_vs_speed.png
    p3 = os.path.join(output_dir, "activation_time_vs_speed.png")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    fig.suptitle("AEB Activation Time vs Approach Speed", fontsize=14, y=0.98)
    for idx, scen in enumerate(SCENARIO_TYPES):
        ax = axes[idx]
        ax.set_title(f"Scenario: {scen}")
        ax.set_xlabel("Ego Speed (m/s)")
        if idx == 0:
            ax.set_ylabel("AEB Activation Time (s)")
        for d in INITIAL_DISTANCES:
            speeds_plot = []
            act_times = []
            for v in VIRYA_SPEEDS:
                entry = sum_map.get((scen, v, d))
                if entry and entry["activation_time_mean"] != "":
                    speeds_plot.append(v)
                    act_times.append(float(entry["activation_time_mean"]))
            if speeds_plot:
                ax.plot(speeds_plot, act_times, marker="o", label=f"d0={d:.1f}m")
        ax.set_xticks(VIRYA_SPEEDS)
        ax.set_xticklabels(["0.50", "1.00", "1.39"])
        ax.grid(True, alpha=0.3)
        if idx == 2:
            ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left", title="Initial Distance")
    plt.tight_layout()
    plt.savefig(p3, dpi=300, bbox_inches="tight")
    plt.close()
    created_plots.append(p3)

    # 4. ttc_at_activation_vs_speed.png
    p4 = os.path.join(output_dir, "ttc_at_activation_vs_speed.png")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    fig.suptitle("TTC at AEB Activation vs Approach Speed", fontsize=14, y=0.98)
    for idx, scen in enumerate(SCENARIO_TYPES):
        ax = axes[idx]
        ax.set_title(f"Scenario: {scen}")
        ax.set_xlabel("Ego Speed (m/s)")
        if idx == 0:
            ax.set_ylabel("TTC at Activation (s)")
        for d in INITIAL_DISTANCES:
            speeds_plot = []
            ttc_vals = []
            for v in VIRYA_SPEEDS:
                entry = sum_map.get((scen, v, d))
                if entry and entry["ttc_at_activation_mean"] != "":
                    speeds_plot.append(v)
                    ttc_vals.append(float(entry["ttc_at_activation_mean"]))
            if speeds_plot:
                ax.plot(speeds_plot, ttc_vals, marker="^", label=f"d0={d:.1f}m")
        ax.axhline(3.00, color="gray", linestyle="--", alpha=0.7, label="L1 Threshold (3.0s)")
        ax.axhline(1.75, color="darkred", linestyle=":", alpha=0.7, label="L3 Threshold (1.75s)")
        ax.set_xticks(VIRYA_SPEEDS)
        ax.set_xticklabels(["0.50", "1.00", "1.39"])
        ax.grid(True, alpha=0.3)
        if idx == 2:
            ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left", title="Legend")
    plt.tight_layout()
    plt.savefig(p4, dpi=300, bbox_inches="tight")
    plt.close()
    created_plots.append(p4)

    # 5. max_actual_decel_vs_speed.png
    p5 = os.path.join(output_dir, "max_actual_decel_vs_speed.png")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    fig.suptitle("Maximum Actual Deceleration Delivered vs Approach Speed", fontsize=14, y=0.98)
    for idx, scen in enumerate(SCENARIO_TYPES):
        ax = axes[idx]
        ax.set_title(f"Scenario: {scen}")
        ax.set_xlabel("Ego Speed (m/s)")
        if idx == 0:
            ax.set_ylabel("Max Actual Deceleration Magnitude (m/s²)")
        for d in INITIAL_DISTANCES:
            speeds_plot = []
            decels = []
            for v in VIRYA_SPEEDS:
                entry = sum_map.get((scen, v, d))
                if entry:
                    speeds_plot.append(v)
                    decels.append(entry["max_actual_decel_mean"])
            ax.plot(speeds_plot, decels, marker="d", label=f"d0={d:.1f}m")
        ax.axhline(2.0, color="orange", linestyle="--", alpha=0.5, label="L1 Target (-2.0)")
        ax.axhline(4.0, color="green", linestyle="--", alpha=0.5, label="L2 Target (-4.0)")
        ax.axhline(6.0, color="red", linestyle="--", alpha=0.5, label="L3 Target (-6.0)")
        ax.set_xticks(VIRYA_SPEEDS)
        ax.set_xticklabels(["0.50", "1.00", "1.39"])
        ax.grid(True, alpha=0.3)
        if idx == 2:
            ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left", title="Distance / Demand")
    plt.tight_layout()
    plt.savefig(p5, dpi=300, bbox_inches="tight")
    plt.close()
    created_plots.append(p5)

    # 6. stopping_distance_vs_speed.png
    p6 = os.path.join(output_dir, "stopping_distance_vs_speed.png")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    fig.suptitle("Ego Vehicle Stopping Distance vs Approach Speed", fontsize=14, y=0.98)
    for idx, scen in enumerate(SCENARIO_TYPES):
        ax = axes[idx]
        ax.set_title(f"Scenario: {scen}")
        ax.set_xlabel("Ego Speed (m/s)")
        if idx == 0:
            ax.set_ylabel("Total Stopping Distance (m)")
        for d in INITIAL_DISTANCES:
            speeds_plot = []
            stop_dists = []
            for v in VIRYA_SPEEDS:
                entry = sum_map.get((scen, v, d))
                if entry and entry["stopping_distance_mean"] != "":
                    speeds_plot.append(v)
                    stop_dists.append(float(entry["stopping_distance_mean"]))
            if speeds_plot:
                ax.plot(speeds_plot, stop_dists, marker="o", label=f"d0={d:.1f}m")
        ax.set_xticks(VIRYA_SPEEDS)
        ax.set_xticklabels(["0.50", "1.00", "1.39"])
        ax.grid(True, alpha=0.3)
        if idx == 2:
            ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left", title="Initial Distance")
    plt.tight_layout()
    plt.savefig(p6, dpi=300, bbox_inches="tight")
    plt.close()
    created_plots.append(p6)

    # 7. state_transitions_vs_speed.png
    p7 = os.path.join(output_dir, "state_transitions_vs_speed.png")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    fig.suptitle("Total FSM State Transitions vs Approach Speed", fontsize=14, y=0.98)
    for idx, scen in enumerate(SCENARIO_TYPES):
        ax = axes[idx]
        ax.set_title(f"Scenario: {scen}")
        ax.set_xlabel("Ego Speed (m/s)")
        if idx == 0:
            ax.set_ylabel("FSM State Transitions Count")
        for d in INITIAL_DISTANCES:
            speeds_plot = []
            trans = []
            for v in VIRYA_SPEEDS:
                entry = sum_map.get((scen, v, d))
                if entry:
                    speeds_plot.append(v)
                    trans.append(entry["fsm_state_transitions_mean"])
            ax.plot(speeds_plot, trans, marker="s", label=f"d0={d:.1f}m")
        ax.set_xticks(VIRYA_SPEEDS)
        ax.set_xticklabels(["0.50", "1.00", "1.39"])
        ax.grid(True, alpha=0.3)
        if idx == 2:
            ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left", title="Initial Distance")
    plt.tight_layout()
    plt.savefig(p7, dpi=300, bbox_inches="tight")
    plt.close()
    created_plots.append(p7)

    # 8. triplet_reversal_count_vs_speed.png
    p8 = os.path.join(output_dir, "triplet_reversal_count_vs_speed.png")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
    fig.suptitle("Number of 3-Sample Sliding FSM Reversal Patterns vs Speed", fontsize=14, y=0.98)
    for idx, scen in enumerate(SCENARIO_TYPES):
        ax = axes[idx]
        ax.set_title(f"Scenario: {scen}")
        ax.set_xlabel("Ego Speed (m/s)")
        if idx == 0:
            ax.set_ylabel("Triplet Reversal Pattern Count")
        for d in INITIAL_DISTANCES:
            speeds_plot = []
            trips = []
            for v in VIRYA_SPEEDS:
                entry = sum_map.get((scen, v, d))
                if entry:
                    speeds_plot.append(v)
                    trips.append(entry["triplet_reversal_count_mean"])
            ax.plot(speeds_plot, trips, marker="x", label=f"d0={d:.1f}m")
        ax.set_xticks(VIRYA_SPEEDS)
        ax.set_xticklabels(["0.50", "1.00", "1.39"])
        ax.grid(True, alpha=0.3)
        if idx == 2:
            ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left", title="Initial Distance")
    plt.tight_layout()
    plt.savefig(p8, dpi=300, bbox_inches="tight")
    plt.close()
    created_plots.append(p8)

    print(f"Generated {len(created_plots)} figures in {output_dir}")
    return created_plots


def main():
    """Main execution function for Step 10 analysis."""
    raw_results, summary_results = run_operating_envelope_study()
    export_csvs(raw_results, summary_results)
    plot_all_figures(summary_results, raw_results)
    print("Step 10 VIRYA operating envelope study finished successfully.")


if __name__ == "__main__":
    main()
