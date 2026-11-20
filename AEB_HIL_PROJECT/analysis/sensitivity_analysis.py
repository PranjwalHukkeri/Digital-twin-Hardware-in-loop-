"""
AEB Controller Sensitivity Analysis Module.

Performs parameter sweeps and trade-off analysis for:
1. MIN_BRAKE_DURATION (engineering design parameter) across CCR, CCRm, and CCRb scenarios.
2. Operating conditions (Ego speed and initial separation distance).

Exports machine-readable CSV datasets to results/sensitivity/ and generates standalone plots under plots/sensitivity/.
"""

import os
import csv
import math
from typing import List, Dict, Any, Tuple
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.simulation import SimulationEngine, SimulationResult
from src.metrics import PerformanceEvaluator
from scenarios.scenarios import CCRScenario, CCRmScenario, CCRbScenario, Scenario


def count_fsm_state_transitions(result: SimulationResult) -> int:
    """
    Count the total number of FSM state transitions during a simulation run.
    """
    if not result.samples:
        return 0
    transitions = 0
    for i in range(1, len(result.samples)):
        if result.samples[i].aeb_state != result.samples[i - 1].aeb_state:
            transitions += 1
    return transitions


def compute_active_braking_metrics(result: SimulationResult) -> Tuple[int, float]:
    """
    Compute active braking episode count and total active braking duration.

    Active-Braking Episode Definition:
    A contiguous sequence of timesteps where desired deceleration < 0.0 m/s^2.
    Transitioning from 0.0 to < 0.0 starts an episode.
    Stepping between active AEB levels continues the active episode.
    Transitioning back to 0.0 finishes the episode.

    Returns:
        Tuple[int, float]: (number_of_episodes, total_active_duration_seconds)
    """
    if not result.samples:
        return 0, 0.0

    episodes = 0
    total_duration = 0.0
    in_active_episode = False
    samples = result.samples
    n_samples = len(samples)

    for i, s in enumerate(samples):
        if i + 1 < n_samples:
            step_dt = samples[i + 1].time - s.time
        elif n_samples >= 2:
            step_dt = samples[-1].time - samples[-2].time
        else:
            step_dt = 0.05

        is_active = s.desired_deceleration < 0.0

        if is_active:
            total_duration += step_dt
            if not in_active_episode:
                episodes += 1
                in_active_episode = True
        else:
            in_active_episode = False

    return episodes, round(total_duration, 4)


def check_boundary_chattering(result: SimulationResult) -> Tuple[bool, str]:
    """
    Detect if the FSM exhibits boundary chattering during the simulation run.

    Chattering Definition:
    Rapid repeated FSM state oscillation caused by threshold crossing where a state
    returns to a previously active state within 2 timesteps
    (e.g., L1 -> WARNING -> L1 or L2 -> L1 -> L2).

    Note: Multiple active-braking episodes occurring due to separate threat appearances
    are recorded as a separate metric and do NOT automatically imply chattering.

    Returns:
        Tuple[bool, str]: (chattering_detected, detailed_description)
    """
    if not result.samples or len(result.samples) < 3:
        return False, "No Boundary Chattering (Insufficient Samples)"

    states = [s.aeb_state for s in result.samples]
    oscillations = []

    for i in range(len(states) - 2):
        s0 = states[i]
        s1 = states[i + 1]
        s2 = states[i + 2]
        # Oscillatory pattern A -> B -> A where A != B
        if s0 != s1 and s0 == s2:
            # Exclude single isolated 1-timestep pulse from SAFE -> ACTIVE -> SAFE
            if s0 == "SAFE" and s2 == "SAFE" and s1 != "SAFE":
                continue
            oscillations.append(
                f"{s0} -> {s1} -> {s0} at t={result.samples[i].time:0.2f}s"
            )

    if oscillations:
        desc = f"Boundary Chattering Detected ({len(oscillations)} oscillations: {oscillations[0]})"
        return True, desc

    return False, "No Boundary Chattering (Stable Transitions)"


def count_triplet_reversal_patterns(result: SimulationResult) -> int:
    """
    Count the number of 3-sample sliding FSM reversal patterns in a simulation run.

    Definition:
    "Number of 3-sample sliding FSM reversal patterns."
    Counts every 3-sample sliding window [s[i], s[i+1], s[i+2]] satisfying:
        s[i] != s[i+1] and s[i] == s[i+2]
    excluding isolated pulses of the form:
        SAFE -> X -> SAFE (where X != SAFE)

    This metric matches the exact same reversal windows that the existing
    check_boundary_chattering() detector places in its internal oscillations list.

    Note:
    This is a sliding-window pattern match count ("Number of 3-sample sliding FSM reversal patterns").
    It does NOT represent physical chattering cycles, instability cycles, or unsafe events.

    Args:
        result (SimulationResult): Trajectory dataset containing recorded samples.

    Returns:
        int: Total count of matching 3-sample reversal windows.
    """
    if not result.samples or len(result.samples) < 3:
        return 0

    states = [s.aeb_state for s in result.samples]
    count = 0

    for i in range(len(states) - 2):
        s0 = states[i]
        s1 = states[i + 1]
        s2 = states[i + 2]
        if s0 != s1 and s0 == s2:
            if s0 == "SAFE" and s2 == "SAFE" and s1 != "SAFE":
                continue
            count += 1

    return count


def run_min_brake_duration_sweep(
    duration_values: List[float] = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50],
    results_dir: str = "results/sensitivity",
    plots_dir: str = "plots/sensitivity",
) -> List[Dict[str, Any]]:
    """
    Run Phase 1 sweep over MIN_BRAKE_DURATION values across CCR, CCRm, and CCRb scenarios.
    """
    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

    scenarios = [
        CCRScenario(name="CCR_Stationary_Target", initial_ego_speed=15.0, initial_distance=30.0),
        CCRmScenario(name="CCRm_Moving_Target", initial_ego_speed=15.0, initial_target_speed=5.0, initial_distance=25.0),
        CCRbScenario(
            name="CCRb_Braking_Target",
            initial_ego_speed=15.0,
            initial_target_speed=10.0,
            initial_distance=30.0,
            target_decel=-2.0,
            brake_start_time=1.0,
        ),
    ]

    sweep_records = []

    for scenario in scenarios:
        for min_dur in duration_values:
            engine = SimulationEngine()
            result = engine.run(scenario=scenario, min_brake_duration=min_dur)
            metrics = PerformanceEvaluator.evaluate(result)

            transitions = count_fsm_state_transitions(result)
            episodes, total_braking_duration = compute_active_braking_metrics(result)
            has_chattering, chattering_desc = check_boundary_chattering(result)

            record = {
                "scenario": scenario.name,
                "min_brake_duration": min_dur,
                "activation_time": metrics.aeb_activation_time,
                "ttc_at_activation": metrics.ttc_at_activation,
                "max_deceleration": metrics.max_braking_deceleration,
                "minimum_distance": round(metrics.min_distance, 4),
                "final_ego_speed": round(metrics.final_ego_speed, 4),
                "final_relative_velocity": round(metrics.final_relative_velocity, 4),
                "min_relative_velocity": round(metrics.min_relative_velocity, 4),
                "vehicle_stop_status": metrics.vehicle_stop_status,
                "collision_status": metrics.collision_status,
                "fsm_state_transitions": transitions,
                "active_braking_episodes": episodes,
                "total_active_braking_duration": total_braking_duration,
                "chattering_detected": has_chattering,
                "chattering_description": chattering_desc,
            }
            sweep_records.append(record)

    # Export to CSV
    csv_path = os.path.join(results_dir, "min_brake_duration_sweep.csv")
    fieldnames = list(sweep_records[0].keys())
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(sweep_records)

    # Generate Individual Plots
    _plot_duration_vs_transitions(sweep_records, os.path.join(plots_dir, "min_brake_duration_vs_state_transitions.png"))
    _plot_duration_vs_min_distance(sweep_records, os.path.join(plots_dir, "min_brake_duration_vs_minimum_distance.png"))
    _plot_duration_vs_braking_duration(sweep_records, os.path.join(plots_dir, "min_brake_duration_vs_total_active_braking_duration.png"))

    return sweep_records


def run_operating_condition_sweep(
    results_dir: str = "results/sensitivity",
    plots_dir: str = "plots/sensitivity",
) -> List[Dict[str, Any]]:
    """
    Run Phase 2 sweep over operating conditions (Ego speed and initial separation distance).
    """
    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

    ego_speeds = [1.0, 2.0, 5.0, 10.0, 15.0]
    initial_distances = [5.0, 10.0, 15.0, 20.0, 30.0]

    sweep_records = []

    # Speed sweep at constant initial distance 30.0 m against stationary target
    for speed in ego_speeds:
        scenario = Scenario("Speed_Sweep", initial_ego_speed=speed, initial_target_speed=0.0, initial_distance=30.0)
        engine = SimulationEngine()
        result = engine.run(scenario=scenario)
        metrics = PerformanceEvaluator.evaluate(result)
        transitions = count_fsm_state_transitions(result)
        episodes, total_braking_duration = compute_active_braking_metrics(result)

        record = {
            "sweep_type": "ego_speed_sweep",
            "ego_speed": speed,
            "initial_distance": 30.0,
            "min_distance": round(metrics.min_distance, 4),
            "final_ego_speed": round(metrics.final_ego_speed, 4),
            "collision_status": metrics.collision_status,
            "fsm_state_transitions": transitions,
            "total_active_braking_duration": total_braking_duration,
        }
        sweep_records.append(record)

    # Distance sweep at constant initial Ego speed 10.0 m/s against stationary target
    for dist in initial_distances:
        scenario = Scenario("Distance_Sweep", initial_ego_speed=10.0, initial_target_speed=0.0, initial_distance=dist)
        engine = SimulationEngine()
        result = engine.run(scenario=scenario)
        metrics = PerformanceEvaluator.evaluate(result)
        transitions = count_fsm_state_transitions(result)
        episodes, total_braking_duration = compute_active_braking_metrics(result)

        record = {
            "sweep_type": "initial_distance_sweep",
            "ego_speed": 10.0,
            "initial_distance": dist,
            "min_distance": round(metrics.min_distance, 4),
            "final_ego_speed": round(metrics.final_ego_speed, 4),
            "collision_status": metrics.collision_status,
            "fsm_state_transitions": transitions,
            "total_active_braking_duration": total_braking_duration,
        }
        sweep_records.append(record)

    # Export to CSV
    csv_path = os.path.join(results_dir, "operating_condition_sweep.csv")
    fieldnames = list(sweep_records[0].keys())
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(sweep_records)

    return sweep_records


# Plot Helper Functions
def _plot_duration_vs_transitions(records: List[Dict[str, Any]], filepath: str) -> None:
    plt.figure(figsize=(8, 5))
    scenarios = set(r["scenario"] for r in records)
    for sc in scenarios:
        sc_recs = [r for r in records if r["scenario"] == sc]
        x = [r["min_brake_duration"] for r in sc_recs]
        y = [r["fsm_state_transitions"] for r in sc_recs]
        plt.plot(x, y, marker="o", linewidth=2, label=sc)

    plt.xlabel("MIN_BRAKE_DURATION (s)", fontsize=11)
    plt.ylabel("Number of FSM State Transitions", fontsize=11)
    plt.title("MIN_BRAKE_DURATION vs FSM State Transitions", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_duration_vs_min_distance(records: List[Dict[str, Any]], filepath: str) -> None:
    plt.figure(figsize=(8, 5))
    scenarios = set(r["scenario"] for r in records)
    for sc in scenarios:
        sc_recs = [r for r in records if r["scenario"] == sc]
        x = [r["min_brake_duration"] for r in sc_recs]
        y = [r["minimum_distance"] for r in sc_recs]
        plt.plot(x, y, marker="s", linewidth=2, label=sc)

    plt.xlabel("MIN_BRAKE_DURATION (s)", fontsize=11)
    plt.ylabel("Minimum Separation Distance (m)", fontsize=11)
    plt.title("MIN_BRAKE_DURATION vs Minimum Distance", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_duration_vs_braking_duration(records: List[Dict[str, Any]], filepath: str) -> None:
    plt.figure(figsize=(8, 5))
    scenarios = set(r["scenario"] for r in records)
    for sc in scenarios:
        sc_recs = [r for r in records if r["scenario"] == sc]
        x = [r["min_brake_duration"] for r in sc_recs]
        y = [r["total_active_braking_duration"] for r in sc_recs]
        plt.plot(x, y, marker="^", linewidth=2, label=sc)

    plt.xlabel("MIN_BRAKE_DURATION (s)", fontsize=11)
    plt.ylabel("Total Active Braking Duration (s)", fontsize=11)
    plt.title("MIN_BRAKE_DURATION vs Total Active Braking Duration", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()
