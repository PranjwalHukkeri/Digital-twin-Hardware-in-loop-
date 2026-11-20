"""
AEB Multi-Seed Sensor Noise Robustness Analysis.

Executes multi-seed sensitivity analysis across CCR, CCRm, and CCRb scenarios
subject to varying levels of zero-mean Gaussian distance measurement noise.

Key experimental parameters:
- Scenarios: CCR_Stationary_Target, CCRm_Moving_Target, CCRb_Braking_Target
- Noise standard deviations (m): [0.00, 0.01, 0.02, 0.05, 0.10, 0.20]
- Seeds: [42, 43, 44, 45, 46, 47, 48, 49, 50, 51] (10 seeds)
- Total runs: 3 * 6 * 10 = 180 runs
- Isolation: sensor_latency = 0.0 s, actuator_tau = 0.0 s, dt = 0.05 s
- Measurement noise applied ONLY to distance sensor.

Note:
"Number of 3-sample sliding FSM reversal patterns" (triplet_reversal_count)
is a structural pattern-match metric and does not represent physical chattering cycles
or unsafe behavior.
"""

import os
import csv
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
DEFAULT_NOISE_VALUES = [0.00, 0.01, 0.02, 0.05, 0.10, 0.20]


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


def aggregate_multiseed_results(
    raw_records: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """
    Aggregate raw simulation records across seeds by (scenario, noise_std).

    Calculates sample mean, standard deviation, minimum, maximum, 5th percentile,
    and 95th percentile for all continuous and integer metrics.
    Calculates collision_count, collision_rate, stopped_count, stopped_rate for status flags.

    Args:
        raw_records: List of run-level records.

    Returns:
        List of aggregated summary dictionaries sorted by scenario and noise_std.
    """
    groups: Dict[Tuple[str, float], List[Dict[str, Any]]] = {}
    for r in raw_records:
        key = (r["scenario"], float(r["noise_std"]))
        groups.setdefault(key, []).append(r)

    summary_records: List[Dict[str, Any]] = []

    numeric_metrics = [
        "minimum_distance",
        "activation_time",
        "ttc_at_activation",
        "state_transition_count",
        "triplet_reversal_count",
        "braking_episode_count",
        "total_braking_duration",
        "final_ego_speed",
        "stopping_distance",
        "max_braking_deceleration",
        "max_actual_deceleration",
        "braking_response_time",
    ]

    for (scenario, noise_std), records in sorted(groups.items()):
        n_samples = len(records)
        collision_count = sum(1 for r in records if r["collision_status"] is True)
        collision_rate = collision_count / n_samples

        stopped_count = sum(1 for r in records if r["vehicle_stop_status"] == "STOPPED")
        stopped_rate = stopped_count / n_samples

        summary_row: Dict[str, Any] = {
            "scenario": scenario,
            "noise_std": noise_std,
            "noise_std_cm": round(noise_std * 100.0, 2),
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
                mean_val = float(np.mean(arr))
                std_val = float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0
                min_val = float(np.min(arr))
                max_val = float(np.max(arr))
                p05_val = float(np.percentile(arr, 5))
                p95_val = float(np.percentile(arr, 95))

                summary_row[f"{metric}_mean"] = round(mean_val, 4)
                summary_row[f"{metric}_std"] = round(std_val, 4)
                summary_row[f"{metric}_min"] = round(min_val, 4)
                summary_row[f"{metric}_max"] = round(max_val, 4)
                summary_row[f"{metric}_5th_percentile"] = round(p05_val, 4)
                summary_row[f"{metric}_95th_percentile"] = round(p95_val, 4)
            else:
                summary_row[f"{metric}_mean"] = None
                summary_row[f"{metric}_std"] = None
                summary_row[f"{metric}_min"] = None
                summary_row[f"{metric}_max"] = None
                summary_row[f"{metric}_5th_percentile"] = None
                summary_row[f"{metric}_95th_percentile"] = None

        summary_records.append(summary_row)

    return summary_records


def run_sensor_noise_multiseed(
    scenarios: Optional[List[Any]] = None,
    noise_values: Optional[List[float]] = None,
    seeds: Optional[List[int]] = None,
    results_dir: str = "results/sensor_noise_multiseed",
    plots_dir: str = "plots/sensor_noise_multiseed",
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    Execute distance sensor noise multi-seed sensitivity analysis.

    Args:
        scenarios: List of scenario instances. Defaults to CCR, CCRm, CCRb.
        noise_values: List of Gaussian noise standard deviation values (m).
        seeds: List of random seeds for multi-seed analysis.
        results_dir: Directory to save raw and summary CSVs.
        plots_dir: Directory to save generated figures.

    Returns:
        Tuple of (raw_records, summary_records).
    """
    if scenarios is None:
        scenarios = get_default_scenarios()
    if noise_values is None:
        noise_values = DEFAULT_NOISE_VALUES
    if seeds is None:
        seeds = DEFAULT_SEEDS

    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

    raw_records: List[Dict[str, Any]] = []

    for scenario in scenarios:
        for noise_std in noise_values:
            for seed in seeds:
                # Controlled isolation:
                # Latency = 0.0 s, Actuator Tau = 0.0 s.
                # Noise applied ONLY to distance sensor.
                engine = SimulationEngine(
                    sensor_latency=0.0,
                    actuator_tau=0.0,
                    distance_noise_std=noise_std,
                    distance_seed=seed,
                )

                result: SimulationResult = engine.run(scenario=scenario)
                metrics = PerformanceEvaluator.evaluate(result)

                state_transitions = count_fsm_state_transitions(result)
                braking_episodes, active_duration = compute_active_braking_metrics(result)
                chattering_detected, _ = check_boundary_chattering(result)
                triplet_reversals = count_triplet_reversal_patterns(result)

                record: Dict[str, Any] = {
                    "scenario": scenario.name,
                    "noise_std": float(noise_std),
                    "noise_std_cm": round(noise_std * 100.0, 2),
                    "seed": int(seed),
                    "activation_time": (
                        round(metrics.aeb_activation_time, 4)
                        if metrics.aeb_activation_time is not None
                        else None
                    ),
                    "ttc_at_activation": (
                        round(metrics.ttc_at_activation, 4)
                        if metrics.ttc_at_activation is not None
                        else None
                    ),
                    "minimum_distance": round(metrics.min_distance, 4),
                    "max_braking_deceleration": round(
                        metrics.max_braking_deceleration, 4
                    ),
                    "max_actual_deceleration": round(
                        metrics.max_actual_deceleration, 4
                    ),
                    "braking_response_time": (
                        round(metrics.ninety_percent_braking_response_time, 4)
                        if metrics.ninety_percent_braking_response_time is not None
                        else None
                    ),
                    "final_ego_speed": round(metrics.final_ego_speed, 4),
                    "collision_status": metrics.collision_status,
                    "vehicle_stop_status": metrics.vehicle_stop_status,
                    "state_transition_count": state_transitions,
                    "boundary_chattering_detected": chattering_detected,
                    "triplet_reversal_count": triplet_reversals,
                    "braking_episode_count": braking_episodes,
                    "total_braking_duration": round(active_duration, 4),
                    "stopping_distance": (
                        round(metrics.stopping_distance, 4)
                        if metrics.stopping_distance is not None
                        else None
                    ),
                }

                raw_records.append(record)

    # ---------------------------------------------------------
    # Save Raw Results CSV
    # ---------------------------------------------------------
    raw_csv_path = os.path.join(results_dir, "sensor_noise_multiseed_raw.csv")
    raw_fieldnames = list(raw_records[0].keys())
    with open(raw_csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=raw_fieldnames)
        writer.writeheader()
        writer.writerows(raw_records)

    # ---------------------------------------------------------
    # Aggregate and Save Summary Results CSV
    # ---------------------------------------------------------
    summary_records = aggregate_multiseed_results(raw_records)
    summary_csv_path = os.path.join(results_dir, "sensor_noise_multiseed_summary.csv")
    summary_fieldnames = list(summary_records[0].keys())
    with open(summary_csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
        writer.writeheader()
        writer.writerows(summary_records)

    # ---------------------------------------------------------
    # Generate Multi-Seed Figures
    # ---------------------------------------------------------
    plot_multiseed_min_distance(
        summary_records,
        os.path.join(plots_dir, "sensor_noise_vs_minimum_distance.png"),
    )
    plot_multiseed_collision_rate(
        summary_records,
        os.path.join(plots_dir, "sensor_noise_vs_collision_rate.png"),
    )
    plot_multiseed_state_transitions(
        summary_records,
        os.path.join(plots_dir, "sensor_noise_vs_state_transitions.png"),
    )
    plot_multiseed_triplet_reversals(
        summary_records,
        os.path.join(plots_dir, "sensor_noise_vs_triplet_reversal_count.png"),
    )
    plot_multiseed_activation_time(
        summary_records,
        os.path.join(plots_dir, "sensor_noise_vs_activation_time.png"),
    )

    return raw_records, summary_records


# ---------------------------------------------------------------------------
# Plotting Helpers
# ---------------------------------------------------------------------------

def _setup_plot_styles():
    markers = {
        "CCR_Stationary_Target": "o",
        "CCRm_Moving_Target": "s",
        "CCRb_Braking_Target": "^",
    }
    colors = {
        "CCR_Stationary_Target": "#1f77b4",
        "CCRm_Moving_Target": "#2ca02c",
        "CCRb_Braking_Target": "#ff7f0e",
    }
    return markers, colors


def plot_multiseed_min_distance(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot minimum separation distance vs sensor noise with 5th-95th percentile band."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))

    for sc in scenarios:
        sc_data = [r for r in summary_records if r["scenario"] == sc]
        x = [r["noise_std"] for r in sc_data]
        mean_y = [r["minimum_distance_mean"] for r in sc_data]
        p05_y = [r["minimum_distance_5th_percentile"] for r in sc_data]
        p95_y = [r["minimum_distance_95th_percentile"] for r in sc_data]

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")

        plt.plot(x, mean_y, marker=mkr, color=col, linewidth=2, label=f"{sc} (Mean)")
        plt.fill_between(x, p05_y, p95_y, color=col, alpha=0.18, label=f"{sc} (5th-95th %ile)")

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("Minimum Separation Distance (m)", fontsize=11)
    plt.title("Multi-Seed Sensor Noise: Minimum Separation Distance", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_multiseed_collision_rate(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot collision rate vs sensor noise for each scenario."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))

    for sc in scenarios:
        sc_data = [r for r in summary_records if r["scenario"] == sc]
        x = [r["noise_std"] for r in sc_data]
        rate = [r["collision_rate"] for r in sc_data]

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")

        plt.plot(x, rate, marker=mkr, color=col, linewidth=2, label=sc)

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("Collision Rate", fontsize=11)
    plt.title("Multi-Seed Sensor Noise: Collision Rate", fontsize=12, fontweight="bold")
    plt.ylim(-0.05, 1.05)
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_multiseed_state_transitions(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot FSM state transitions vs sensor noise with 5th-95th percentile band."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))

    for sc in scenarios:
        sc_data = [r for r in summary_records if r["scenario"] == sc]
        x = [r["noise_std"] for r in sc_data]
        mean_y = [r["state_transition_count_mean"] for r in sc_data]
        p05_y = [r["state_transition_count_5th_percentile"] for r in sc_data]
        p95_y = [r["state_transition_count_95th_percentile"] for r in sc_data]

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")

        plt.plot(x, mean_y, marker=mkr, color=col, linewidth=2, label=f"{sc} (Mean)")
        plt.fill_between(x, p05_y, p95_y, color=col, alpha=0.18, label=f"{sc} (5th-95th %ile)")

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("FSM State Transition Count", fontsize=11)
    plt.title("Multi-Seed Sensor Noise: FSM State Transition Count", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_multiseed_triplet_reversals(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot 3-sample sliding FSM reversal patterns vs sensor noise with 5th-95th percentile band."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))

    for sc in scenarios:
        sc_data = [r for r in summary_records if r["scenario"] == sc]
        x = [r["noise_std"] for r in sc_data]
        mean_y = [r["triplet_reversal_count_mean"] for r in sc_data]
        p05_y = [r["triplet_reversal_count_5th_percentile"] for r in sc_data]
        p95_y = [r["triplet_reversal_count_95th_percentile"] for r in sc_data]

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")

        plt.plot(x, mean_y, marker=mkr, color=col, linewidth=2, label=f"{sc} (Mean)")
        plt.fill_between(x, p05_y, p95_y, color=col, alpha=0.18, label=f"{sc} (5th-95th %ile)")

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("3-Sample Sliding FSM Reversal Patterns", fontsize=11)
    plt.title("Multi-Seed Sensor Noise: 3-Sample Sliding FSM Reversal Patterns", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_multiseed_activation_time(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot AEB activation time vs sensor noise with 5th-95th percentile band."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))

    for sc in scenarios:
        sc_data = [r for r in summary_records if r["scenario"] == sc]
        x = [r["noise_std"] for r in sc_data]
        mean_y = [r["activation_time_mean"] for r in sc_data]
        p05_y = [r["activation_time_5th_percentile"] for r in sc_data]
        p95_y = [r["activation_time_95th_percentile"] for r in sc_data]

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")

        plt.plot(x, mean_y, marker=mkr, color=col, linewidth=2, label=f"{sc} (Mean)")
        plt.fill_between(x, p05_y, p95_y, color=col, alpha=0.18, label=f"{sc} (5th-95th %ile)")

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("AEB Activation Time (s)", fontsize=11)
    plt.title("Multi-Seed Sensor Noise: AEB Activation Time", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


if __name__ == "__main__":
    print("Starting Multi-Seed Sensor Noise Robustness Sweep (180 runs)...")
    raw_res, summary_res = run_sensor_noise_multiseed()
    print(f"Sweep complete. Total raw runs: {len(raw_res)}, Summary records: {len(summary_res)}")
