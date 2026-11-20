"""
AEB Combined Sensor and Actuator Impairment Robustness Study.

Evaluates the joint sensitivity and robustness of AEB performance across
CCR, CCRm, and CCRb scenarios subject to combined:
- Sensor distance measurement noise (Gaussian std: 0.00, 0.05, 0.10, 0.20 m)
- Sensor measurement latency (0.00, 0.05, 0.10 s)
- Brake actuator first-order lag time constant tau (0.00, 0.10, 0.20, 0.30 s)
- Multi-seed pseudo-random sequences (10 seeds: 42 to 51)

Total experimental matrix:
3 scenarios * 4 noise levels * 3 latencies * 4 actuator taus * 10 seeds = 1440 runs.

Note:
This is a software robustness study in a discrete-time simulation environment.
It does not claim Hardware-in-the-Loop (HIL), real vehicle, or regulatory certification.
"Number of 3-sample sliding FSM reversal patterns" (triplet_reversal_count)
measures structural sliding-window reversal patterns, not physical chattering cycles.
"""

import os
import csv
import math
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
DEFAULT_NOISE_VALUES = [0.00, 0.05, 0.10, 0.20]
DEFAULT_LATENCY_VALUES = [0.00, 0.05, 0.10]
DEFAULT_ACTUATOR_TAU_VALUES = [0.00, 0.10, 0.20, 0.30]


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


def aggregate_combined_results(
    raw_records: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """
    Aggregate raw records over seeds by (scenario, noise_std, sensor_latency, actuator_tau).

    Calculates mean, std, min, max, 5th percentile, and 95th percentile for all continuous
    and integer metrics. Computes collision_rate and stopped_rate for status flags.

    Args:
        raw_records: List of run-level records.

    Returns:
        List of aggregated summary dictionaries sorted by condition.
    """
    groups: Dict[Tuple[str, float, float, float], List[Dict[str, Any]]] = {}
    for r in raw_records:
        key = (
            r["scenario"],
            float(r["noise_std"]),
            float(r["sensor_latency"]),
            float(r["actuator_tau"]),
        )
        groups.setdefault(key, []).append(r)

    summary_records: List[Dict[str, Any]] = []

    numeric_metrics = [
        "minimum_distance",
        "aeb_activation_time",
        "ttc_at_activation",
        "state_transition_count",
        "triplet_reversal_count",
        "max_braking_deceleration",
        "max_actual_deceleration",
        "ninety_percent_braking_response_time",
        "stopping_time",
        "stopping_distance",
        "final_ego_speed",
        "braking_episode_count",
        "total_braking_duration",
    ]

    for (scenario, noise_std, latency, tau), records in sorted(groups.items()):
        n_samples = len(records)
        collision_count = sum(1 for r in records if r["collision_status"] is True)
        collision_rate = collision_count / n_samples

        stopped_count = sum(1 for r in records if r["vehicle_stop_status"] == "STOPPED")
        stopped_rate = stopped_count / n_samples

        row: Dict[str, Any] = {
            "scenario": scenario,
            "noise_std": noise_std,
            "noise_std_cm": round(noise_std * 100.0, 2),
            "sensor_latency": latency,
            "actuator_tau": tau,
            "sample_size": n_samples,
            "collision_count": collision_count,
            "collision_rate": round(collision_rate, 4),
            "stopped_count": stopped_count,
            "stopped_rate": round(stopped_rate, 4),
        }

        for metric in numeric_metrics:
            vals = [
                float(r[metric])
                for r in records
                if r[metric] is not None and not math.isnan(float(r[metric]))
            ]
            if vals:
                arr = np.array(vals, dtype=float)
                mean_val = float(np.mean(arr))
                std_val = float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0
                min_val = float(np.min(arr))
                max_val = float(np.max(arr))
                p05_val = float(np.percentile(arr, 5))
                p95_val = float(np.percentile(arr, 95))

                row[f"{metric}_mean"] = round(mean_val, 4)
                row[f"{metric}_std"] = round(std_val, 4)
                row[f"{metric}_min"] = round(min_val, 4)
                row[f"{metric}_max"] = round(max_val, 4)
                row[f"{metric}_5th_percentile"] = round(p05_val, 4)
                row[f"{metric}_95th_percentile"] = round(p95_val, 4)
            else:
                row[f"{metric}_mean"] = None
                row[f"{metric}_std"] = None
                row[f"{metric}_min"] = None
                row[f"{metric}_max"] = None
                row[f"{metric}_5th_percentile"] = None
                row[f"{metric}_95th_percentile"] = None

        summary_records.append(row)

    return summary_records


def run_combined_impairment_sweep(
    scenarios: Optional[List[Any]] = None,
    noise_values: Optional[List[float]] = None,
    latency_values: Optional[List[float]] = None,
    actuator_tau_values: Optional[List[float]] = None,
    seeds: Optional[List[int]] = None,
    results_dir: str = "results/combined_impairment",
    plots_dir: str = "plots/combined_impairment",
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    Execute combined sensor and actuator impairment sweep.

    Args:
        scenarios: List of scenario instances.
        noise_values: Distance sensor noise standard deviations (m).
        latency_values: Sensor measurement latencies (s).
        actuator_tau_values: Brake actuator lag time constants (s).
        seeds: Random seeds.
        results_dir: Directory for CSV outputs.
        plots_dir: Directory for generated plots.

    Returns:
        Tuple of (raw_records, summary_records).
    """
    if scenarios is None:
        scenarios = get_default_scenarios()
    if noise_values is None:
        noise_values = DEFAULT_NOISE_VALUES
    if latency_values is None:
        latency_values = DEFAULT_LATENCY_VALUES
    if actuator_tau_values is None:
        actuator_tau_values = DEFAULT_ACTUATOR_TAU_VALUES
    if seeds is None:
        seeds = DEFAULT_SEEDS

    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

    raw_records: List[Dict[str, Any]] = []

    for scenario in scenarios:
        for noise_std in noise_values:
            for latency in latency_values:
                for tau in actuator_tau_values:
                    for seed in seeds:
                        engine = SimulationEngine(
                            sensor_latency=latency,
                            actuator_tau=tau,
                            distance_noise_std=noise_std,
                            distance_seed=seed,
                        )

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

                        record: Dict[str, Any] = {
                            "scenario": scenario.name,
                            "noise_std": float(noise_std),
                            "noise_std_cm": round(noise_std * 100.0, 2),
                            "sensor_latency": float(latency),
                            "actuator_tau": float(tau),
                            "seed": int(seed),
                            "minimum_distance": round(metrics.min_distance, 4),
                            "collision_status": metrics.collision_status,
                            "aeb_activation_time": act_time,
                            "activation_time": act_time,
                            "ttc_at_activation": (
                                round(metrics.ttc_at_activation, 4)
                                if metrics.ttc_at_activation is not None
                                else None
                            ),
                            "state_transition_count": state_transitions,
                            "triplet_reversal_count": triplet_reversals,
                            "max_braking_deceleration": round(
                                metrics.max_braking_deceleration, 4
                            ),
                            "max_actual_deceleration": round(
                                metrics.max_actual_deceleration, 4
                            ),
                            "ninety_percent_braking_response_time": resp_time,
                            "braking_response_time": resp_time,
                            "stopping_time": (
                                round(metrics.stopping_time, 4)
                                if metrics.stopping_time is not None
                                else None
                            ),
                            "stopping_distance": (
                                round(metrics.stopping_distance, 4)
                                if metrics.stopping_distance is not None
                                else None
                            ),
                            "vehicle_stop_status": metrics.vehicle_stop_status,
                            "final_ego_speed": round(metrics.final_ego_speed, 4),
                            "braking_episode_count": braking_episodes,
                            "total_braking_duration": round(active_duration, 4),
                            "boundary_chattering_detected": chattering_detected,
                        }

                        raw_records.append(record)

    # ---------------------------------------------------------
    # Save Raw Results CSV
    # ---------------------------------------------------------
    raw_csv_path = os.path.join(results_dir, "combined_impairment_raw.csv")
    raw_fieldnames = list(raw_records[0].keys())
    with open(raw_csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=raw_fieldnames)
        writer.writeheader()
        writer.writerows(raw_records)

    # ---------------------------------------------------------
    # Aggregate and Save Summary CSV
    # ---------------------------------------------------------
    summary_records = aggregate_combined_results(raw_records)
    summary_csv_path = os.path.join(results_dir, "combined_impairment_summary.csv")
    summary_fieldnames = list(summary_records[0].keys())
    with open(summary_csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
        writer.writeheader()
        writer.writerows(summary_records)

    # ---------------------------------------------------------
    # Generate Plots
    # ---------------------------------------------------------
    generate_all_plots(summary_records, plots_dir)

    return raw_records, summary_records


# ---------------------------------------------------------------------------
# Plotting Implementations
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


def plot_minimum_distance_vs_sensor_noise(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot minimum distance vs sensor noise across scenarios with 5th-95th percentile envelope."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))
    noises = sorted(set(r["noise_std"] for r in summary_records))

    for sc in scenarios:
        mean_y, p05_y, p95_y = [], [], []
        for n in noises:
            pts = [r for r in summary_records if r["scenario"] == sc and r["noise_std"] == n]
            vals = [p["minimum_distance_mean"] for p in pts if p["minimum_distance_mean"] is not None]
            p05s = [p["minimum_distance_5th_percentile"] for p in pts if p["minimum_distance_5th_percentile"] is not None]
            p95s = [p["minimum_distance_95th_percentile"] for p in pts if p["minimum_distance_95th_percentile"] is not None]
            mean_y.append(np.mean(vals) if vals else float("nan"))
            p05_y.append(np.min(p05s) if p05s else float("nan"))
            p95_y.append(np.max(p95s) if p95s else float("nan"))

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")
        plt.plot(noises, mean_y, marker=mkr, color=col, linewidth=2, label=f"{sc} (Mean)")
        plt.fill_between(noises, p05_y, p95_y, color=col, alpha=0.18, label=f"{sc} (5th-95th %ile envelope)")

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("Minimum Separation Distance (m)", fontsize=11)
    plt.title("Combined Impairment: Minimum Separation Distance vs Sensor Noise", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_collision_rate_vs_sensor_noise(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot collision rate vs sensor noise for each scenario."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))
    noises = sorted(set(r["noise_std"] for r in summary_records))

    for sc in scenarios:
        rates = []
        for n in noises:
            pts = [r for r in summary_records if r["scenario"] == sc and r["noise_std"] == n]
            total_samples = sum(p["sample_size"] for p in pts)
            total_collisions = sum(p["collision_count"] for p in pts)
            rate = total_collisions / total_samples if total_samples > 0 else 0.0
            rates.append(rate)

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")
        plt.plot(noises, rates, marker=mkr, color=col, linewidth=2, label=sc)

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("Collision Rate", fontsize=11)
    plt.title("Combined Impairment: Collision Rate vs Sensor Noise", fontsize=12, fontweight="bold")
    plt.ylim(-0.05, 1.05)
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_state_transitions_vs_sensor_noise(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot FSM state transition count vs sensor noise with envelope."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))
    noises = sorted(set(r["noise_std"] for r in summary_records))

    for sc in scenarios:
        mean_y, p05_y, p95_y = [], [], []
        for n in noises:
            pts = [r for r in summary_records if r["scenario"] == sc and r["noise_std"] == n]
            vals = [p["state_transition_count_mean"] for p in pts if p["state_transition_count_mean"] is not None]
            p05s = [p["state_transition_count_5th_percentile"] for p in pts if p["state_transition_count_5th_percentile"] is not None]
            p95s = [p["state_transition_count_95th_percentile"] for p in pts if p["state_transition_count_95th_percentile"] is not None]
            mean_y.append(np.mean(vals) if vals else float("nan"))
            p05_y.append(np.min(p05s) if p05s else float("nan"))
            p95_y.append(np.max(p95s) if p95s else float("nan"))

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")
        plt.plot(noises, mean_y, marker=mkr, color=col, linewidth=2, label=f"{sc} (Mean)")
        plt.fill_between(noises, p05_y, p95_y, color=col, alpha=0.18, label=f"{sc} (Envelope)")

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("FSM State Transition Count", fontsize=11)
    plt.title("Combined Impairment: FSM State Transitions vs Sensor Noise", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_triplet_reversals_vs_sensor_noise(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot 3-sample sliding FSM reversal patterns vs sensor noise with envelope."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))
    noises = sorted(set(r["noise_std"] for r in summary_records))

    for sc in scenarios:
        mean_y, p05_y, p95_y = [], [], []
        for n in noises:
            pts = [r for r in summary_records if r["scenario"] == sc and r["noise_std"] == n]
            vals = [p["triplet_reversal_count_mean"] for p in pts if p["triplet_reversal_count_mean"] is not None]
            p05s = [p["triplet_reversal_count_5th_percentile"] for p in pts if p["triplet_reversal_count_5th_percentile"] is not None]
            p95s = [p["triplet_reversal_count_95th_percentile"] for p in pts if p["triplet_reversal_count_95th_percentile"] is not None]
            mean_y.append(np.mean(vals) if vals else float("nan"))
            p05_y.append(np.min(p05s) if p05s else float("nan"))
            p95_y.append(np.max(p95s) if p95s else float("nan"))

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")
        plt.plot(noises, mean_y, marker=mkr, color=col, linewidth=2, label=f"{sc} (Mean)")
        plt.fill_between(noises, p05_y, p95_y, color=col, alpha=0.18, label=f"{sc} (Envelope)")

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("3-Sample Sliding FSM Reversal Patterns", fontsize=11)
    plt.title("Combined Impairment: 3-Sample Sliding FSM Reversal Patterns vs Sensor Noise", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_activation_time_vs_sensor_latency(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot AEB activation time vs sensor latency across scenarios."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))
    latencies = sorted(set(r["sensor_latency"] for r in summary_records))

    for sc in scenarios:
        mean_y, p05_y, p95_y = [], [], []
        for lat in latencies:
            pts = [r for r in summary_records if r["scenario"] == sc and r["sensor_latency"] == lat]
            vals = [p["aeb_activation_time_mean"] for p in pts if p["aeb_activation_time_mean"] is not None]
            p05s = [p["aeb_activation_time_5th_percentile"] for p in pts if p["aeb_activation_time_5th_percentile"] is not None]
            p95s = [p["aeb_activation_time_95th_percentile"] for p in pts if p["aeb_activation_time_95th_percentile"] is not None]
            mean_y.append(np.mean(vals) if vals else float("nan"))
            p05_y.append(np.min(p05s) if p05s else float("nan"))
            p95_y.append(np.max(p95s) if p95s else float("nan"))

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")
        plt.plot(latencies, mean_y, marker=mkr, color=col, linewidth=2, label=f"{sc} (Mean)")
        plt.fill_between(latencies, p05_y, p95_y, color=col, alpha=0.18, label=f"{sc} (Envelope)")

    plt.xlabel("Sensor Latency (s)", fontsize=11)
    plt.ylabel("AEB Activation Time (s)", fontsize=11)
    plt.title("Combined Impairment: AEB Activation Time vs Sensor Latency", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_braking_response_time_vs_actuator_tau(
    summary_records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot 90% braking response time vs brake actuator time constant tau."""
    plt.figure(figsize=(8, 5))
    markers, colors = _setup_plot_styles()
    scenarios = sorted(set(r["scenario"] for r in summary_records))
    taus = sorted(set(r["actuator_tau"] for r in summary_records))

    for sc in scenarios:
        mean_y, p05_y, p95_y = [], [], []
        for tau in taus:
            pts = [r for r in summary_records if r["scenario"] == sc and r["actuator_tau"] == tau]
            vals = [
                p["ninety_percent_braking_response_time_mean"]
                for p in pts
                if p["ninety_percent_braking_response_time_mean"] is not None
            ]
            p05s = [
                p["ninety_percent_braking_response_time_5th_percentile"]
                for p in pts
                if p["ninety_percent_braking_response_time_5th_percentile"] is not None
            ]
            p95s = [
                p["ninety_percent_braking_response_time_95th_percentile"]
                for p in pts
                if p["ninety_percent_braking_response_time_95th_percentile"] is not None
            ]
            mean_y.append(np.mean(vals) if vals else float("nan"))
            p05_y.append(np.min(p05s) if p05s else float("nan"))
            p95_y.append(np.max(p95s) if p95s else float("nan"))

        col = colors.get(sc, "#333333")
        mkr = markers.get(sc, "o")
        plt.plot(taus, mean_y, marker=mkr, color=col, linewidth=2, label=f"{sc} (Mean)")
        plt.fill_between(taus, p05_y, p95_y, color=col, alpha=0.18, label=f"{sc} (Envelope)")

    plt.xlabel("Brake Actuator Time Constant tau (s)", fontsize=11)
    plt.ylabel("90% Braking Response Time (s)", fontsize=11)
    plt.title("Combined Impairment: Braking Response Time vs Actuator Lag", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def plot_2d_heatmaps(
    summary_records: List[Dict[str, Any]],
    plots_dir: str,
) -> None:
    """Generate 2D heatmaps showing interaction between sensor noise and actuator tau."""
    scenarios = sorted(set(r["scenario"] for r in summary_records))
    noises = sorted(set(r["noise_std"] for r in summary_records))
    taus = sorted(set(r["actuator_tau"] for r in summary_records))

    # 1. Minimum Distance Heatmap (3-panel)
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), sharey=True)
    if not isinstance(axes, (list, np.ndarray)):
        axes = [axes]

    im_handles = []
    for idx, sc in enumerate(scenarios):
        ax = axes[idx]
        grid = np.zeros((len(noises), len(taus)))
        for i, n in enumerate(noises):
            for j, t in enumerate(taus):
                pts = [
                    r for r in summary_records
                    if r["scenario"] == sc and r["noise_std"] == n and r["actuator_tau"] == t
                ]
                vals = [p["minimum_distance_mean"] for p in pts if p["minimum_distance_mean"] is not None]
                grid[i, j] = np.mean(vals) if vals else float("nan")

        im = ax.imshow(
            grid,
            cmap="YlGnBu",
            origin="lower",
            aspect="auto",
        )
        im_handles.append(im)

        # Annotate cells
        for i in range(len(noises)):
            for j in range(len(taus)):
                val = grid[i, j]
                text_color = "white" if val > np.nanmean(grid) else "black"
                ax.text(j, i, f"{val:0.3f}m", ha="center", va="center", color=text_color, fontsize=9, fontweight="bold")

        ax.set_xticks(range(len(taus)))
        ax.set_xticklabels([f"{t:0.2f}s" for t in taus])
        ax.set_yticks(range(len(noises)))
        ax.set_yticklabels([f"{n:0.2f}m" for n in noises])
        ax.set_xlabel("Actuator tau", fontsize=10)
        if idx == 0:
            ax.set_ylabel("Sensor Noise Std Dev", fontsize=10)
        ax.set_title(f"{sc}\nMean Min Distance", fontsize=11, fontweight="bold")

    fig.subplots_adjust(top=0.86, bottom=0.15, left=0.07, right=0.90, wspace=0.25)
    cbar_ax = fig.add_axes([0.92, 0.15, 0.015, 0.71])
    fig.colorbar(im_handles[-1], cax=cbar_ax, label="Mean Minimum Distance (m)")
    fig.suptitle("2D Impairment Interaction: Minimum Distance (Noise std vs Actuator tau)", fontsize=13, fontweight="bold")
    plt.savefig(os.path.join(plots_dir, "heatmap_noise_vs_actuator_tau_minimum_distance.png"), dpi=300)
    plt.close()

    # 2. Collision Rate Heatmap (3-panel)
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), sharey=True)
    if not isinstance(axes, (list, np.ndarray)):
        axes = [axes]

    im_handles = []
    for idx, sc in enumerate(scenarios):
        ax = axes[idx]
        grid = np.zeros((len(noises), len(taus)))
        for i, n in enumerate(noises):
            for j, t in enumerate(taus):
                pts = [
                    r for r in summary_records
                    if r["scenario"] == sc and r["noise_std"] == n and r["actuator_tau"] == t
                ]
                total_samples = sum(p["sample_size"] for p in pts)
                total_collisions = sum(p["collision_count"] for p in pts)
                rate = total_collisions / total_samples if total_samples > 0 else 0.0
                grid[i, j] = rate

        im = ax.imshow(
            grid,
            cmap="Reds",
            origin="lower",
            aspect="auto",
            vmin=0.0,
            vmax=1.0,
        )
        im_handles.append(im)

        for i in range(len(noises)):
            for j in range(len(taus)):
                rate = grid[i, j]
                ax.text(j, i, f"{rate:0.1%}", ha="center", va="center", color="black", fontsize=9, fontweight="bold")

        ax.set_xticks(range(len(taus)))
        ax.set_xticklabels([f"{t:0.2f}s" for t in taus])
        ax.set_yticks(range(len(noises)))
        ax.set_yticklabels([f"{n:0.2f}m" for n in noises])
        ax.set_xlabel("Actuator tau", fontsize=10)
        if idx == 0:
            ax.set_ylabel("Sensor Noise Std Dev", fontsize=10)
        ax.set_title(f"{sc}\nCollision Rate", fontsize=11, fontweight="bold")

    fig.subplots_adjust(top=0.86, bottom=0.15, left=0.07, right=0.90, wspace=0.25)
    cbar_ax = fig.add_axes([0.92, 0.15, 0.015, 0.71])
    fig.colorbar(im_handles[-1], cax=cbar_ax, label="Collision Rate")
    fig.suptitle("2D Impairment Interaction: Collision Rate (Noise std vs Actuator tau)", fontsize=13, fontweight="bold")
    plt.savefig(os.path.join(plots_dir, "heatmap_noise_vs_actuator_tau_collision_rate.png"), dpi=300)
    plt.close()


def generate_all_plots(summary_records: List[Dict[str, Any]], plots_dir: str) -> None:
    """Generate all required 1D and 2D interaction figures."""
    plot_minimum_distance_vs_sensor_noise(
        summary_records,
        os.path.join(plots_dir, "minimum_distance_vs_sensor_noise.png"),
    )
    plot_collision_rate_vs_sensor_noise(
        summary_records,
        os.path.join(plots_dir, "collision_rate_vs_sensor_noise.png"),
    )
    plot_state_transitions_vs_sensor_noise(
        summary_records,
        os.path.join(plots_dir, "state_transitions_vs_sensor_noise.png"),
    )
    plot_triplet_reversals_vs_sensor_noise(
        summary_records,
        os.path.join(plots_dir, "triplet_reversal_count_vs_sensor_noise.png"),
    )
    plot_activation_time_vs_sensor_latency(
        summary_records,
        os.path.join(plots_dir, "activation_time_vs_sensor_latency.png"),
    )
    plot_braking_response_time_vs_actuator_tau(
        summary_records,
        os.path.join(plots_dir, "braking_response_time_vs_actuator_tau.png"),
    )
    plot_2d_heatmaps(summary_records, plots_dir)


if __name__ == "__main__":
    print("Executing Full Combined Sensor + Actuator Impairment Study (1440 runs)...")
    raw_res, summary_res = run_combined_impairment_sweep()
    print(f"Sweep complete. Total raw runs: {len(raw_res)}, Summary records: {len(summary_res)}")
