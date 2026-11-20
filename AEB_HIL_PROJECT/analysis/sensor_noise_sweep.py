"""
AEB Sensor Noise Sensitivity Analysis Sweep.

Evaluates the sensitivity of AEB performance across CCR, CCRm, and CCRb scenarios
subject to varying levels of zero-mean Gaussian distance measurement noise.

To isolate the effects of sensor measurement noise:
- Actuator dynamics are kept ideal: actuator_tau = 0.0 s
- Sensor latency is set to zero: sensor_latency = 0.0 s

Noise is applied as a controlled software sensitivity study on the separation distance
sensor (units: meters). Noise standard deviations are sensitivity-analysis parameters,
NOT hardware specifications or claims of VIRYA sensor performance.
"""

import os
import csv
import math
from typing import List, Dict, Any, Optional

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


def run_sensor_noise_sweep(
    noise_values: List[float] = [
        0.0,
        0.01,
        0.02,
        0.05,
        0.10,
        0.20,
    ],
    seed: int = 42,
    results_dir: str = "results/sensor_noise",
    plots_dir: str = "plots/sensor_noise",
) -> List[Dict[str, Any]]:
    """
    Execute distance sensor noise sensitivity sweep across CCR, CCRm, and CCRb scenarios.

    Args:
        noise_values: List of Gaussian noise standard deviation values (m).
        seed: Fixed random seed for reproducibility across noise sweeps.
        results_dir: Output directory for CSV results.
        plots_dir: Output directory for generated figures.

    Returns:
        List of performance summary dictionaries.
    """
    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

    # Secondary root results directory for convenience
    os.makedirs("results", exist_ok=True)

    scenarios = [
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

    records: List[Dict[str, Any]] = []

    for scenario in scenarios:
        for noise_std in noise_values:
            # Create a fresh engine for each noise level and scenario.
            # Only distance measurement noise is varied; latency and actuator tau remain 0.0.
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

            record = {
                "scenario": scenario.name,
                "noise_std": float(noise_std),
                "noise_std_cm": round(noise_std * 100.0, 2),
                "seed": seed,
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

            records.append(record)

    # ---------------------------------------------------------
    # Export CSV Datasets
    # ---------------------------------------------------------
    fieldnames = list(records[0].keys())

    csv_paths = [
        os.path.join(results_dir, "sensor_noise_sweep.csv"),
        os.path.join("results", "sensor_noise_sweep.csv"),
    ]

    for path in csv_paths:
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(records)

    # ---------------------------------------------------------
    # Generate Plots
    # ---------------------------------------------------------
    _plot_noise_vs_min_distance(
        records,
        os.path.join(plots_dir, "sensor_noise_vs_minimum_distance.png"),
    )

    _plot_noise_vs_ttc_activation(
        records,
        os.path.join(plots_dir, "sensor_noise_vs_ttc_at_activation.png"),
    )

    _plot_noise_vs_activation_time(
        records,
        os.path.join(plots_dir, "sensor_noise_vs_activation_time.png"),
    )

    _plot_noise_vs_state_transitions(
        records,
        os.path.join(plots_dir, "sensor_noise_vs_state_transitions.png"),
    )

    _plot_noise_vs_chattering(
        records,
        os.path.join(plots_dir, "sensor_noise_vs_chattering.png"),
    )

    _plot_noise_vs_triplet_reversals(
        records,
        os.path.join(plots_dir, "sensor_noise_vs_triplet_reversal_count.png"),
    )

    return records


def _plot_noise_vs_min_distance(
    records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot distance sensor noise standard deviation vs minimum separation distance."""
    plt.figure(figsize=(8, 5))

    scenarios = sorted(set(r["scenario"] for r in records))
    markers = {"CCR_Stationary_Target": "o", "CCRm_Moving_Target": "s", "CCRb_Braking_Target": "^"}

    for scenario in scenarios:
        scenario_records = [r for r in records if r["scenario"] == scenario]
        x = [r["noise_std"] for r in scenario_records]
        y = [r["minimum_distance"] for r in scenario_records]

        plt.plot(
            x,
            y,
            marker=markers.get(scenario, "o"),
            linewidth=2,
            markersize=6,
            label=scenario,
        )

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("Minimum Distance (m)", fontsize=11)
    plt.title("Sensor Noise Sensitivity: Minimum Separation Distance", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_noise_vs_ttc_activation(
    records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot distance sensor noise standard deviation vs TTC at AEB activation."""
    plt.figure(figsize=(8, 5))

    scenarios = sorted(set(r["scenario"] for r in records))
    markers = {"CCR_Stationary_Target": "o", "CCRm_Moving_Target": "s", "CCRb_Braking_Target": "^"}

    for scenario in scenarios:
        scenario_records = [r for r in records if r["scenario"] == scenario]
        x = [r["noise_std"] for r in scenario_records]
        y = [
            r["ttc_at_activation"] if r["ttc_at_activation"] is not None else float("nan")
            for r in scenario_records
        ]

        plt.plot(
            x,
            y,
            marker=markers.get(scenario, "o"),
            linewidth=2,
            markersize=6,
            label=scenario,
        )

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("TTC at AEB Activation (s)", fontsize=11)
    plt.title("Sensor Noise Sensitivity: TTC at AEB Activation", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_noise_vs_activation_time(
    records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot distance sensor noise standard deviation vs AEB activation time."""
    plt.figure(figsize=(8, 5))

    scenarios = sorted(set(r["scenario"] for r in records))
    markers = {"CCR_Stationary_Target": "o", "CCRm_Moving_Target": "s", "CCRb_Braking_Target": "^"}

    for scenario in scenarios:
        scenario_records = [r for r in records if r["scenario"] == scenario]
        x = [r["noise_std"] for r in scenario_records]
        y = [
            r["activation_time"] if r["activation_time"] is not None else float("nan")
            for r in scenario_records
        ]

        plt.plot(
            x,
            y,
            marker=markers.get(scenario, "o"),
            linewidth=2,
            markersize=6,
            label=scenario,
        )

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("AEB Activation Time (s)", fontsize=11)
    plt.title("Sensor Noise Sensitivity: AEB Activation Time", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_noise_vs_state_transitions(
    records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot distance sensor noise standard deviation vs number of AEB state transitions."""
    plt.figure(figsize=(8, 5))

    scenarios = sorted(set(r["scenario"] for r in records))
    markers = {"CCR_Stationary_Target": "o", "CCRm_Moving_Target": "s", "CCRb_Braking_Target": "^"}

    for scenario in scenarios:
        scenario_records = [r for r in records if r["scenario"] == scenario]
        x = [r["noise_std"] for r in scenario_records]
        y = [r["state_transition_count"] for r in scenario_records]

        plt.plot(
            x,
            y,
            marker=markers.get(scenario, "o"),
            linewidth=2,
            markersize=6,
            label=scenario,
        )

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("AEB FSM State Transitions", fontsize=11)
    plt.title("Sensor Noise Sensitivity: FSM State Transition Count", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_noise_vs_chattering(
    records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot distance sensor noise standard deviation vs boundary chattering detection."""
    plt.figure(figsize=(8, 5))

    scenarios = sorted(set(r["scenario"] for r in records))
    markers = {
        "CCR_Stationary_Target": "o",
        "CCRm_Moving_Target": "s",
        "CCRb_Braking_Target": "^",
    }
    # Minor vertical offset for distinct visibility if multiple scenarios overlap at binary 0 or 1
    offsets = {
        "CCR_Stationary_Target": 0.015,
        "CCRm_Moving_Target": 0.0,
        "CCRb_Braking_Target": -0.015,
    }

    for scenario in scenarios:
        scenario_records = [r for r in records if r["scenario"] == scenario]
        x = [r["noise_std"] for r in scenario_records]
        y = [
            (1 if r["boundary_chattering_detected"] else 0) + offsets.get(scenario, 0.0)
            for r in scenario_records
        ]

        plt.plot(
            x,
            y,
            marker=markers.get(scenario, "o"),
            linewidth=2,
            markersize=6,
            label=scenario,
        )

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("Boundary Chattering Detected", fontsize=11)
    plt.title(
        "Sensor Noise Sensitivity: Boundary Chattering Detection",
        fontsize=12,
        fontweight="bold",
    )
    plt.ylim(-0.15, 1.15)
    plt.yticks([0, 1], ["False (0)", "True (1)"])
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_noise_vs_triplet_reversals(
    records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot distance sensor noise standard deviation vs number of 3-sample sliding FSM reversal patterns."""
    plt.figure(figsize=(8, 5))

    scenarios = sorted(set(r["scenario"] for r in records))
    markers = {
        "CCR_Stationary_Target": "o",
        "CCRm_Moving_Target": "s",
        "CCRb_Braking_Target": "^",
    }

    for scenario in scenarios:
        scenario_records = [r for r in records if r["scenario"] == scenario]
        x = [r["noise_std"] for r in scenario_records]
        y = [r["triplet_reversal_count"] for r in scenario_records]

        plt.plot(
            x,
            y,
            marker=markers.get(scenario, "o"),
            linewidth=2,
            markersize=6,
            label=scenario,
        )

    plt.xlabel("Distance Sensor Noise Std Dev (m)", fontsize=11)
    plt.ylabel("Triplet Reversal Pattern Count", fontsize=11)
    plt.title(
        "Sensor Noise Sensitivity: 3-Sample Sliding FSM Reversal Patterns",
        fontsize=12,
        fontweight="bold",
    )
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


if __name__ == "__main__":
    results = run_sensor_noise_sweep()

    print("\nSensor noise sweep completed.")
    print(f"Runs completed: {len(results)}")
    print("\nSummary Results:")
    for r in results:
        print(
            f"{r['scenario']:25s} | "
            f"Noise Std: {r['noise_std']:4.2f} m | "
            f"Min Dist: {r['minimum_distance']:6.4f} m | "
            f"TTC Act: {r['ttc_at_activation']} s | "
            f"Transitions: {r['state_transition_count']:2d} | "
            f"Chattering: {r['boundary_chattering_detected']} | "
            f"Triplet Rev: {r['triplet_reversal_count']:2d} | "
            f"Episodes: {r['braking_episode_count']} | "
            f"Collision: {r['collision_status']}"
        )
