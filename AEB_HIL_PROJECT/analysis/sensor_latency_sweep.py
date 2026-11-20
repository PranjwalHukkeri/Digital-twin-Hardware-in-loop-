"""
AEB Sensor Latency Analysis Sweep.

Evaluates the impact of sensor measurement latency on AEB performance
across CCR, CCRm, and CCRb scenarios.

The brake actuator is kept ideal (tau=0.0 s) so that this experiment
isolates the effect of sensor latency.
"""

import os
import csv
import math
from typing import List, Dict, Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.simulation import SimulationEngine
from src.metrics import PerformanceEvaluator
from scenarios.scenarios import (
    CCRScenario,
    CCRmScenario,
    CCRbScenario,
)


def run_sensor_latency_sweep(
    latency_values: List[float] = [
        0.0,
        0.05,
        0.10,
        0.15,
        0.20,
        0.30,
    ],
    results_dir: str = "results/sensor_latency",
    plots_dir: str = "plots/sensor_latency",
) -> List[Dict[str, Any]]:
    """
    Run sensor-latency sweep across CCR, CCRm, and CCRb scenarios.

    Args:
        latency_values: Sensor latency values in seconds.
        results_dir: Directory for CSV results.
        plots_dir: Directory for generated plots.

    Returns:
        List of performance summary records.
    """

    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

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

    records = []

    for scenario in scenarios:
        for latency in latency_values:

            # Create a fresh engine for each latency value.
            # Actuator remains ideal so only sensor latency varies.
            engine = SimulationEngine(
                sensor_latency=latency,
                actuator_tau=0.0,
            )

            result = engine.run(
                scenario=scenario,
            )

            metrics = PerformanceEvaluator.evaluate(result)

            record = {
                "scenario": scenario.name,
                "sensor_latency": latency,
                "sensor_latency_ms": round(latency * 1000.0, 1),

                "activation_time": metrics.aeb_activation_time,

                "ttc_at_activation": (
                    metrics.ttc_at_activation
                    if metrics.ttc_at_activation is not None
                    else None
                ),

                "max_commanded_deceleration": round(
                    metrics.max_braking_deceleration,
                    4,
                ),

                "max_actual_deceleration": round(
                    metrics.max_actual_deceleration,
                    4,
                ),

                "ninety_percent_braking_response_time": (
                    metrics.ninety_percent_braking_response_time
                ),

                "minimum_distance": round(
                    metrics.min_distance,
                    4,
                ),

                "stopping_distance": (
                    round(metrics.stopping_distance, 4)
                    if metrics.stopping_distance is not None
                    else None
                ),

                "final_ego_speed": round(
                    metrics.final_ego_speed,
                    4,
                ),

                "final_relative_velocity": round(
                    metrics.final_relative_velocity,
                    4,
                ),

                "min_relative_velocity": round(
                    metrics.min_relative_velocity,
                    4,
                ),

                "vehicle_stop_status": metrics.vehicle_stop_status,

                "collision_status": metrics.collision_status,
            }

            records.append(record)

    # ---------------------------------------------------------
    # Export CSV
    # ---------------------------------------------------------

    csv_path = os.path.join(
        results_dir,
        "sensor_latency_sweep.csv",
    )

    fieldnames = list(records[0].keys())

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
        )
        writer.writeheader()
        writer.writerows(records)

    # ---------------------------------------------------------
    # Generate plots
    # ---------------------------------------------------------

    _plot_latency_vs_min_distance(
        records,
        os.path.join(
            plots_dir,
            "sensor_latency_vs_minimum_distance.png",
        ),
    )

    _plot_latency_vs_activation_time(
        records,
        os.path.join(
            plots_dir,
            "sensor_latency_vs_activation_time.png",
        ),
    )

    _plot_latency_vs_ttc_activation(
        records,
        os.path.join(
            plots_dir,
            "sensor_latency_vs_ttc_at_activation.png",
        ),
    )

    return records


def _plot_latency_vs_min_distance(
    records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot sensor latency against minimum separation distance."""

    plt.figure(figsize=(8, 5))

    scenarios = sorted(
        set(record["scenario"] for record in records)
    )

    for scenario in scenarios:
        scenario_records = [
            record
            for record in records
            if record["scenario"] == scenario
        ]

        x = [
            record["sensor_latency_ms"]
            for record in scenario_records
        ]

        y = [
            record["minimum_distance"]
            for record in scenario_records
        ]

        plt.plot(
            x,
            y,
            marker="o",
            linewidth=2,
            label=scenario,
        )

    plt.xlabel(
        "Sensor Latency (ms)",
        fontsize=11,
    )

    plt.ylabel(
        "Minimum Distance (m)",
        fontsize=11,
    )

    plt.title(
        "Sensor Latency vs Minimum Separation Distance",
        fontsize=12,
        fontweight="bold",
    )

    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_latency_vs_activation_time(
    records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot sensor latency against AEB activation time."""

    plt.figure(figsize=(8, 5))

    scenarios = sorted(
        set(record["scenario"] for record in records)
    )

    for scenario in scenarios:
        scenario_records = [
            record
            for record in records
            if record["scenario"] == scenario
        ]

        x = [
            record["sensor_latency_ms"]
            for record in scenario_records
        ]

        y = [
            record["activation_time"]
            if record["activation_time"] is not None
            else float("nan")
            for record in scenario_records
        ]

        plt.plot(
            x,
            y,
            marker="s",
            linewidth=2,
            label=scenario,
        )

    plt.xlabel(
        "Sensor Latency (ms)",
        fontsize=11,
    )

    plt.ylabel(
        "AEB Activation Time (s)",
        fontsize=11,
    )

    plt.title(
        "Sensor Latency vs AEB Activation Time",
        fontsize=12,
        fontweight="bold",
    )

    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_latency_vs_ttc_activation(
    records: List[Dict[str, Any]],
    filepath: str,
) -> None:
    """Plot sensor latency against TTC at AEB activation."""

    plt.figure(figsize=(8, 5))

    scenarios = sorted(
        set(record["scenario"] for record in records)
    )

    for scenario in scenarios:
        scenario_records = [
            record
            for record in records
            if record["scenario"] == scenario
        ]

        x = [
            record["sensor_latency_ms"]
            for record in scenario_records
        ]

        y = [
            record["ttc_at_activation"]
            if record["ttc_at_activation"] is not None
            else float("nan")
            for record in scenario_records
        ]

        plt.plot(
            x,
            y,
            marker="^",
            linewidth=2,
            label=scenario,
        )

    plt.xlabel(
        "Sensor Latency (ms)",
        fontsize=11,
    )

    plt.ylabel(
        "TTC at AEB Activation (s)",
        fontsize=11,
    )

    plt.title(
        "Sensor Latency vs TTC at AEB Activation",
        fontsize=12,
        fontweight="bold",
    )

    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


if __name__ == "__main__":
    results = run_sensor_latency_sweep()

    print("\nSensor latency sweep completed.")
    print(f"Runs completed: {len(results)}")

    print("\nResults:")

    for record in results:
        print(
            f"{record['scenario']:28s} | "
            f"Latency: {record['sensor_latency_ms']:6.1f} ms | "
            f"Min distance: {record['minimum_distance']:7.4f} m | "
            f"Activation: {record['activation_time']} | "
            f"Collision: {record['collision_status']}"
        )
