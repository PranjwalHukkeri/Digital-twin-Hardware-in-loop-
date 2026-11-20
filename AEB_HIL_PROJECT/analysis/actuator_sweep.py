"""
AEB Actuator Dynamics Analysis Sweep Module.

Evaluates the impact of brake actuator dynamic time constant (tau) on AEB performance
metrics across CCR, CCRm, and CCRb benchmark scenarios using exact ZOH discretization.

Exports CSV datasets to results/actuator_dynamics/ and generates standalone plots under plots/actuator_dynamics/.
"""

import os
import csv
import math
from typing import List, Dict, Any
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.simulation import SimulationEngine, SimulationResult
from src.metrics import PerformanceEvaluator
from scenarios.scenarios import CCRScenario, CCRmScenario, CCRbScenario


def run_actuator_tau_sweep(
    tau_values: List[float] = [0.0, 0.05, 0.10, 0.15, 0.20, 0.30],
    results_dir: str = "results/actuator_dynamics",
    plots_dir: str = "plots/actuator_dynamics",
) -> List[Dict[str, Any]]:
    """
    Run actuator dynamics parameter sweep across CCR, CCRm, and CCRb scenarios.

    Args:
        tau_values (List[float]): Actuator time constants in seconds. tau=0.0 represents ideal.
        results_dir (str): Directory path for CSV exports.
        plots_dir (str): Directory path for plot images.

    Returns:
        List[Dict[str, Any]]: Trajectory summary records.
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
    ccr_ideal_result = None
    ccr_tau015_result = None

    for scenario in scenarios:
        for tau in tau_values:
            engine = SimulationEngine()
            result = engine.run(scenario=scenario, actuator_tau=tau)
            metrics = PerformanceEvaluator.evaluate(result)

            if scenario.name == "CCR_Stationary_Target":
                if math.isclose(tau, 0.0, abs_tol=1e-5):
                    ccr_ideal_result = result
                elif math.isclose(tau, 0.15, abs_tol=1e-5):
                    ccr_tau015_result = result

            record = {
                "scenario": scenario.name,
                "actuator_tau": tau,
                "activation_time": metrics.aeb_activation_time,
                "ttc_at_activation": metrics.ttc_at_activation,
                "max_commanded_deceleration": round(metrics.max_braking_deceleration, 4),
                "max_actual_deceleration": round(metrics.max_actual_deceleration, 4),
                "ninety_percent_braking_response_time": metrics.ninety_percent_braking_response_time,
                "minimum_distance": round(metrics.min_distance, 4),
                "stopping_distance": round(metrics.stopping_distance, 4) if metrics.stopping_distance is not None else None,
                "final_ego_speed": round(metrics.final_ego_speed, 4),
                "final_relative_velocity": round(metrics.final_relative_velocity, 4),
                "min_relative_velocity": round(metrics.min_relative_velocity, 4),
                "vehicle_stop_status": metrics.vehicle_stop_status,
                "collision_status": metrics.collision_status,
            }
            sweep_records.append(record)

    # Export main CSV
    csv_path = os.path.join(results_dir, "actuator_sweep.csv")
    fieldnames = list(sweep_records[0].keys())
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(sweep_records)

    # Export Detailed Diagnostic CSV and Comparison Plot for CCR (tau=0 vs tau=0.15)
    if ccr_ideal_result is not None and ccr_tau015_result is not None:
        _export_comparison_diagnostic(
            ccr_ideal_result,
            ccr_tau015_result,
            os.path.join(results_dir, "ccr_ideal_vs_tau_0.15_comparison.csv"),
        )
        _plot_ideal_vs_tau_015_deceleration(
            ccr_ideal_result,
            ccr_tau015_result,
            os.path.join(plots_dir, "actuator_ideal_vs_tau_015_comparison.png"),
        )

    # Generate Standard Plots
    _plot_tau_vs_min_distance(sweep_records, os.path.join(plots_dir, "actuator_tau_vs_minimum_distance.png"))
    _plot_tau_vs_max_actual_decel(sweep_records, os.path.join(plots_dir, "actuator_tau_vs_max_actual_deceleration.png"))
    _plot_tau_vs_stopping_distance(sweep_records, os.path.join(plots_dir, "actuator_tau_vs_stopping_distance.png"))

    return sweep_records


def _export_comparison_diagnostic(
    res_ideal: SimulationResult, res_tau015: SimulationResult, filepath: str
) -> None:
    """Export comparison CSV matching ideal (tau=0) and dynamic (tau=0.15) trajectories timestep-by-timestep."""
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    records = []
    max_len = max(len(res_ideal.samples), len(res_tau015.samples))

    for i in range(max_len):
        s0 = res_ideal.samples[i] if i < len(res_ideal.samples) else None
        s15 = res_tau015.samples[i] if i < len(res_tau015.samples) else None
        t_val = s0.time if s0 is not None else s15.time

        records.append({
            "time": t_val,
            "TTC_tau_0": round(s0.ttc, 4) if (s0 and not math.isinf(s0.ttc)) else ("inf" if s0 else ""),
            "state_tau_0": s0.aeb_state if s0 else "",
            "desired_decel_tau_0": round(s0.desired_deceleration, 4) if s0 else "",
            "actual_decel_tau_0": round(s0.actual_deceleration, 4) if s0 else "",
            "ego_speed_tau_0": round(s0.ego_speed, 4) if s0 else "",
            "distance_tau_0": round(s0.distance, 4) if s0 else "",
            "TTC_tau_015": round(s15.ttc, 4) if (s15 and not math.isinf(s15.ttc)) else ("inf" if s15 else ""),
            "state_tau_015": s15.aeb_state if s15 else "",
            "desired_decel_tau_015": round(s15.desired_deceleration, 4) if s15 else "",
            "actual_decel_tau_015": round(s15.actual_deceleration, 4) if s15 else "",
            "ego_speed_tau_015": round(s15.ego_speed, 4) if s15 else "",
            "distance_tau_015": round(s15.distance, 4) if s15 else "",
        })

    with open(filepath, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)


def _plot_ideal_vs_tau_015_deceleration(
    res_ideal: SimulationResult, res_tau015: SimulationResult, filepath: str
) -> None:
    """Plot comparison between ideal command/deceleration and tau=0.15s command/actual deceleration."""
    plt.figure(figsize=(8, 5))
    plt.plot(
        res_ideal.time_history,
        res_ideal.deceleration_history,
        label="Ideal (tau=0.0s) Command/Actual",
        linestyle="--",
        linewidth=1.5,
    )
    plt.plot(
        res_tau015.time_history,
        res_tau015.deceleration_history,
        label="tau=0.15s Desired Command",
        linestyle=":",
        linewidth=2,
    )
    plt.plot(
        res_tau015.time_history,
        res_tau015.actual_deceleration_history,
        label="tau=0.15s Actual Output",
        linestyle="-",
        linewidth=2,
    )

    plt.xlabel("Time (s)", fontsize=11)
    plt.ylabel("Deceleration (m/s²)", fontsize=11)
    plt.title(
        "CCR Deceleration Comparison: Ideal (tau=0.0s) vs Dynamic (tau=0.15s)",
        fontsize=12,
        fontweight="bold",
    )
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


# Plot Helper Functions
def _plot_tau_vs_min_distance(records: List[Dict[str, Any]], filepath: str) -> None:
    plt.figure(figsize=(8, 5))
    scenarios = set(r["scenario"] for r in records)
    for sc in scenarios:
        sc_recs = [r for r in records if r["scenario"] == sc]
        x = [r["actuator_tau"] for r in sc_recs]
        y = [r["minimum_distance"] for r in sc_recs]
        plt.plot(x, y, marker="o", linewidth=2, label=sc)

    plt.xlabel("Actuator Time Constant tau (s)", fontsize=11)
    plt.ylabel("Minimum Distance (m)", fontsize=11)
    plt.title("Actuator Time Constant vs Minimum Separation Distance", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_tau_vs_max_actual_decel(records: List[Dict[str, Any]], filepath: str) -> None:
    plt.figure(figsize=(8, 5))
    scenarios = set(r["scenario"] for r in records)
    for sc in scenarios:
        sc_recs = [r for r in records if r["scenario"] == sc]
        x = [r["actuator_tau"] for r in sc_recs]
        y = [r["max_actual_deceleration"] for r in sc_recs]
        plt.plot(x, y, marker="s", linewidth=2, label=sc)

    plt.xlabel("Actuator Time Constant tau (s)", fontsize=11)
    plt.ylabel("Max Actual Deceleration (m/s²)", fontsize=11)
    plt.title("Actuator Time Constant vs Maximum Actual Deceleration", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()


def _plot_tau_vs_stopping_distance(records: List[Dict[str, Any]], filepath: str) -> None:
    plt.figure(figsize=(8, 5))
    scenarios = set(r["scenario"] for r in records)
    for sc in scenarios:
        sc_recs = [r for r in records if r["scenario"] == sc]
        x = [r["actuator_tau"] for r in sc_recs]
        y = [
            r["stopping_distance"]
            if r["vehicle_stop_status"] == "STOPPED" and r["stopping_distance"] is not None
            else float("nan")
            for r in sc_recs
        ]
        plt.plot(x, y, marker="^", linewidth=2, label=sc)

    plt.xlabel("Actuator Time Constant tau (s)", fontsize=11)
    plt.ylabel("Ego Stopping Distance (m)", fontsize=11)
    plt.title("Actuator Time Constant vs Ego Stopping Distance", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filepath, dpi=300)
    plt.close()
