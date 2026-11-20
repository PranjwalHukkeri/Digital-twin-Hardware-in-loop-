"""
Main Execution Runner & Result Visualization for AEB Baseline Simulation.

Executes the closed-loop baseline AEB simulation, computes performance metrics,
prints a summary, and generates performance plots directly into plots/.
"""

import os
import csv
import matplotlib.pyplot as plt
from config.parameters import (
    TTC_WARNING,
    TTC_LEVEL_1,
    TTC_LEVEL_2,
    TTC_LEVEL_3,
    INITIAL_EGO_SPEED,
    INITIAL_TARGET_SPEED,
    INITIAL_DISTANCE,
)
from src.simulation import SimulationEngine
from src.metrics import PerformanceEvaluator


def generate_baseline_plots():
    """Run baseline simulation, compute performance metrics, and save plots & logs directly to plots/."""
    engine = SimulationEngine()
    result = engine.run(
        initial_ego_speed=INITIAL_EGO_SPEED,
        initial_target_speed=INITIAL_TARGET_SPEED,
        initial_distance=INITIAL_DISTANCE,
    )

    # 1. Compute and Print AEB Performance Metrics
    metrics = PerformanceEvaluator.evaluate(result)
    metrics.print_summary()

    times = result.time_history
    speeds = result.ego_speed_history
    distances = result.distance_history
    ttcs = result.ttc_history
    states = result.aeb_state_history
    decelerations = result.deceleration_history

    # Root project directory
    project_dir = os.path.dirname(os.path.abspath(__file__))
    plots_dir = os.path.join(project_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    # 2. Save Telemetry CSV Log directly in plots/
    csv_path = os.path.join(plots_dir, "simulation_telemetry_log.csv")
    with open(csv_path, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time_s",
                "ego_speed_m_s",
                "target_speed_m_s",
                "distance_m",
                "relative_velocity_m_s",
                "ttc_s",
                "aeb_state",
                "desired_deceleration_m_s2",
            ]
        )
        for s in result.samples:
            writer.writerow(
                [
                    s.time,
                    s.ego_speed,
                    s.target_speed,
                    s.distance,
                    s.relative_velocity,
                    s.ttc,
                    s.aeb_state,
                    s.desired_deceleration,
                ]
            )

    # State text-to-numeric mapping for step plotting
    state_mapping = {
        "SAFE": 0,
        "WARNING": 1,
        "AEB_LEVEL_1": 2,
        "AEB_LEVEL_2": 3,
        "AEB_LEVEL_3": 4,
    }
    numeric_states = [state_mapping.get(s, 0) for s in states]

    # Cap infinite TTC values for clean plotting
    capped_ttcs = [min(t_val, 6.0) for t_val in ttcs]

    # --------------------------------------------------------------------------
    # Combined Multi-Panel Figure (plots/baseline_aeb_simulation.png)
    # --------------------------------------------------------------------------
    fig, axs = plt.subplots(5, 1, figsize=(10, 14), sharex=True)
    fig.suptitle(
        "AEB Baseline Simulation Telemetry & State Performance",
        fontsize=14,
        fontweight="bold",
    )

    # Subplot 1: Ego Speed vs Time
    axs[0].plot(times, speeds, label="Ego Speed (m/s)")
    axs[0].set_ylabel("Speed (m/s)")
    axs[0].grid(True)
    axs[0].legend(loc="upper right")

    # Subplot 2: Distance to Target vs Time
    axs[1].plot(times, distances, label="Distance to Target (m)")
    axs[1].set_ylabel("Distance (m)")
    axs[1].grid(True)
    axs[1].legend(loc="upper right")

    # Subplot 3: TTC vs Time with Threshold Lines
    axs[2].plot(times, capped_ttcs, label="Time-To-Collision (s)")
    axs[2].axhline(y=TTC_WARNING, linestyle="--", label=f"Warning ({TTC_WARNING}s)")
    axs[2].axhline(y=TTC_LEVEL_1, linestyle="--", label=f"Level 1 ({TTC_LEVEL_1}s)")
    axs[2].axhline(y=TTC_LEVEL_2, linestyle="--", label=f"Level 2 ({TTC_LEVEL_2}s)")
    axs[2].axhline(y=TTC_LEVEL_3, linestyle="--", label=f"Level 3 ({TTC_LEVEL_3}s)")
    axs[2].set_ylabel("TTC (s)")
    axs[2].set_ylim(0, 6.5)
    axs[2].grid(True)
    axs[2].legend(loc="upper right", fontsize=8)

    # Subplot 4: Desired Deceleration vs Time
    axs[3].plot(times, decelerations, label="Desired Deceleration (m/s²)")
    axs[3].set_ylabel("Deceleration (m/s²)")
    axs[3].grid(True)
    axs[3].legend(loc="upper right")

    # Subplot 5: AEB State vs Time
    axs[4].step(times, numeric_states, where="post", label="AEB State")
    axs[4].set_yticks(list(state_mapping.values()))
    axs[4].set_yticklabels(list(state_mapping.keys()))
    axs[4].set_ylabel("AEB State")
    axs[4].set_xlabel("Time (s)")
    axs[4].grid(True)
    axs[4].legend(loc="upper right")

    plt.tight_layout()
    combined_plot_path = os.path.join(plots_dir, "baseline_aeb_simulation.png")
    fig.savefig(combined_plot_path, dpi=300)
    plt.close(fig)

    # --------------------------------------------------------------------------
    # Individual Plot Files
    # --------------------------------------------------------------------------
    fig1, ax1 = plt.subplots(figsize=(7, 4))
    ax1.plot(times, speeds)
    ax1.set_title("Ego Vehicle Speed vs Time")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Speed (m/s)")
    ax1.grid(True)
    fig1.savefig(os.path.join(plots_dir, "ego_speed_vs_time.png"), dpi=300)
    plt.close(fig1)

    fig2, ax2 = plt.subplots(figsize=(7, 4))
    ax2.plot(times, distances)
    ax2.set_title("Distance to Target vs Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Distance (m)")
    ax2.grid(True)
    fig2.savefig(os.path.join(plots_dir, "distance_vs_time.png"), dpi=300)
    plt.close(fig2)

    fig3, ax3 = plt.subplots(figsize=(7, 4))
    ax3.plot(times, capped_ttcs, label="TTC (s)")
    ax3.axhline(y=TTC_WARNING, linestyle="--", label=f"Warning ({TTC_WARNING}s)")
    ax3.axhline(y=TTC_LEVEL_1, linestyle="--", label=f"Level 1 ({TTC_LEVEL_1}s)")
    ax3.axhline(y=TTC_LEVEL_2, linestyle="--", label=f"Level 2 ({TTC_LEVEL_2}s)")
    ax3.axhline(y=TTC_LEVEL_3, linestyle="--", label=f"Level 3 ({TTC_LEVEL_3}s)")
    ax3.set_title("Time-To-Collision (TTC) vs Time")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("TTC (s)")
    ax3.set_ylim(0, 6.5)
    ax3.grid(True)
    ax3.legend(loc="upper right")
    fig3.savefig(os.path.join(plots_dir, "ttc_vs_time.png"), dpi=300)
    plt.close(fig3)

    fig4, ax4 = plt.subplots(figsize=(7, 4))
    ax4.plot(times, decelerations)
    ax4.set_title("Desired Deceleration vs Time")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Deceleration (m/s²)")
    ax4.grid(True)
    fig4.savefig(os.path.join(plots_dir, "deceleration_vs_time.png"), dpi=300)
    plt.close(fig4)

    fig5, ax5 = plt.subplots(figsize=(7, 4))
    ax5.step(times, numeric_states, where="post")
    ax5.set_yticks(list(state_mapping.values()))
    ax5.set_yticklabels(list(state_mapping.keys()))
    ax5.set_title("AEB State vs Time")
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("AEB State")
    ax5.grid(True)
    fig5.savefig(os.path.join(plots_dir, "aeb_state_vs_time.png"), dpi=300)
    plt.close(fig5)

    print("\nAll files saved directly in plots/:")
    for fname in sorted(os.listdir(plots_dir)):
        if not fname.startswith("."):
            print(f" - plots/{fname}")


if __name__ == "__main__":
    generate_baseline_plots()
