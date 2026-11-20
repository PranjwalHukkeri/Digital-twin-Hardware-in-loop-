"""
Unit Tests for Combined Sensor + Actuator Impairment Robustness Study.

Verifies:
- Exact aggregation math across multi-dimensional conditions
- Fast execution on reduced test matrix (8 runs)
- CSV schema and integrity for raw and summary datasets
- Successful creation of all 1D and 2D interaction figures
"""

import os
import csv
import pytest
import numpy as np

from analysis.combined_impairment import (
    aggregate_combined_results,
    run_combined_impairment_sweep,
)
from scenarios.scenarios import CCRScenario


def test_combined_impairment_aggregation_math():
    """Verify statistical aggregation math across multi-dimensional conditions."""
    raw_records = [
        {
            "scenario": "CCR_Test",
            "noise_std": 0.05,
            "noise_std_cm": 5.0,
            "sensor_latency": 0.05,
            "actuator_tau": 0.10,
            "seed": 42,
            "minimum_distance": 1.5,
            "collision_status": False,
            "aeb_activation_time": 1.0,
            "activation_time": 1.0,
            "ttc_at_activation": 2.5,
            "state_transition_count": 12,
            "triplet_reversal_count": 2,
            "max_braking_deceleration": 6.0,
            "max_actual_deceleration": 6.0,
            "ninety_percent_braking_response_time": 0.23,
            "braking_response_time": 0.23,
            "stopping_time": 2.5,
            "stopping_distance": 15.0,
            "vehicle_stop_status": "STOPPED",
            "final_ego_speed": 0.0,
            "braking_episode_count": 1,
            "total_braking_duration": 2.0,
            "boundary_chattering_detected": True,
        },
        {
            "scenario": "CCR_Test",
            "noise_std": 0.05,
            "noise_std_cm": 5.0,
            "sensor_latency": 0.05,
            "actuator_tau": 0.10,
            "seed": 43,
            "minimum_distance": 0.5,
            "collision_status": True,
            "aeb_activation_time": 1.1,
            "activation_time": 1.1,
            "ttc_at_activation": 2.4,
            "state_transition_count": 16,
            "triplet_reversal_count": 4,
            "max_braking_deceleration": 6.0,
            "max_actual_deceleration": 6.0,
            "ninety_percent_braking_response_time": 0.25,
            "braking_response_time": 0.25,
            "stopping_time": 2.6,
            "stopping_distance": 16.0,
            "vehicle_stop_status": "NOT_STOPPED",
            "final_ego_speed": 0.5,
            "braking_episode_count": 2,
            "total_braking_duration": 2.2,
            "boundary_chattering_detected": True,
        },
    ]

    summary = aggregate_combined_results(raw_records)
    assert len(summary) == 1
    row = summary[0]

    assert row["scenario"] == "CCR_Test"
    assert row["noise_std"] == 0.05
    assert row["sensor_latency"] == 0.05
    assert row["actuator_tau"] == 0.10
    assert row["sample_size"] == 2
    assert row["collision_count"] == 1
    assert row["collision_rate"] == 0.5
    assert row["stopped_count"] == 1
    assert row["stopped_rate"] == 0.5

    assert row["minimum_distance_mean"] == 1.0
    assert row["minimum_distance_min"] == 0.5
    assert row["minimum_distance_max"] == 1.5
    assert row["state_transition_count_mean"] == 14.0
    assert row["triplet_reversal_count_mean"] == 3.0


def test_run_combined_impairment_reduced_config(tmp_path):
    """Verify combined impairment sweep on reduced 8-run test matrix."""
    results_dir = str(tmp_path / "results")
    plots_dir = str(tmp_path / "plots")

    test_scenarios = [
        CCRScenario(
            name="CCR_Stationary_Target",
            initial_ego_speed=15.0,
            initial_distance=30.0,
        )
    ]
    test_noises = [0.00, 0.05]
    test_latencies = [0.00]
    test_taus = [0.00, 0.10]
    test_seeds = [42, 43]

    raw_records, summary_records = run_combined_impairment_sweep(
        scenarios=test_scenarios,
        noise_values=test_noises,
        latency_values=test_latencies,
        actuator_tau_values=test_taus,
        seeds=test_seeds,
        results_dir=results_dir,
        plots_dir=plots_dir,
    )

    # 1 scenario * 2 noises * 1 latency * 2 taus * 2 seeds = 8 runs
    assert len(raw_records) == 8
    # 1 scenario * 2 noises * 1 latency * 2 taus = 4 summary rows
    assert len(summary_records) == 4

    raw_csv = os.path.join(results_dir, "combined_impairment_raw.csv")
    summary_csv = os.path.join(results_dir, "combined_impairment_summary.csv")
    assert os.path.exists(raw_csv)
    assert os.path.exists(summary_csv)

    # Check raw CSV columns
    with open(raw_csv, "r") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        required = [
            "scenario",
            "noise_std",
            "noise_std_cm",
            "sensor_latency",
            "actuator_tau",
            "seed",
            "minimum_distance",
            "collision_status",
            "aeb_activation_time",
            "ttc_at_activation",
            "state_transition_count",
            "triplet_reversal_count",
            "max_braking_deceleration",
            "max_actual_deceleration",
            "ninety_percent_braking_response_time",
            "stopping_time",
            "stopping_distance",
            "vehicle_stop_status",
        ]
        for req in required:
            assert req in fieldnames

        for row in reader:
            assert int(row["triplet_reversal_count"]) >= 0
            assert row["collision_status"] in ["True", "False"]

    # Check summary CSV columns
    with open(summary_csv, "r") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        assert "collision_rate" in fieldnames
        assert "minimum_distance_mean" in fieldnames
        assert "triplet_reversal_count_mean" in fieldnames

    # Check plot outputs
    assert os.path.exists(os.path.join(plots_dir, "minimum_distance_vs_sensor_noise.png"))
    assert os.path.exists(os.path.join(plots_dir, "collision_rate_vs_sensor_noise.png"))
    assert os.path.exists(os.path.join(plots_dir, "state_transitions_vs_sensor_noise.png"))
    assert os.path.exists(os.path.join(plots_dir, "triplet_reversal_count_vs_sensor_noise.png"))
    assert os.path.exists(os.path.join(plots_dir, "activation_time_vs_sensor_latency.png"))
    assert os.path.exists(os.path.join(plots_dir, "braking_response_time_vs_actuator_tau.png"))
    assert os.path.exists(os.path.join(plots_dir, "heatmap_noise_vs_actuator_tau_minimum_distance.png"))
    assert os.path.exists(os.path.join(plots_dir, "heatmap_noise_vs_actuator_tau_collision_rate.png"))
