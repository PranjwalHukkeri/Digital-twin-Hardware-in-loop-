"""
Unit Tests for Multi-Seed Sensor Noise Robustness Analysis.

Verifies:
- Statistical aggregation math (mean, std, min, max, percentiles, collision rate)
- Execution of multi-seed sweep with a fast reduced configuration
- Generation and integrity of raw and summary CSV records
- Generation of multi-seed sensitivity plots
"""

import os
import csv
import pytest
import numpy as np

from analysis.sensor_noise_multiseed import (
    aggregate_multiseed_results,
    run_sensor_noise_multiseed,
)
from scenarios.scenarios import CCRScenario


def test_multiseed_aggregation_math():
    """Verify exact mathematical aggregation of raw records across seeds."""
    raw_records = [
        {
            "scenario": "CCR_Test",
            "noise_std": 0.05,
            "noise_std_cm": 5.0,
            "seed": 42,
            "activation_time": 1.0,
            "ttc_at_activation": 2.5,
            "minimum_distance": 1.2,
            "max_braking_deceleration": 6.0,
            "max_actual_deceleration": 6.0,
            "braking_response_time": 0.1,
            "final_ego_speed": 0.0,
            "collision_status": False,
            "vehicle_stop_status": "STOPPED",
            "state_transition_count": 10,
            "boundary_chattering_detected": True,
            "triplet_reversal_count": 2,
            "braking_episode_count": 1,
            "total_braking_duration": 2.0,
            "stopping_distance": 15.0,
        },
        {
            "scenario": "CCR_Test",
            "noise_std": 0.05,
            "noise_std_cm": 5.0,
            "seed": 43,
            "activation_time": 1.2,
            "ttc_at_activation": 2.3,
            "minimum_distance": 0.8,
            "max_braking_deceleration": 6.0,
            "max_actual_deceleration": 6.0,
            "braking_response_time": 0.1,
            "final_ego_speed": 0.2,
            "collision_status": True,
            "vehicle_stop_status": "NOT_STOPPED",
            "state_transition_count": 14,
            "boundary_chattering_detected": True,
            "triplet_reversal_count": 4,
            "braking_episode_count": 2,
            "total_braking_duration": 2.4,
            "stopping_distance": 16.0,
        },
    ]

    summary = aggregate_multiseed_results(raw_records)
    assert len(summary) == 1
    row = summary[0]

    assert row["scenario"] == "CCR_Test"
    assert row["noise_std"] == 0.05
    assert row["sample_size"] == 2
    assert row["collision_count"] == 1
    assert row["collision_rate"] == 0.5
    assert row["stopped_count"] == 1
    assert row["stopped_rate"] == 0.5

    # Minimum distance: [1.2, 0.8] -> mean=1.0, min=0.8, max=1.2
    assert row["minimum_distance_mean"] == 1.0
    assert row["minimum_distance_min"] == 0.8
    assert row["minimum_distance_max"] == 1.2
    assert row["minimum_distance_5th_percentile"] <= row["minimum_distance_95th_percentile"]

    # State transitions: [10, 14] -> mean=12.0
    assert row["state_transition_count_mean"] == 12.0

    # Triplet reversals: [2, 4] -> mean=3.0
    assert row["triplet_reversal_count_mean"] == 3.0


def test_run_sensor_noise_multiseed_reduced_config(tmp_path):
    """Verify execution of multi-seed sweep on reduced test configuration."""
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
    test_seeds = [42, 43]

    raw_records, summary_records = run_sensor_noise_multiseed(
        scenarios=test_scenarios,
        noise_values=test_noises,
        seeds=test_seeds,
        results_dir=results_dir,
        plots_dir=plots_dir,
    )

    # 1 scenario * 2 noise values * 2 seeds = 4 runs
    assert len(raw_records) == 4
    # 1 scenario * 2 noise values = 2 summary rows
    assert len(summary_records) == 2

    # Verify CSV files
    raw_csv = os.path.join(results_dir, "sensor_noise_multiseed_raw.csv")
    summary_csv = os.path.join(results_dir, "sensor_noise_multiseed_summary.csv")
    assert os.path.exists(raw_csv)
    assert os.path.exists(summary_csv)

    # Check raw CSV fields
    with open(raw_csv, "r") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        required_fields = [
            "scenario",
            "noise_std",
            "noise_std_cm",
            "seed",
            "activation_time",
            "ttc_at_activation",
            "minimum_distance",
            "max_braking_deceleration",
            "max_actual_deceleration",
            "braking_response_time",
            "final_ego_speed",
            "collision_status",
            "vehicle_stop_status",
            "state_transition_count",
            "triplet_reversal_count",
            "braking_episode_count",
            "total_braking_duration",
            "stopping_distance",
        ]
        for field in required_fields:
            assert field in fieldnames

        for row in reader:
            assert int(row["triplet_reversal_count"]) >= 0
            assert row["collision_status"] in ["True", "False"]

    # Check summary CSV fields
    with open(summary_csv, "r") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        assert "collision_rate" in fieldnames
        assert "minimum_distance_mean" in fieldnames
        assert "triplet_reversal_count_mean" in fieldnames
        assert "triplet_reversal_count_5th_percentile" in fieldnames
        assert "triplet_reversal_count_95th_percentile" in fieldnames

        for row in reader:
            assert 0.0 <= float(row["collision_rate"]) <= 1.0

    # Verify all 5 plots exist
    assert os.path.exists(os.path.join(plots_dir, "sensor_noise_vs_minimum_distance.png"))
    assert os.path.exists(os.path.join(plots_dir, "sensor_noise_vs_collision_rate.png"))
    assert os.path.exists(os.path.join(plots_dir, "sensor_noise_vs_state_transitions.png"))
    assert os.path.exists(os.path.join(plots_dir, "sensor_noise_vs_triplet_reversal_count.png"))
    assert os.path.exists(os.path.join(plots_dir, "sensor_noise_vs_activation_time.png"))
