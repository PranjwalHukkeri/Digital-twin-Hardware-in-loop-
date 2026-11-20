"""
Unit Tests for AEB Performance Evaluation Framework Module.

Verifies:
- Registry coverage of all 14 mandatory project evaluation metrics
- Comprehensive evaluation execution on synthetic and simulated results
- Integrity of classification categories and TBD status flags
"""

import pytest
from analysis.performance_evaluation import (
    METRIC_REGISTRY,
    evaluate_comprehensive_performance,
)
from src.simulation import SimulationEngine, SimulationSample, SimulationResult
from scenarios.scenarios import CCRScenario


def test_metric_registry_coverage():
    """Verify registry defines all 14 mandatory project metrics."""
    required_metrics = [
        "collision_status",
        "minimum_distance",
        "aeb_activation_time",
        "ttc_at_activation",
        "max_braking_deceleration",
        "max_actual_deceleration",
        "ninety_percent_braking_response_time",
        "stopping_time",
        "stopping_distance",
        "vehicle_stop_status",
        "final_relative_velocity",
        "min_relative_velocity",
        "state_transition_count",
        "triplet_reversal_count",
    ]

    for m in required_metrics:
        assert m in METRIC_REGISTRY
        spec = METRIC_REGISTRY[m]
        assert spec.category in [
            "Safety",
            "Timeliness",
            "Braking Effectiveness",
            "Controller Stability",
            "Robustness",
        ]
        assert len(spec.unit) > 0
        assert len(spec.definition) > 0
        assert len(spec.physical_meaning) > 0
        assert len(spec.threshold_source_needed) > 0


def test_evaluate_comprehensive_performance_execution():
    """Verify end-to-end execution of comprehensive performance evaluation."""
    engine = SimulationEngine(sensor_latency=0.0, actuator_tau=0.0, distance_noise_std=0.0)
    scenario = CCRScenario(name="CCR_Test", initial_ego_speed=15.0, initial_distance=30.0)
    result = engine.run(scenario=scenario)

    eval_report = evaluate_comprehensive_performance(result)
    assert eval_report["scenario_name"] == "CCR_Test"
    assert "metrics" in eval_report

    metrics_dict = eval_report["metrics"]
    for key in METRIC_REGISTRY:
        assert key in metrics_dict
        entry = metrics_dict[key]
        assert "value" in entry
        assert "unit" in entry
        assert "category" in entry
        assert "status" in entry

    # Verify specific known values for baseline CCR
    assert metrics_dict["collision_status"]["value"] is False
    assert metrics_dict["minimum_distance"]["value"] > 0.8
    assert metrics_dict["aeb_activation_time"]["value"] == 0.0
    assert metrics_dict["max_braking_deceleration"]["value"] == 6.0
    assert metrics_dict["state_transition_count"]["value"] in (14, 19)
    assert metrics_dict["triplet_reversal_count"]["value"] == 1
