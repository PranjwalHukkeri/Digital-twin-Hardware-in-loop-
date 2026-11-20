"""
Automated Unit Tests for AEB Performance Evaluator Module.
"""

import pytest
from src.simulation import SimulationEngine
from src.metrics import PerformanceEvaluator, AEBPerformanceMetrics


def test_metrics_baseline_scenario():
    """Verify evaluation metrics calculation on baseline simulation run."""
    engine = SimulationEngine()
    result = engine.run(
        initial_ego_speed=1.0, initial_target_speed=0.0, initial_distance=5.0
    )

    metrics = PerformanceEvaluator.evaluate(result)

    assert isinstance(metrics, AEBPerformanceMetrics)
    assert metrics.initial_ego_speed == 1.0
    assert metrics.initial_distance == 5.0
    assert metrics.min_distance > 0.0
    assert metrics.aeb_activation_time is not None
    assert metrics.ttc_at_activation <= 3.00
    assert metrics.max_braking_deceleration == 2.0
    assert metrics.collision_status is False


def test_metrics_higher_speed_scenario():
    """Verify metrics calculation when vehicle comes to complete stop from higher speed."""
    engine = SimulationEngine(max_time=15.0)
    result = engine.run(
        initial_ego_speed=15.0, initial_target_speed=0.0, initial_distance=30.0
    )

    metrics = PerformanceEvaluator.evaluate(result)

    assert metrics.initial_ego_speed == 15.0
    assert metrics.initial_distance == 30.0
    assert metrics.stopping_time is not None
    assert metrics.stopping_distance is not None
    assert metrics.stopping_distance > 0.0
    assert metrics.collision_status is False


def test_metrics_no_aeb_trigger():
    """Verify metrics when separation distance is large and AEB never triggers."""
    engine = SimulationEngine(max_time=2.0)
    result = engine.run(
        initial_ego_speed=1.0, initial_target_speed=0.0, initial_distance=100.0
    )

    metrics = PerformanceEvaluator.evaluate(result)

    assert metrics.aeb_activation_time is None
    assert metrics.ttc_at_activation is None
    assert metrics.max_braking_deceleration == 0.0
    assert metrics.collision_status is False


def test_robust_collision_detection_high_speed():
    """Verify robust collision detection when high speed approach causes raw_distance <= 0."""
    engine = SimulationEngine(max_time=2.0)
    # High speed (30 m/s), small gap (2 m), no time to brake before impact
    result = engine.run(
        initial_ego_speed=30.0, initial_target_speed=0.0, initial_distance=2.0
    )

    metrics = PerformanceEvaluator.evaluate(result)

    assert metrics.collision_status is True
    # Verify raw_distance history captured negative values
    assert any(s.raw_distance <= 0.0 for s in result.samples)
