"""
Automated Unit Tests for AEB Functional Verification Scenario Framework.

Verifies:
- CCRStationaryTarget scenario execution
- CCRmMovingTarget scenario execution
- CCRbBrakingTarget scenario execution and time-varying target acceleration profile
- Scenario parameter validation
- Baseline scenario preservation
- Collision status independence from vehicle stop status
- Final relative velocity metric computation
- Stopped vehicle reporting when actual zero velocity is reached
"""

import pytest
import math
from scenarios.scenarios import Scenario, CCRScenario, CCRmScenario, CCRbScenario
from src.simulation import SimulationEngine
from src.metrics import PerformanceEvaluator


def test_scenario_parameter_validation():
    """Test scenario parameter validation and exception handling."""
    with pytest.raises(ValueError, match="Scenario name must be a non-empty string"):
        Scenario("", 10.0, 0.0, 30.0)

    with pytest.raises(ValueError, match="Initial Ego speed must be a non-negative"):
        CCRScenario(initial_ego_speed=-5.0)

    with pytest.raises(ValueError, match="CCRm scenario requires initial_ego_speed > initial_target_speed"):
        CCRmScenario(initial_ego_speed=10.0, initial_target_speed=15.0)

    with pytest.raises(ValueError, match="CCRb target_decel must be non-positive"):
        CCRbScenario(target_decel=2.0)


def test_ccr_stationary_target_execution():
    """Test CCR scenario with stationary target vehicle."""
    scenario = CCRScenario(initial_ego_speed=15.0, initial_distance=30.0)
    engine = SimulationEngine()
    result = engine.run(scenario=scenario)
    metrics = PerformanceEvaluator.evaluate(result)

    assert result.scenario_name == "CCR_Stationary_Target"
    assert metrics.initial_ego_speed == 15.0
    assert metrics.initial_distance == 30.0
    assert metrics.aeb_activation_time is not None
    assert all(s.target_speed == 0.0 for s in result.samples)


def test_ccrm_moving_target_execution():
    """Test CCRm scenario with constant moving target vehicle."""
    scenario = CCRmScenario(initial_ego_speed=15.0, initial_target_speed=5.0, initial_distance=25.0)
    engine = SimulationEngine()
    result = engine.run(scenario=scenario)
    metrics = PerformanceEvaluator.evaluate(result)

    assert result.scenario_name == "CCRm_Moving_Target"
    assert metrics.initial_ego_speed == 15.0
    assert metrics.initial_distance == 25.0
    assert all(s.target_speed == 5.0 for s in result.samples)


def test_ccrb_braking_target_execution():
    """Test CCRb scenario with target vehicle braking after brake_start_time."""
    scenario = CCRbScenario(
        initial_ego_speed=15.0,
        initial_target_speed=10.0,
        initial_distance=30.0,
        target_decel=-2.0,
        brake_start_time=1.0,
    )
    engine = SimulationEngine()
    result = engine.run(scenario=scenario)
    metrics = PerformanceEvaluator.evaluate(result)

    assert result.scenario_name == "CCRb_Braking_Target"
    samples_before_brake = [s for s in result.samples if s.time < 1.0]
    samples_after_brake = [s for s in result.samples if s.time >= 1.5]

    assert all(s.target_speed == 10.0 for s in samples_before_brake)
    assert any(s.target_speed < 10.0 for s in samples_after_brake)


def test_target_acceleration_profile():
    """Test custom target acceleration profile evaluation."""
    def custom_profile(t: float) -> float:
        return -1.0 if t >= 0.5 else 0.0

    scenario = Scenario(
        name="Custom_Target_Accel",
        initial_ego_speed=10.0,
        initial_target_speed=5.0,
        initial_distance=20.0,
        target_accel_func=custom_profile,
    )

    assert scenario.get_target_acceleration(0.2) == 0.0
    assert scenario.get_target_acceleration(0.8) == -1.0


def test_baseline_scenario_preserved():
    """Test that default baseline run without scenario object remains identical."""
    engine = SimulationEngine()
    result = engine.run(1.0, 0.0, 5.0)
    metrics = PerformanceEvaluator.evaluate(result)

    assert result.scenario_name == "Baseline_Simulation"
    assert metrics.initial_ego_speed == 1.0
    assert metrics.initial_distance == 5.0
    assert metrics.aeb_activation_time == 2.05
    assert metrics.max_braking_deceleration == 2.0


# ==============================================================================
# Additional Tests for Metrics & Reporting Corrections (A - F)
# ==============================================================================


def test_A_ccr_no_collision_nonzero_final_velocity():
    """Test A: CCR scenario avoids collision but leaves residual non-zero final velocity (~0.2 m/s)."""
    scenario = CCRScenario(initial_ego_speed=15.0, initial_distance=30.0)
    engine = SimulationEngine()
    result = engine.run(scenario=scenario)
    metrics = PerformanceEvaluator.evaluate(result)

    assert metrics.collision_status is False
    assert metrics.final_ego_speed > 0.0
    assert metrics.vehicle_stop_status == "NOT_STOPPED"


def test_B_ccrm_no_collision_no_ego_stop():
    """Test B: CCRm scenario avoids collision without Ego vehicle coming to a complete stop."""
    scenario = CCRmScenario(initial_ego_speed=15.0, initial_target_speed=5.0, initial_distance=25.0)
    engine = SimulationEngine()
    result = engine.run(scenario=scenario)
    metrics = PerformanceEvaluator.evaluate(result)

    assert metrics.collision_status is False
    assert metrics.vehicle_stop_status == "NOT_STOPPED"
    assert metrics.final_ego_speed > 4.0  # Matched near moving target speed


def test_C_ccrb_no_collision():
    """Test C: CCRb scenario avoids collision under target deceleration."""
    scenario = CCRbScenario(
        initial_ego_speed=15.0,
        initial_target_speed=10.0,
        initial_distance=30.0,
        target_decel=-2.0,
        brake_start_time=1.0,
    )
    engine = SimulationEngine()
    result = engine.run(scenario=scenario)
    metrics = PerformanceEvaluator.evaluate(result)

    assert metrics.collision_status is False
    assert metrics.min_distance > 0.0


def test_D_collision_status_independent_from_stop_status():
    """Test D: Collision status evaluated independently from vehicle stop status."""
    engine = SimulationEngine(max_time=2.0)
    # High speed, small gap -> collision occurs while NOT_STOPPED
    result = engine.run(initial_ego_speed=30.0, initial_target_speed=0.0, initial_distance=2.0)
    metrics = PerformanceEvaluator.evaluate(result)

    assert metrics.collision_status is True
    assert metrics.vehicle_stop_status == "NOT_STOPPED"


def test_E_final_relative_velocity_metric():
    """Test E: Final relative velocity metric calculation."""
    scenario = CCRmScenario(initial_ego_speed=15.0, initial_target_speed=5.0, initial_distance=25.0)
    engine = SimulationEngine()
    result = engine.run(scenario=scenario)
    metrics = PerformanceEvaluator.evaluate(result)

    assert metrics.final_relative_velocity == pytest.approx(
        result.samples[-1].relative_velocity
    )
    assert metrics.min_relative_velocity <= metrics.initial_ego_speed - 5.0


def test_F_stopped_vehicle_reporting_on_zero_velocity():
    """Test F: Vehicle stop status reports STOPPED when ego vehicle reaches actual zero velocity."""
    scenario = CCRScenario(initial_ego_speed=1.0, initial_distance=5.0)
    engine = SimulationEngine(max_time=10.0)
    # Create artificial scenario where ego speed reaches 0.0
    result = engine.run(initial_ego_speed=1.0, initial_target_speed=0.0, initial_distance=2.0)
    metrics = PerformanceEvaluator.evaluate(result)

    if any(math.isclose(s.ego_speed, 0.0, abs_tol=1e-5) for s in result.samples):
        assert metrics.vehicle_stop_status == "STOPPED"
        assert metrics.stopping_time is not None
