"""
Automated Unit Tests for AEB Controller Kinematics, TTC, and Decision Layer.
"""

import math
import pytest
from src.aeb_controller import AEBController
from config.parameters import A_LEVEL_1, A_LEVEL_2, A_LEVEL_3


@pytest.fixture
def controller():
    """Fixture providing an instance of AEBController."""
    return AEBController()


# ==============================================================================
# Kinematic & TTC Calculation Tests
# ==============================================================================


def test_approaching_target(controller):
    """Test TTC calculation when Ego is faster than Target (approaching)."""
    ego_speed = 20.0  # m/s (~72 km/h)
    target_speed = 10.0  # m/s (~36 km/h)
    distance = 50.0  # meters

    v_rel = controller.calculate_relative_velocity(ego_speed, target_speed)
    ttc = controller.calculate_ttc(ego_speed, target_speed, distance)

    assert v_rel == pytest.approx(10.0)
    assert ttc == pytest.approx(5.0)


def test_equal_speeds(controller):
    """Test TTC calculation when Ego and Target move at equal speeds (V_rel = 0)."""
    ego_speed = 15.0
    target_speed = 15.0
    distance = 30.0

    v_rel = controller.calculate_relative_velocity(ego_speed, target_speed)
    ttc = controller.calculate_ttc(ego_speed, target_speed, distance)

    assert v_rel == pytest.approx(0.0)
    assert math.isinf(ttc) and ttc > 0


def test_ego_slower_than_target(controller):
    """Test TTC calculation when Ego is slower than Target (moving away)."""
    ego_speed = 10.0
    target_speed = 15.0
    distance = 40.0

    v_rel = controller.calculate_relative_velocity(ego_speed, target_speed)
    ttc = controller.calculate_ttc(ego_speed, target_speed, distance)

    assert v_rel == pytest.approx(-5.0)
    assert math.isinf(ttc) and ttc > 0


def test_zero_relative_velocity(controller):
    """Test TTC when relative velocity is zero at stationary or equal speeds."""
    ego_speed = 0.0
    target_speed = 0.0
    distance = 10.0

    v_rel = controller.calculate_relative_velocity(ego_speed, target_speed)
    ttc = controller.calculate_ttc(ego_speed, target_speed, distance)

    assert v_rel == pytest.approx(0.0)
    assert math.isinf(ttc) and ttc > 0


def test_invalid_distance(controller):
    """Test safety handling for negative distance."""
    ego_speed = 10.0
    target_speed = 0.0
    distance = -5.0

    with pytest.raises(ValueError, match="Distance cannot be negative"):
        controller.calculate_ttc(ego_speed, target_speed, distance)


def test_non_finite_inputs(controller):
    """Test safety handling for NaN or Infinite inputs."""
    with pytest.raises(ValueError, match="must be finite"):
        controller.calculate_ttc(float("nan"), 10.0, 50.0)

    with pytest.raises(ValueError, match="must be finite"):
        controller.calculate_ttc(20.0, 10.0, float("inf"))


# ==============================================================================
# Decision Layer & Boundary Value Unit Tests
# ==============================================================================


@pytest.mark.parametrize(
    "ttc, expected_state, expected_decel",
    [
        # Infinite TTC
        (float("inf"), "SAFE", 0.0),
        # Above TTC_WARNING (4.00 s)
        (4.05, "SAFE", 0.0),
        # Boundary TTC_WARNING (4.00 s)
        (4.00, "WARNING", 0.0),
        # Slightly below 4.00 s (e.g. 3.95 s)
        (3.95, "WARNING", 0.0),
        # Slightly above 3.00 s (e.g. 3.05 s)
        (3.05, "WARNING", 0.0),
        # Boundary TTC_LEVEL_1 (3.00 s)
        (3.00, "AEB_LEVEL_1", A_LEVEL_1),
        # Slightly below 3.00 s (e.g. 2.95 s)
        (2.95, "AEB_LEVEL_1", A_LEVEL_1),
        # Slightly above 2.25 s (e.g. 2.30 s)
        (2.30, "AEB_LEVEL_1", A_LEVEL_1),
        # Boundary TTC_LEVEL_2 (2.25 s)
        (2.25, "AEB_LEVEL_2", A_LEVEL_2),
        # Slightly below 2.25 s (e.g. 2.20 s)
        (2.20, "AEB_LEVEL_2", A_LEVEL_2),
        # Slightly above 1.75 s (e.g. 1.80 s)
        (1.80, "AEB_LEVEL_2", A_LEVEL_2),
        # Boundary TTC_LEVEL_3 (1.75 s)
        (1.75, "AEB_LEVEL_3", A_LEVEL_3),
        # Slightly below 1.75 s (e.g. 1.70 s)
        (1.70, "AEB_LEVEL_3", A_LEVEL_3),
        # Very small / zero TTC
        (0.50, "AEB_LEVEL_3", A_LEVEL_3),
        (0.00, "AEB_LEVEL_3", A_LEVEL_3),
    ],
)
def test_aeb_decision_boundaries(controller, ttc, expected_state, expected_decel):
    """Test decision logic states and decelerations across all boundary values."""
    state, decel = controller.evaluate_aeb_state(ttc)
    assert state == expected_state
    assert decel == pytest.approx(expected_decel)
