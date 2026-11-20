"""
Automated Unit Tests for Discrete-Time Longitudinal Vehicle Model.
"""

import pytest
from src.vehicle_model import VehicleModel
from config.parameters import DT


def test_constant_velocity():
    """Test state update when acceleration is zero (constant velocity)."""
    initial_ego_speed = 10.0  # m/s
    initial_distance = 50.0  # m
    target_speed = 0.0  # stationary target

    model = VehicleModel(
        initial_ego_speed=initial_ego_speed, initial_distance=initial_distance
    )

    # Step 1 with a = 0.0
    ego_speed, distance = model.step(
        acceleration=0.0, target_speed=target_speed, dt=DT
    )

    # Ego speed should remain 10.0 m/s
    assert ego_speed == pytest.approx(10.0)
    # Distance should decrease by 10.0 * 0.05 = 0.5 m => 49.5 m
    assert distance == pytest.approx(49.5)


def test_braking():
    """Test state update when deceleration is applied."""
    model = VehicleModel(initial_ego_speed=10.0, initial_distance=50.0)
    deceleration = -2.0  # m/s^2
    target_speed = 0.0

    # Step 1: dt = 0.05 s
    # delta_d = (0.0 - 10.0) * 0.05 = -0.5 m => d = 49.5 m
    # v_ego = 10.0 + (-2.0) * 0.05 = 9.9 m/s
    ego_speed, distance = model.step(
        acceleration=deceleration, target_speed=target_speed, dt=DT
    )

    assert ego_speed == pytest.approx(9.9)
    assert distance == pytest.approx(49.5)

    # Step 2:
    # delta_d = (0.0 - 9.9) * 0.05 = -0.495 m => d = 49.005 m
    # v_ego = 9.9 + (-2.0) * 0.05 = 9.8 m/s
    ego_speed, distance = model.step(
        acceleration=deceleration, target_speed=target_speed, dt=DT
    )

    assert ego_speed == pytest.approx(9.8)
    assert distance == pytest.approx(49.005)


def test_speed_approaching_zero():
    """Test that ego speed clamps to 0.0 and never becomes negative during heavy braking."""
    model = VehicleModel(initial_ego_speed=1.0, initial_distance=100.0)
    heavy_deceleration = -6.0  # m/s^2

    # Step 1: 1.0 + (-6.0)*0.05 = 0.7 m/s
    model.step(acceleration=heavy_deceleration, target_speed=0.0, dt=DT)
    assert model.ego_speed == pytest.approx(0.7)

    # Step 2: 0.7 + (-6.0)*0.05 = 0.4 m/s
    model.step(acceleration=heavy_deceleration, target_speed=0.0, dt=DT)
    assert model.ego_speed == pytest.approx(0.4)

    # Step 3: 0.4 + (-6.0)*0.05 = 0.1 m/s
    model.step(acceleration=heavy_deceleration, target_speed=0.0, dt=DT)
    assert model.ego_speed == pytest.approx(0.1)

    # Step 4: 0.1 + (-6.0)*0.05 = -0.2 => should clamp to 0.0
    model.step(acceleration=heavy_deceleration, target_speed=0.0, dt=DT)
    assert model.ego_speed == 0.0

    # Step 5: further step with zero speed should stay 0.0
    model.step(acceleration=heavy_deceleration, target_speed=0.0, dt=DT)
    assert model.ego_speed == 0.0


def test_distance_evolution_and_impact_clamping():
    """Test relative distance evolution and collision floor clamping at 0.0."""
    model = VehicleModel(initial_ego_speed=20.0, initial_distance=0.5)
    target_speed = 0.0

    # Step 1: delta_d = (0 - 20) * 0.05 = -1.0 m => distance becomes 0.5 - 1.0 = -0.5 -> clamped to 0.0
    ego_speed, distance = model.step(
        acceleration=0.0, target_speed=target_speed, dt=DT
    )

    assert distance == 0.0
    assert ego_speed == pytest.approx(20.0)


def test_invalid_initialization_inputs():
    """Test safety handling for negative initialization inputs."""
    with pytest.raises(ValueError, match="non-negative"):
        VehicleModel(initial_ego_speed=-5.0, initial_distance=10.0)

    with pytest.raises(ValueError, match="non-negative"):
        VehicleModel(initial_ego_speed=10.0, initial_distance=-2.0)
