"""
Automated Unit Tests for Brake Actuator Dynamics Model (ZOH Formulation).

Verifies:
- Parameter validation for tau (must be non-negative and finite)
- Zero command response from zero initial state
- Exact ZOH discrete update for tau = dt, tau > dt, and tau < dt
- Asymptotic convergence toward commanded deceleration over time
- Response speed comparison between smaller and larger tau values
- Ideal instantaneous response for tau = 0.0
"""

import pytest
import math
from src.brake_actuator import BrakeActuator


def test_actuator_parameter_validation():
    """Test exception raising for invalid tau or initial deceleration parameters."""
    with pytest.raises(ValueError, match="Actuator tau must be a non-negative finite number"):
        BrakeActuator(tau=-0.1)

    with pytest.raises(ValueError, match="Actuator tau must be a non-negative finite number"):
        BrakeActuator(tau=float("nan"))

    with pytest.raises(ValueError, match="Actuator tau must be a non-negative finite number"):
        BrakeActuator(tau=float("inf"))

    with pytest.raises(ValueError, match="Initial deceleration must be finite"):
        BrakeActuator(initial_deceleration=float("nan"))


def test_zero_command_zero_output():
    """Test that zero commanded deceleration produces zero output from zero state."""
    actuator = BrakeActuator(tau=0.15)
    output = actuator.step(a_command=0.0, dt=0.05)
    assert output == 0.0


def test_ideal_actuator_tau_zero():
    """Test that tau = 0 gives ideal instantaneous response (a_actual = a_command)."""
    actuator = BrakeActuator(tau=0.0)
    a_cmd = -4.0
    out = actuator.step(a_cmd, dt=0.05)
    assert out == a_cmd


def test_exact_zoh_response_tau_equals_dt():
    """Deterministic test for tau = dt (tau = 0.05s, dt = 0.05s)."""
    tau = 0.05
    dt = 0.05
    actuator = BrakeActuator(tau=tau)
    a_cmd = -4.0

    output = actuator.step(a_command=a_cmd, dt=dt)
    expected_alpha = 1.0 - math.exp(-1.0)
    expected_output = expected_alpha * a_cmd

    assert math.isclose(output, expected_output, abs_tol=1e-6)
    assert output > a_cmd  # Less negative than full command
    assert output < 0.0


def test_exact_zoh_response_tau_greater_than_dt():
    """Deterministic test for tau > dt (tau = 0.20s, dt = 0.05s)."""
    tau = 0.20
    dt = 0.05
    actuator = BrakeActuator(tau=tau)
    a_cmd = -4.0

    output = actuator.step(a_command=a_cmd, dt=dt)
    expected_alpha = 1.0 - math.exp(-dt / tau)
    expected_output = expected_alpha * a_cmd

    assert math.isclose(output, expected_output, abs_tol=1e-6)
    assert output > a_cmd


def test_exact_zoh_response_tau_less_than_dt():
    """Deterministic test for tau < dt (tau = 0.01s, dt = 0.05s)."""
    tau = 0.01
    dt = 0.05
    actuator = BrakeActuator(tau=tau)
    a_cmd = -4.0

    output = actuator.step(a_command=a_cmd, dt=dt)
    expected_alpha = 1.0 - math.exp(-dt / tau)
    expected_output = expected_alpha * a_cmd

    assert math.isclose(output, expected_output, abs_tol=1e-6)
    # Output should be almost equal to a_cmd (-3.973 vs -4.0)
    assert math.isclose(output, a_cmd, abs_tol=0.05)


def test_actual_deceleration_approaches_command_over_time():
    """Test that actual deceleration asymptotically approaches commanded deceleration."""
    tau = 0.15
    dt = 0.05
    actuator = BrakeActuator(tau=tau)
    a_cmd = -6.0

    outputs = []
    for _ in range(20):
        outputs.append(actuator.step(a_command=a_cmd, dt=dt))

    # Output should monotonically decrease toward -6.0 m/s^2
    for i in range(1, len(outputs)):
        assert outputs[i] < outputs[i - 1]

    # After 1.0 s (>> tau=0.15s), output should be very close to -6.0
    assert math.isclose(outputs[-1], a_cmd, abs_tol=0.05)


def test_smaller_tau_produces_faster_response():
    """Test that smaller tau produces faster deceleration response than larger tau."""
    dt = 0.05
    a_cmd = -4.0

    actuator_fast = BrakeActuator(tau=0.05)
    actuator_slow = BrakeActuator(tau=0.20)

    fast_out = actuator_fast.step(a_cmd, dt)
    slow_out = actuator_slow.step(a_cmd, dt)

    assert fast_out < slow_out  # More negative = larger braking deceleration
