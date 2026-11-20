"""
Brake Actuator Dynamics Model Module

Implements a first-order lag model for brake actuator response dynamics using the
exact Zero-Order Hold (ZOH) discrete-time equivalent:
    tau * (da_actual / dt) + a_actual = a_command

Exact ZOH discrete formulation:
    alpha = 1.0 - math.exp(-dt / tau)
    a_actual[k+1] = a_actual[k] + alpha * (a_command[k] - a_actual[k])

Where:
    a_command: Desired deceleration commanded by the AEB controller (m/s^2)
    a_actual:  Actual deceleration output delivered to the vehicle chassis (m/s^2)
    tau:       Actuator time constant (s). tau = 0.0 represents an ideal instantaneous actuator.
"""

import math


class BrakeActuator:
    """
    First-order discrete-time brake actuator dynamics model using exact ZOH discretization.
    """

    def __init__(self, tau: float = 0.0, initial_deceleration: float = 0.0):
        """
        Initialize the brake actuator model.

        Args:
            tau (float): Actuator time constant in seconds. tau >= 0.0.
                         tau = 0.0 represents an ideal instantaneous actuator.
            initial_deceleration (float): Initial actual deceleration state (m/s^2).

        Raises:
            ValueError: If tau is negative or non-finite.
        """
        if not math.isfinite(tau) or tau < 0.0:
            raise ValueError(
                f"Actuator tau must be a non-negative finite number. Got tau={tau}"
            )
        if not math.isfinite(initial_deceleration):
            raise ValueError(
                f"Initial deceleration must be finite. Got {initial_deceleration}"
            )

        self.tau = float(tau)
        self.initial_deceleration = float(initial_deceleration)
        self.a_actual = float(initial_deceleration)

    def reset(self) -> None:
        """Reset the internal actual deceleration state to initial condition."""
        self.a_actual = float(self.initial_deceleration)

    def step(self, a_command: float, dt: float) -> float:
        """
        Advance the actuator state by one simulation timestep dt using exact ZOH discrete update.

        Args:
            a_command (float): Commanded deceleration from controller (m/s^2).
            dt (float): Discrete timestep duration in seconds.

        Returns:
            float: Updated actual deceleration output (m/s^2).

        Raises:
            ValueError: If dt <= 0 or non-finite, or a_command is non-finite.
        """
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"Timestep dt must be positive and finite. Got dt={dt}")
        if not math.isfinite(a_command):
            raise ValueError(
                f"Commanded deceleration must be finite. Got a_command={a_command}"
            )

        # Ideal instantaneous actuator case (tau = 0)
        if math.isclose(self.tau, 0.0, abs_tol=1e-9):
            self.a_actual = float(a_command)
            return self.a_actual

        # Exact zero-order hold (ZOH) discrete update factor
        alpha = 1.0 - math.exp(-dt / self.tau)
        self.a_actual += alpha * (a_command - self.a_actual)
        return self.a_actual
