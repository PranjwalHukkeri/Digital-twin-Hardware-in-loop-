"""
Discrete-Time Longitudinal Vehicle Kinematics Model

Phase 1: Basic Point-Mass / Kinematic Vehicle Dynamic Model.

Updates Ego vehicle speed and relative distance to Target vehicle:
    v_ego[k+1] = max(0.0, v_ego[k] + acceleration * DT)
    d[k+1]     = max(0.0, d[k] + (target_speed - ego_speed) * DT)

Architecture Note:
This model represents the initial virtual plant (Digital Twin). It is completely
decoupled from tire dynamics, mass parameters, and hardware interfaces (dSPACE).
A higher-fidelity vehicle dynamics plant can replace this class in later phases.
"""

import math
from config.parameters import DT


class VehicleModel:
    """
    Discrete-time longitudinal vehicle kinematics state model.
    """

    def __init__(self, initial_ego_speed: float, initial_distance: float):
        """
        Initialize vehicle kinematics state.

        Args:
            initial_ego_speed: Initial Ego vehicle speed (m/s).
            initial_distance: Initial separation distance to target (m).

        Raises:
            ValueError: If inputs are negative or non-finite.
        """
        if not math.isfinite(initial_ego_speed) or initial_ego_speed < 0.0:
            raise ValueError("Ego speed must be a non-negative finite number.")

        if not math.isfinite(initial_distance) or initial_distance < 0.0:
            raise ValueError("Distance must be a non-negative finite number.")

        self.ego_speed = float(initial_ego_speed)
        self.distance = float(initial_distance)

    def step(
        self, acceleration: float, target_speed: float, dt: float = DT
    ) -> tuple[float, float]:
        """
        Advance vehicle kinematics state by one discrete timestep dt.

        Updates:
            d[k+1] = max(0.0, d[k] + (target_speed - ego_speed) * dt)
            v[k+1] = max(0.0, v[k] + acceleration * dt)

        Args:
            acceleration: Ego vehicle acceleration/deceleration (m/s^2).
            target_speed: Target vehicle speed (m/s).
            dt: Timestep duration in seconds (defaults to config DT).

        Returns:
            tuple[float, float]: Updated (ego_speed, distance).

        Raises:
            ValueError: If inputs are invalid or non-finite.
        """
        if (
            not math.isfinite(acceleration)
            or not math.isfinite(target_speed)
            or not math.isfinite(dt)
        ):
            raise ValueError("Inputs must be finite numbers.")

        if target_speed < 0.0:
            raise ValueError("Target speed cannot be negative.")

        if dt <= 0.0:
            raise ValueError("Time step dt must be positive.")

        # Distance evolution based on relative speed
        delta_d = (target_speed - self.ego_speed) * dt
        self.distance = max(0.0, self.distance + delta_d)

        # Ego speed evolution (clamped to >= 0.0 with floating-point epsilon threshold)
        next_speed = self.ego_speed + acceleration * dt
        if next_speed < 1e-6:
            self.ego_speed = 0.0
        else:
            self.ego_speed = float(next_speed)

        return self.ego_speed, self.distance
