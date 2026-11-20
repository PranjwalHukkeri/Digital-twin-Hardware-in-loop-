"""
AEB Functional Verification Scenario Definitions

Defines modular test scenario classes for AEB functional verification:
- Scenario: Base scenario abstraction
- CCRScenario: Car-to-Car Rear Stationary target
- CCRmScenario: Car-to-Car Rear Moving target
- CCRbScenario: Car-to-Car Rear Braking target with time-varying target acceleration profile

Parameter Classification Note:
- Initial Literature-based: TTC thresholds (4.00, 3.00, 2.25, 1.75s) and decel (-2.0, -4.0, -6.0 m/s^2)
- Software Assumptions: Timestep DT (0.05s)
- Engineering Design Parameters: MIN_BRAKE_DURATION (0.20s), generic scenario speed/distance inputs
- TBD Parameters: Vehicle mass, tire friction, actuator lag, sensor noise, formal Euro NCAP pass/fail limits
"""

import math
from typing import Callable, Optional


class Scenario:
    """
    Base Scenario Abstraction for AEB Simulation Verification.
    """

    def __init__(
        self,
        name: str,
        initial_ego_speed: float,
        initial_target_speed: float,
        initial_distance: float,
        simulation_duration: float = 10.0,
        target_accel_func: Optional[Callable[[float], float]] = None,
    ):
        """
        Initialize base scenario parameters.

        Args:
            name (str): Scenario identifier name.
            initial_ego_speed (float): Initial Ego speed (m/s).
            initial_target_speed (float): Initial Target speed (m/s).
            initial_distance (float): Initial separation distance (m).
            simulation_duration (float): Maximum simulation run duration (s).
            target_accel_func (Callable[[float], float], optional): Target acceleration profile a_target(t).

        Raises:
            ValueError: If inputs are invalid or non-finite.
        """
        if not name or not isinstance(name, str):
            raise ValueError("Scenario name must be a non-empty string.")

        if not math.isfinite(initial_ego_speed) or initial_ego_speed < 0.0:
            raise ValueError("Initial Ego speed must be a non-negative finite number.")

        if not math.isfinite(initial_target_speed) or initial_target_speed < 0.0:
            raise ValueError("Initial Target speed must be a non-negative finite number.")

        if not math.isfinite(initial_distance) or initial_distance < 0.0:
            raise ValueError("Initial distance must be a non-negative finite number.")

        if not math.isfinite(simulation_duration) or simulation_duration <= 0.0:
            raise ValueError("Simulation duration must be a positive finite number.")

        self.name = name
        self.initial_ego_speed = float(initial_ego_speed)
        self.initial_target_speed = float(initial_target_speed)
        self.initial_distance = float(initial_distance)
        self.simulation_duration = float(simulation_duration)
        self.target_accel_func = (
            target_accel_func if target_accel_func is not None else (lambda t: 0.0)
        )

    def get_target_acceleration(self, time: float) -> float:
        """
        Get target vehicle acceleration at simulation time t.
        """
        if not math.isfinite(time) or time < 0.0:
            return 0.0
        return float(self.target_accel_func(time))


class CCRScenario(Scenario):
    """
    Car-to-Car Rear Stationary (CCR) Scenario: Target vehicle is stationary (V_target = 0.0 m/s).
    """

    def __init__(
        self,
        name: str = "CCR_Stationary_Target",
        initial_ego_speed: float = 10.0,
        initial_distance: float = 30.0,
        simulation_duration: float = 10.0,
    ):
        super().__init__(
            name=name,
            initial_ego_speed=initial_ego_speed,
            initial_target_speed=0.0,
            initial_distance=initial_distance,
            simulation_duration=simulation_duration,
            target_accel_func=lambda t: 0.0,
        )


class CCRmScenario(Scenario):
    """
    Car-to-Car Rear Moving (CCRm) Scenario: Target vehicle moves at constant positive speed.
    """

    def __init__(
        self,
        name: str = "CCRm_Moving_Target",
        initial_ego_speed: float = 15.0,
        initial_target_speed: float = 5.0,
        initial_distance: float = 30.0,
        simulation_duration: float = 10.0,
    ):
        if initial_ego_speed <= initial_target_speed:
            raise ValueError(
                "CCRm scenario requires initial_ego_speed > initial_target_speed."
            )

        super().__init__(
            name=name,
            initial_ego_speed=initial_ego_speed,
            initial_target_speed=initial_target_speed,
            initial_distance=initial_distance,
            simulation_duration=simulation_duration,
            target_accel_func=lambda t: 0.0,
        )


class CCRbScenario(Scenario):
    """
    Car-to-Car Rear Braking (CCRb) Scenario: Target vehicle moves initially and decelerates according to a profile.
    """

    def __init__(
        self,
        name: str = "CCRb_Braking_Target",
        initial_ego_speed: float = 15.0,
        initial_target_speed: float = 10.0,
        initial_distance: float = 30.0,
        target_decel: float = -2.0,
        brake_start_time: float = 1.0,
        simulation_duration: float = 10.0,
    ):
        if target_decel > 0.0:
            raise ValueError("CCRb target_decel must be non-positive (deceleration).")

        if brake_start_time < 0.0 or not math.isfinite(brake_start_time):
            raise ValueError("Brake start time must be non-negative finite number.")

        def braking_profile(t: float) -> float:
            if t >= brake_start_time:
                return float(target_decel)
            return 0.0

        super().__init__(
            name=name,
            initial_ego_speed=initial_ego_speed,
            initial_target_speed=initial_target_speed,
            initial_distance=initial_distance,
            simulation_duration=simulation_duration,
            target_accel_func=braking_profile,
        )
        self.target_decel = float(target_decel)
        self.brake_start_time = float(brake_start_time)
