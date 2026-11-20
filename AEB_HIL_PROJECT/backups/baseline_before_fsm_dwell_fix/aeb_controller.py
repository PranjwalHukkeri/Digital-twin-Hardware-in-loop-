"""
AEB Controller Module with Explicit Finite State Machine (FSM) Architecture.

Implements discrete-time AEB kinematic risk calculation (V_rel, TTC) and a 5-state
deterministic FSM decision layer (SAFE, WARNING, AEB_LEVEL_1, AEB_LEVEL_2, AEB_LEVEL_3).
"""

import math
from typing import Tuple
from config.parameters import (
    TTC_WARNING,
    TTC_LEVEL_1,
    TTC_LEVEL_2,
    TTC_LEVEL_3,
    A_LEVEL_1,
    A_LEVEL_2,
    A_LEVEL_3,
    MIN_BRAKE_DURATION,
)


class AEBController:
    """
    Finite State Machine (FSM) Autonomous Emergency Braking Controller.
    """

    def __init__(self, min_brake_duration: float = MIN_BRAKE_DURATION):
        """
        Initialize controller FSM state memory.

        Args:
            min_brake_duration (float): Minimum active braking dwell time in seconds.
                                       Engineering design parameter for anti-chattering.
        """
        self.min_brake_duration = float(min_brake_duration)
        self.current_state: str = "SAFE"
        self.state_time: float = 0.0

    def reset(self) -> None:
        """Reset FSM state memory to default initial SAFE state."""
        self.current_state = "SAFE"
        self.state_time = 0.0

    @staticmethod
    def calculate_relative_velocity(ego_speed: float, target_speed: float) -> float:
        """
        Calculate relative velocity: V_rel = V_ego - V_target.

        Args:
            ego_speed (float): Speed of Ego vehicle (m/s).
            target_speed (float): Speed of Target vehicle (m/s).

        Returns:
            float: Relative velocity (m/s).

        Raises:
            ValueError: If inputs are non-finite.
        """
        if not math.isfinite(ego_speed) or not math.isfinite(target_speed):
            raise ValueError("Inputs must be finite numbers.")
        return ego_speed - target_speed

    @staticmethod
    def calculate_ttc(ego_speed: float, target_speed: float, distance: float) -> float:
        """
        Calculate Time-To-Collision (TTC) = distance / V_rel.

        Args:
            ego_speed (float): Speed of Ego vehicle (m/s).
            target_speed (float): Speed of Target vehicle (m/s).
            distance (float): Separation distance to target vehicle (m).

        Returns:
            float: Time-To-Collision in seconds. Returns float('inf') if V_rel <= 0.

        Raises:
            ValueError: If inputs are non-finite or distance is negative.
        """
        if (
            not math.isfinite(ego_speed)
            or not math.isfinite(target_speed)
            or not math.isfinite(distance)
        ):
            raise ValueError("Inputs must be finite numbers.")

        if distance < 0.0:
            raise ValueError("Distance cannot be negative.")

        v_rel = ego_speed - target_speed

        if v_rel <= 0.0:
            return float("inf")

        return distance / v_rel

    def update_state(
        self, ttc: float, v_rel: float, dt: float = 0.05
    ) -> Tuple[str, float]:
        """
        Advance controller FSM state given current TTC, relative velocity, and timestep dt.
        Follows strict 5-priority transition precedence:

        Priority 1: Safety-Critical Release / Threat Clearance (V_rel <= 0 or TTC > TTC_WARNING)
        Priority 2: Threat Escalation (TTC <= TTC_next_higher_level)
        Priority 3: Minimum Dwell Retention (state_time < min_brake_duration)
        Priority 4: Threat De-escalation (state_time >= min_brake_duration and TTC > current_level_entry)
        Priority 5: Normal Release (state_time >= min_brake_duration and TTC > TTC_WARNING)

        Args:
            ttc (float): Current Time-To-Collision (s).
            v_rel (float): Current relative velocity (m/s).
            dt (float): Timestep duration (s).

        Returns:
            Tuple[str, float]: Updated (current_state, desired_deceleration)
        """
        if not math.isfinite(ttc) or not math.isfinite(v_rel):
            self.current_state = "SAFE"
            self.state_time = 0.0
            return self.current_state, 0.0

        # Increment dwell time in active state
        self.state_time += dt

        # ----------------------------------------------------------------------
        # Priority 1: Safety-Critical Release / Threat Clearance
        # If target moving away or equal speed (v_rel <= 0) or distance fully safe (TTC > TTC_WARNING)
        # ----------------------------------------------------------------------
        if v_rel <= 0.0 or ttc > TTC_WARNING:
            if self.current_state != "SAFE":
                self.current_state = "SAFE"
                self.state_time = 0.0
            return self.current_state, 0.0

        # State decision logic based on current state memory
        state = self.current_state

        # STATE 1: SAFE
        if state == "SAFE":
            if ttc <= TTC_LEVEL_3:
                self.current_state = "AEB_LEVEL_3"
                self.state_time = 0.0
            elif ttc <= TTC_LEVEL_2:
                self.current_state = "AEB_LEVEL_2"
                self.state_time = 0.0
            elif ttc <= TTC_LEVEL_1:
                self.current_state = "AEB_LEVEL_1"
                self.state_time = 0.0
            elif ttc <= TTC_WARNING:
                self.current_state = "WARNING"
                self.state_time = 0.0

        # STATE 2: WARNING
        elif state == "WARNING":
            # Priority 2: Escalation
            if ttc <= TTC_LEVEL_3:
                self.current_state = "AEB_LEVEL_3"
                self.state_time = 0.0
            elif ttc <= TTC_LEVEL_2:
                self.current_state = "AEB_LEVEL_2"
                self.state_time = 0.0
            elif ttc <= TTC_LEVEL_1:
                self.current_state = "AEB_LEVEL_1"
                self.state_time = 0.0
            # Priority 5: Normal Release
            elif ttc > TTC_WARNING:
                self.current_state = "SAFE"
                self.state_time = 0.0

        # STATE 3: AEB_LEVEL_1
        elif state == "AEB_LEVEL_1":
            # Priority 2: Escalation
            if ttc <= TTC_LEVEL_3:
                self.current_state = "AEB_LEVEL_3"
                self.state_time = 0.0
            elif ttc <= TTC_LEVEL_2:
                self.current_state = "AEB_LEVEL_2"
                self.state_time = 0.0
            # Priority 3: Minimum Dwell Retention
            elif self.state_time < self.min_brake_duration:
                pass  # Retain AEB_LEVEL_1
            # Priority 4: De-escalation / Normal Release
            elif ttc > TTC_LEVEL_1:
                if ttc <= TTC_WARNING:
                    self.current_state = "WARNING"
                else:
                    self.current_state = "SAFE"
                self.state_time = 0.0

        # STATE 4: AEB_LEVEL_2
        elif state == "AEB_LEVEL_2":
            # Priority 2: Escalation
            if ttc <= TTC_LEVEL_3:
                self.current_state = "AEB_LEVEL_3"
                self.state_time = 0.0
            # Priority 3: Minimum Dwell Retention
            elif self.state_time < self.min_brake_duration:
                pass  # Retain AEB_LEVEL_2
            # Priority 4: De-escalation
            elif ttc > TTC_LEVEL_2:
                if ttc <= TTC_LEVEL_1:
                    self.current_state = "AEB_LEVEL_1"
                elif ttc <= TTC_WARNING:
                    self.current_state = "WARNING"
                else:
                    self.current_state = "SAFE"
                self.state_time = 0.0

        # STATE 5: AEB_LEVEL_3
        elif state == "AEB_LEVEL_3":
            # Priority 3: Minimum Dwell Retention
            if self.state_time < self.min_brake_duration:
                pass  # Retain AEB_LEVEL_3
            # Priority 4: De-escalation
            elif ttc > TTC_LEVEL_3:
                if ttc <= TTC_LEVEL_2:
                    self.current_state = "AEB_LEVEL_2"
                elif ttc <= TTC_LEVEL_1:
                    self.current_state = "AEB_LEVEL_1"
                elif ttc <= TTC_WARNING:
                    self.current_state = "WARNING"
                else:
                    self.current_state = "SAFE"
                self.state_time = 0.0

        # Map current state to desired deceleration
        decel_map = {
            "SAFE": 0.0,
            "WARNING": 0.0,
            "AEB_LEVEL_1": A_LEVEL_1,
            "AEB_LEVEL_2": A_LEVEL_2,
            "AEB_LEVEL_3": A_LEVEL_3,
        }
        return self.current_state, decel_map[self.current_state]

    def evaluate_aeb_state(self, ttc: float) -> Tuple[str, float]:
        """
        Stateless static mapping helper for baseline boundary checks.
        Maps instantaneous TTC directly to AEB state and deceleration demand.
        """
        if not math.isfinite(ttc) or ttc < 0.0:
            return "SAFE", 0.0
        if ttc > TTC_WARNING:
            return "SAFE", 0.0
        if ttc > TTC_LEVEL_1:
            return "WARNING", 0.0
        if ttc > TTC_LEVEL_2:
            return "AEB_LEVEL_1", A_LEVEL_1
        if ttc > TTC_LEVEL_3:
            return "AEB_LEVEL_2", A_LEVEL_2
        return "AEB_LEVEL_3", A_LEVEL_3
