"""
AEB Performance Evaluation & Metrics Computation Module

Calculates key performance indicators (KPIs) for Autonomous Emergency Braking (AEB)
simulation runs based on Euro NCAP and literature standards.

Robust Collision Criterion Definition:
A collision is declared (collision_status = True) if and only if the unclamped/raw
separation distance (raw_distance) becomes <= 0.0 meters while the Ego vehicle is
still actively closing distance toward the target (relative_velocity > 1e-4 m/s).
Collision status evaluates collision risk independently from vehicle stop status.

Stopping Distance Numerical Method:
Ego vehicle stopping distance is computed using left-hand rectangular Euler integration
of Ego vehicle ground speed over time from simulation start until complete stop:
    d_stop = sum(v_ego[k] * dt[k])  for k = 0 to N_stop - 1
Discretization error is bounded by O(dt). This directly measures Ego ground displacement
independent of target vehicle movement using actual sample timestamps without hardcoded fallbacks.

90-Percent Braking Response Time Definition & Limitations:
The time interval from the initial active braking command (first desired_deceleration < 0.0)
until the magnitude of actual_deceleration reaches 90% of the initial commanded step deceleration magnitude:
    abs(actual_deceleration) >= 0.90 * abs(initial_commanded_deceleration)
Limitation:
Evaluates response specifically to the initial step command. If commanded braking escalates
(e.g., LEVEL_1 -> LEVEL_2) prior to reaching 90% of the initial step command, this metric
tracks time to reach 90% of the initial step magnitude.
"""

from dataclasses import dataclass
from typing import Optional
import math
from src.simulation import SimulationResult


@dataclass
class AEBPerformanceMetrics:
    """Dataclass encapsulating baseline AEB evaluation KPIs."""

    scenario_name: Optional[str]
    initial_ego_speed: float
    initial_distance: float
    final_ego_speed: float
    final_relative_velocity: float
    min_relative_velocity: float
    min_distance: float
    aeb_activation_time: Optional[float]
    ttc_at_activation: Optional[float]
    max_braking_deceleration: float  # Commanded max deceleration magnitude
    max_actual_deceleration: float   # Actual delivered max deceleration magnitude
    ninety_percent_braking_response_time: Optional[float]  # 90% braking response time (s)
    stopping_time: Optional[float]
    stopping_distance: Optional[float]
    vehicle_stop_status: str  # "STOPPED" or "NOT_STOPPED"
    collision_status: bool  # True if collision occurred, False otherwise

    @property
    def braking_response_delay(self) -> Optional[float]:
        """Backward compatibility alias for ninety_percent_braking_response_time."""
        return self.ninety_percent_braking_response_time

    def print_summary(self) -> None:
        """Print a formatted, human-readable evaluation summary to stdout."""
        print("==================================================")
        print("         AEB PERFORMANCE METRICS SUMMARY          ")
        print("==================================================")
        if self.scenario_name:
            print(f" Scenario Name           : {self.scenario_name}")
        print(f" Initial Ego Speed       : {self.initial_ego_speed:.2f} m/s")
        print(f" Initial Distance        : {self.initial_distance:.2f} m")
        print(f" Final Ego Speed         : {self.final_ego_speed:.2f} m/s")
        print(f" Final Relative Velocity : {self.final_relative_velocity:.2f} m/s")
        print(f" Min Relative Velocity   : {self.min_relative_velocity:.2f} m/s")
        print(f" Minimum Distance        : {self.min_distance:.2f} m")
        print(
            f" Max Commanded Decel     : {self.max_braking_deceleration:.2f} m/s²"
        )
        print(
            f" Max Actual Decel        : {self.max_actual_deceleration:.2f} m/s²"
        )

        if self.aeb_activation_time is not None:
            print(f" AEB Activation Time     : {self.aeb_activation_time:.2f} s")
            print(f" TTC at Activation       : {self.ttc_at_activation:.2f} s")
        else:
            print(f" AEB Activation Time     : Not Triggered")
            print(f" TTC at Activation       : N/A")

        if self.ninety_percent_braking_response_time is not None:
            print(f" 90% Braking Response Time : {self.ninety_percent_braking_response_time:.2f} s")
        else:
            print(f" 90% Braking Response Time : N/A")

        if self.stopping_time is not None:
            print(f" Total Stopping Time     : {self.stopping_time:.2f} s")
            print(f" Total Stopping Distance : {self.stopping_distance:.2f} m")
        else:
            print(f" Total Stopping Time     : Vehicle did not stop")
            print(f" Total Stopping Distance : N/A")

        print(f" Vehicle Stop Status     : {self.vehicle_stop_status}")
        collision_str = "YES (COLLISION DETECTED)" if self.collision_status else "NO"
        print(f" Collision Status        : {collision_str}")
        print("==================================================")


class PerformanceEvaluator:
    """Evaluates telemetry trajectories from a SimulationResult object."""

    @staticmethod
    def evaluate(result: SimulationResult) -> AEBPerformanceMetrics:
        """
        Compute evaluation metrics from a SimulationResult trajectory.

        Args:
            result (SimulationResult): Trajectory telemetry dataset.

        Returns:
            AEBPerformanceMetrics: Structured performance metrics object.

        Raises:
            ValueError: If simulation result is empty.
        """
        if not result.samples:
            raise ValueError("SimulationResult contains no recorded samples.")

        samples = result.samples
        sample_0 = samples[0]
        sample_final = samples[-1]

        scenario_name = result.scenario_name
        initial_ego_speed = sample_0.ego_speed
        initial_distance = sample_0.distance
        final_ego_speed = sample_final.ego_speed
        final_relative_velocity = sample_final.relative_velocity
        min_relative_velocity = min(s.relative_velocity for s in samples)
        min_distance = min(s.distance for s in samples)

        # Maximum commanded deceleration applied
        min_cmd_accel = min(s.desired_deceleration for s in samples)
        max_braking_deceleration = abs(min_cmd_accel) if min_cmd_accel < 0.0 else 0.0

        # Maximum actual deceleration delivered
        min_act_accel = min(s.actual_deceleration for s in samples)
        max_actual_deceleration = abs(min_act_accel) if min_act_accel < 0.0 else 0.0

        # AEB activation metrics & 90% response time computation
        active_braking_samples = [
            s for s in samples if s.desired_deceleration < 0.0
        ]
        if active_braking_samples:
            first_brake = active_braking_samples[0]
            aeb_activation_time = first_brake.time
            ttc_at_activation = first_brake.ttc

            # Magnitude threshold: abs(actual_deceleration) >= 0.90 * abs(initial_commanded_deceleration)
            target_90_magnitude = 0.90 * abs(first_brake.desired_deceleration)
            first_brake_idx = samples.index(first_brake)

            reach_90_sample = None
            for s in samples[first_brake_idx:]:
                if abs(s.actual_deceleration) >= target_90_magnitude:
                    reach_90_sample = s
                    break

            if reach_90_sample is not None:
                ninety_percent_braking_response_time = round(
                    reach_90_sample.time - aeb_activation_time, 4
                )
            else:
                ninety_percent_braking_response_time = None
        else:
            aeb_activation_time = None
            ttc_at_activation = None
            ninety_percent_braking_response_time = None

        # Stopping time and Ego stopping distance via numerical integration
        stopped_samples = [
            s for s in samples if math.isclose(s.ego_speed, 0.0, abs_tol=1e-5)
        ]
        if stopped_samples:
            first_stop = stopped_samples[0]
            stopping_time = first_stop.time
            vehicle_stop_status = "STOPPED"

            # Compute Ego stopping distance using sample timestamps without hardcoded fallbacks
            stop_index = samples.index(first_stop)
            if stop_index > 0:
                ego_dist = 0.0
                valid_integration = True
                for i in range(stop_index):
                    if i + 1 < len(samples):
                        dt = samples[i + 1].time - samples[i].time
                    elif i > 0:
                        dt = samples[i].time - samples[i - 1].time
                    else:
                        valid_integration = False
                        break
                    ego_dist += samples[i].ego_speed * dt

                stopping_distance = ego_dist if valid_integration else None
            else:
                stopping_distance = 0.0
        else:
            stopping_time = None
            stopping_distance = None
            vehicle_stop_status = "NOT_STOPPED"

        # Robust Collision Criterion Definition:
        # Collision occurs if unclamped raw_distance <= 0.0 while relative_velocity > 1e-4 m/s
        collision_status = False
        for s in samples:
            if s.raw_distance <= 0.0 and s.relative_velocity > 1e-4:
                collision_status = True
                break

        return AEBPerformanceMetrics(
            scenario_name=scenario_name,
            initial_ego_speed=initial_ego_speed,
            initial_distance=initial_distance,
            final_ego_speed=final_ego_speed,
            final_relative_velocity=final_relative_velocity,
            min_relative_velocity=min_relative_velocity,
            min_distance=min_distance,
            aeb_activation_time=aeb_activation_time,
            ttc_at_activation=ttc_at_activation,
            max_braking_deceleration=max_braking_deceleration,
            max_actual_deceleration=max_actual_deceleration,
            ninety_percent_braking_response_time=ninety_percent_braking_response_time,
            stopping_time=stopping_time,
            stopping_distance=stopping_distance,
            vehicle_stop_status=vehicle_stop_status,
            collision_status=collision_status,
        )
