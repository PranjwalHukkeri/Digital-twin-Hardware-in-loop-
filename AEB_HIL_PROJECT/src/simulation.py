"""
AEB Closed-Loop Simulation Engine Module

Orchestrates the discrete-time execution loop integrating the AEBController,
BrakeActuator, SensorModel, VehicleModel, and Scenario definitions,
recording full telemetry trajectories at each timestep.
"""

from dataclasses import dataclass, field
from typing import List, Optional
import math

from config.parameters import (
    DT,
    INITIAL_EGO_SPEED,
    INITIAL_TARGET_SPEED,
    INITIAL_DISTANCE,
)
from src.aeb_controller import AEBController
from src.brake_actuator import BrakeActuator
from src.sensor_model import SensorModel
from src.vehicle_model import VehicleModel
from scenarios.scenarios import Scenario


@dataclass
class SimulationSample:
    """Telemetry signals recorded at a single simulation timestep."""

    time: float
    ego_speed: float
    target_speed: float
    distance: float
    relative_velocity: float = 0.0
    ttc: float = float("inf")
    aeb_state: str = "SAFE"
    desired_deceleration: float = 0.0
    raw_distance: float = 0.0
    actual_deceleration: float = 0.0
    measured_ego_speed: float = 0.0
    measured_target_speed: float = 0.0
    measured_distance: float = 0.0


@dataclass
class SimulationResult:
    """Complete time-series dataset of a simulation run."""

    scenario_name: Optional[str] = None
    samples: List[SimulationSample] = field(default_factory=list)

    @property
    def time_history(self) -> List[float]:
        return [s.time for s in self.samples]

    @property
    def ego_speed_history(self) -> List[float]:
        return [s.ego_speed for s in self.samples]

    @property
    def target_speed_history(self) -> List[float]:
        return [s.target_speed for s in self.samples]

    @property
    def distance_history(self) -> List[float]:
        return [s.distance for s in self.samples]

    @property
    def measured_ego_speed_history(self) -> List[float]:
        return [s.measured_ego_speed for s in self.samples]

    @property
    def measured_target_speed_history(self) -> List[float]:
        return [s.measured_target_speed for s in self.samples]

    @property
    def measured_distance_history(self) -> List[float]:
        return [s.measured_distance for s in self.samples]

    @property
    def relative_velocity_history(self) -> List[float]:
        return [s.relative_velocity for s in self.samples]

    @property
    def ttc_history(self) -> List[float]:
        return [s.ttc for s in self.samples]

    @property
    def aeb_state_history(self) -> List[str]:
        return [s.aeb_state for s in self.samples]

    @property
    def deceleration_history(self) -> List[float]:
        return [s.desired_deceleration for s in self.samples]

    @property
    def actual_deceleration_history(self) -> List[float]:
        return [s.actual_deceleration for s in self.samples]

    @property
    def raw_distance_history(self) -> List[float]:
        return [s.raw_distance for s in self.samples]


class SimulationEngine:
    """Closed-loop discrete-time simulation environment for AEB validation."""

    def __init__(
        self,
        dt: float = DT,
        max_time: float = 10.0,
        min_brake_duration: Optional[float] = None,
        actuator_tau: float = 0.0,
        sensor_latency: float = 0.0,
        distance_noise_std: float = 0.0,
        distance_seed: Optional[int] = None,
        ego_speed_noise_std: float = 0.0,
        ego_speed_seed: Optional[int] = None,
        target_speed_noise_std: float = 0.0,
        target_speed_seed: Optional[int] = None,
    ):
        if dt <= 0.0 or not math.isfinite(dt):
            raise ValueError(
                f"Invalid timestep dt={dt}. Must be positive and finite."
            )

        if max_time <= 0.0 or not math.isfinite(max_time):
            raise ValueError(
                f"Invalid max_time={max_time}. Must be positive and finite."
            )

        self.dt = dt
        self.max_time = max_time

        if min_brake_duration is None:
            self.controller = AEBController()
        else:
            self.controller = AEBController(
                min_brake_duration=min_brake_duration
            )

        self.actuator = BrakeActuator(tau=actuator_tau)

        self.ego_speed_sensor = SensorModel(
            latency=sensor_latency,
            noise_std=ego_speed_noise_std,
            dt=self.dt,
            seed=ego_speed_seed,
        )
        self.target_speed_sensor = SensorModel(
            latency=sensor_latency,
            noise_std=target_speed_noise_std,
            dt=self.dt,
            seed=target_speed_seed,
        )
        self.distance_sensor = SensorModel(
            latency=sensor_latency,
            noise_std=distance_noise_std,
            dt=self.dt,
            seed=distance_seed,
        )

        self.vehicle_model: Optional[VehicleModel] = None

    def run(
        self,
        initial_ego_speed: float = INITIAL_EGO_SPEED,
        initial_target_speed: float = INITIAL_TARGET_SPEED,
        initial_distance: float = INITIAL_DISTANCE,
        scenario: Optional[Scenario] = None,
        min_brake_duration: Optional[float] = None,
        actuator_tau: Optional[float] = None,
    ) -> SimulationResult:
        """Execute a closed-loop simulation."""

        if min_brake_duration is not None:
            self.controller = AEBController(
                min_brake_duration=min_brake_duration
            )

        if actuator_tau is not None:
            self.actuator.set_tau(actuator_tau)

        self.actuator.reset()

        if scenario is not None:
            scenario_name = scenario.name
            ego_speed_init = scenario.initial_ego_speed
            target_speed_init = scenario.initial_target_speed
            dist_init = scenario.initial_distance
            max_t = scenario.simulation_duration
            target_accel_fn = scenario.get_target_acceleration
        else:
            scenario_name = "Baseline_Simulation"
            ego_speed_init = initial_ego_speed
            target_speed_init = initial_target_speed
            dist_init = initial_distance
            max_t = self.max_time

            def target_accel_fn(t: float) -> float:
                return 0.0

        self.vehicle_model = VehicleModel(
            ego_speed_init,
            dist_init,
        )

        self.controller.reset()
        self.ego_speed_sensor.reset()
        self.target_speed_sensor.reset()
        self.distance_sensor.reset()

        result = SimulationResult(
            scenario_name=scenario_name
        )

        current_time = 0.0
        current_target_speed = target_speed_init

        while current_time <= max_t + 1e-12:

            true_ego_speed = self.vehicle_model.ego_speed
            true_distance = self.vehicle_model.distance
            true_target_speed = current_target_speed

            measured_ego_speed = self.ego_speed_sensor.measure(
                true_ego_speed
            )
            measured_target_speed = self.target_speed_sensor.measure(
                true_target_speed
            )
            measured_distance = self.distance_sensor.measure(
                true_distance
            )

            relative_velocity = (
                self.controller.calculate_relative_velocity(
                    measured_ego_speed,
                    measured_target_speed,
                )
            )

            ttc = self.controller.calculate_ttc(
                measured_ego_speed,
                measured_target_speed,
                measured_distance,
            )

            aeb_state, desired_deceleration = (
                self.controller.update_state(
                    ttc=ttc,
                    v_rel=relative_velocity,
                    dt=self.dt,
                )
            )

            actual_deceleration = self.actuator.step(
                desired_deceleration,
                self.dt,
            )

            raw_distance_next = (
                true_distance
                + (true_target_speed - true_ego_speed) * self.dt
            )

            result.samples.append(
                SimulationSample(
                    time=round(current_time, 6),
                    ego_speed=true_ego_speed,
                    target_speed=true_target_speed,
                    distance=max(true_distance, 0.0),
                    relative_velocity=relative_velocity,
                    ttc=ttc,
                    aeb_state=aeb_state,
                    desired_deceleration=desired_deceleration,
                    actual_deceleration=actual_deceleration,
                    raw_distance=raw_distance_next,
                    measured_ego_speed=measured_ego_speed,
                    measured_target_speed=measured_target_speed,
                    measured_distance=measured_distance,
                )
            )

            self.vehicle_model.step(
                actual_deceleration,
                current_target_speed,
                self.dt,
            )

            target_acceleration = target_accel_fn(current_time)

            current_target_speed = max(
                0.0,
                current_target_speed
                + target_acceleration * self.dt,
            )

            current_time += self.dt

            if (
                self.vehicle_model.ego_speed <= 1e-5
                and relative_velocity <= 0.0
            ):
                break

            if raw_distance_next <= 0.0:
                break

        return result
