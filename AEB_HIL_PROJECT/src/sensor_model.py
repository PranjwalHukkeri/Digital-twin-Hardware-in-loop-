from collections import deque
import math
import numpy as np


class SensorModel:
    """
    Discrete-time sensor model for AEB simulation.

    Current scope:
    - deterministic measurement latency
    - zero-mean Gaussian measurement noise
    - no faults

    Latency must be an integer multiple of the simulation timestep.
    """

    def __init__(
        self,
        latency=0.0,
        noise_std=0.0,
        dt=0.05,
        seed=None,
    ):
        if not math.isfinite(latency) or latency < 0.0:
            raise ValueError("latency must be finite and >= 0")

        if isinstance(noise_std, bool) or not math.isfinite(noise_std) or noise_std < 0.0:
            raise ValueError("noise_std must be finite and >= 0")

        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and > 0")

        delay_steps_float = latency / dt
        delay_steps = round(delay_steps_float)

        if not math.isclose(
            delay_steps_float,
            delay_steps,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError(
                "latency must be an integer multiple of dt"
            )

        self.latency = float(latency)
        self.noise_std = float(noise_std)
        self.dt = float(dt)
        self.seed = seed
        self.delay_steps = int(delay_steps)

        self.rng = np.random.default_rng(seed)
        self._rng = self.rng

        self._history = deque()

    def reset(self):
        """Clear stored sensor history."""
        self._history.clear()

    def measure(self, value):
        """
        Return the sensor measurement after the configured delay and noise.

        During startup, before enough history exists, the earliest
        available value is returned.
        """
        if not math.isfinite(value):
            raise ValueError("sensor value must be finite")

        self._history.append(float(value))

        if self.delay_steps == 0:
            delayed_value = float(value)
        elif len(self._history) <= self.delay_steps:
            delayed_value = self._history[0]
        else:
            delayed_value = self._history.popleft()

        if self.noise_std == 0.0:
            return delayed_value

        noise = float(self.rng.normal(0.0, self.noise_std))
        return float(delayed_value + noise)
