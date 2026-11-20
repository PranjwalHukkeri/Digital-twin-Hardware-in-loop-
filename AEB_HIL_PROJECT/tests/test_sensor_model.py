import pytest

from src.sensor_model import SensorModel


def test_zero_latency_is_immediate():
    sensor = SensorModel(latency=0.0, dt=0.05)

    assert sensor.measure(10.0) == 10.0
    assert sensor.measure(20.0) == 20.0


def test_one_step_latency():
    sensor = SensorModel(latency=0.05, dt=0.05)

    values = [10.0, 20.0, 30.0, 40.0]
    outputs = [sensor.measure(value) for value in values]

    assert outputs == [10.0, 10.0, 20.0, 30.0]


def test_two_step_latency():
    sensor = SensorModel(latency=0.10, dt=0.05)

    values = [10.0, 20.0, 30.0, 40.0, 50.0]
    outputs = [sensor.measure(value) for value in values]

    assert outputs == [10.0, 10.0, 10.0, 20.0, 30.0]


def test_reset_clears_sensor_history():
    sensor = SensorModel(latency=0.10, dt=0.05)

    sensor.measure(10.0)
    sensor.measure(20.0)

    sensor.reset()

    assert sensor.measure(100.0) == 100.0


def test_negative_latency_is_rejected():
    with pytest.raises(ValueError):
        SensorModel(latency=-0.05, dt=0.05)


def test_non_integer_latency_is_rejected():
    with pytest.raises(ValueError):
        SensorModel(latency=0.12, dt=0.05)


def test_invalid_dt_is_rejected():
    with pytest.raises(ValueError):
        SensorModel(latency=0.05, dt=0.0)


def test_non_finite_sensor_value_is_rejected():
    sensor = SensorModel(latency=0.0, dt=0.05)

    with pytest.raises(ValueError):
        sensor.measure(float("nan"))

    with pytest.raises(ValueError):
        sensor.measure(float("inf"))


# A. Zero-noise compatibility
def test_zero_latency_zero_noise_compatibility():
    sensor = SensorModel(latency=0.0, noise_std=0.0, dt=0.05)

    values = [10.0, 20.0, 30.0]
    outputs = [sensor.measure(v) for v in values]

    assert outputs == [10.0, 20.0, 30.0]


def test_one_step_latency_zero_noise_compatibility():
    sensor = SensorModel(latency=0.05, noise_std=0.0, dt=0.05)

    values = [10.0, 20.0, 30.0, 40.0]
    outputs = [sensor.measure(v) for v in values]

    assert outputs == [10.0, 10.0, 20.0, 30.0]


# B. Invalid noise_std
def test_negative_noise_std_is_rejected():
    with pytest.raises(ValueError):
        SensorModel(noise_std=-0.1)


def test_nan_noise_std_is_rejected():
    with pytest.raises(ValueError):
        SensorModel(noise_std=float("nan"))


def test_infinity_noise_std_is_rejected():
    with pytest.raises(ValueError):
        SensorModel(noise_std=float("inf"))

    with pytest.raises(ValueError):
        SensorModel(noise_std=float("-inf"))


# C. Reproducibility
def test_noise_reproducibility_with_same_seed():
    sensor1 = SensorModel(latency=0.0, noise_std=0.5, seed=42)
    sensor2 = SensorModel(latency=0.0, noise_std=0.5, seed=42)

    inputs = [10.0, 20.0, 30.0, 40.0, 50.0] * 10
    out1 = [sensor1.measure(v) for v in inputs]
    out2 = [sensor2.measure(v) for v in inputs]

    assert out1 == out2


# D. Different seeds
def test_noise_different_seeds_produce_different_sequences():
    sensor1 = SensorModel(latency=0.0, noise_std=0.5, seed=42)
    sensor2 = SensorModel(latency=0.0, noise_std=0.5, seed=99)

    inputs = [10.0] * 50
    out1 = [sensor1.measure(v) for v in inputs]
    out2 = [sensor2.measure(v) for v in inputs]

    assert out1 != out2


# E. Noise is actually applied
def test_nonzero_noise_is_applied():
    sensor = SensorModel(latency=0.0, noise_std=0.5, seed=123)

    inputs = [10.0] * 20
    outputs = [sensor.measure(v) for v in inputs]

    assert any(out != inp for out, inp in zip(outputs, inputs))


# F. Statistical sanity
def test_noise_statistical_sanity():
    import numpy as np

    noise_std = 0.5
    sensor = SensorModel(latency=0.0, noise_std=noise_std, seed=42)
    n_samples = 10000

    # Constant input 0.0, so measurement = 0.0 + noise = noise
    samples = np.array([sensor.measure(0.0) for _ in range(n_samples)])

    sample_mean = float(np.mean(samples))
    sample_std = float(np.std(samples, ddof=1))

    assert sample_mean == pytest.approx(0.0, abs=0.05)
    assert sample_std == pytest.approx(noise_std, rel=0.05)
