"""
Automated Unit & Closed-Loop Verification Tests for SimulationEngine.
"""

import pytest
from src.simulation import SimulationEngine, SimulationResult


def test_baseline_simulation_execution():
    """Verify execution of initial baseline scenario (v_ego=1.0 m/s, target=0.0 m/s, d=5.0m)."""
    engine = SimulationEngine()
    result = engine.run(
        initial_ego_speed=1.0, initial_target_speed=0.0, initial_distance=5.0
    )

    assert isinstance(result, SimulationResult)
    assert len(result.samples) > 0

    # 1. Verify TTC decreases while approaching target before braking
    sample_t0 = result.samples[0]
    sample_warning = [s for s in result.samples if s.aeb_state == "WARNING"][0]

    assert sample_t0.ttc > sample_warning.ttc
    assert sample_t0.aeb_state == "SAFE"
    assert sample_warning.aeb_state == "WARNING"

    # 2. Verify AEB activates when appropriate TTC threshold is reached (AEB_LEVEL_1 at TTC <= 3.0s)
    active_braking_samples = [
        s for s in result.samples if s.desired_deceleration < 0.0
    ]
    assert len(active_braking_samples) > 0

    first_brake = active_braking_samples[0]
    assert first_brake.aeb_state in ["AEB_LEVEL_1", "AEB_LEVEL_2", "AEB_LEVEL_3"]
    assert first_brake.ttc <= 3.00

    # 3. Verify Ego speed decreases after braking
    brake_idx = result.samples.index(first_brake)
    speed_at_brake = first_brake.ego_speed
    speed_after_brake = result.samples[brake_idx + 1].ego_speed
    assert speed_after_brake < speed_at_brake

    # 4. Verify distance never becomes negative
    assert all(s.distance >= 0.0 for s in result.samples)

    # 5. Verify ego speed never becomes negative
    assert all(s.ego_speed >= 0.0 for s in result.samples)


def test_simulation_higher_speed_stopping():
    """Verify closed-loop deceleration under higher speed approach (v_ego=10.0 m/s, d=30.0m)."""
    engine = SimulationEngine(max_time=15.0)
    result = engine.run(
        initial_ego_speed=10.0, initial_target_speed=0.0, initial_distance=30.0
    )

    assert len(result.samples) > 0
    assert all(s.distance >= 0.0 for s in result.samples)
    assert all(s.ego_speed >= 0.0 for s in result.samples)

    # Verify speed was significantly reduced from 10.0 m/s
    final_sample = result.samples[-1]
    assert final_sample.ego_speed < 1.0


def test_simulation_no_crash():
    """Verify numerical stability across various speeds and distances."""
    engine = SimulationEngine()
    # High speed scenario (20 m/s, 30m gap)
    res_high = engine.run(
        initial_ego_speed=20.0, initial_target_speed=0.0, initial_distance=30.0
    )
    assert len(res_high.samples) > 0
    assert all(s.distance >= 0.0 for s in res_high.samples)

    # Low distance scenario (10 m/s, 2m gap)
    res_close = engine.run(
        initial_ego_speed=10.0, initial_target_speed=0.0, initial_distance=2.0
    )
    assert len(res_close.samples) > 0
    assert all(s.distance >= 0.0 for s in res_close.samples)
