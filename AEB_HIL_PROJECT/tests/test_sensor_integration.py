from src.simulation import SimulationEngine


def test_sensor_latency_changes_aeb_response():
    """
    Verify that non-zero sensor latency changes the closed-loop AEB response.

    The test compares an ideal sensor against a 100 ms delayed sensor
    for the same baseline scenario.
    """
    ideal = SimulationEngine(
        dt=0.05,
        max_time=3.0,
        sensor_latency=0.0,
    )

    delayed = SimulationEngine(
        dt=0.05,
        max_time=3.0,
        sensor_latency=0.10,
    )

    ideal_result = ideal.run(
        initial_ego_speed=15.0,
        initial_target_speed=0.0,
        initial_distance=30.0,
    )

    delayed_result = delayed.run(
        initial_ego_speed=15.0,
        initial_target_speed=0.0,
        initial_distance=30.0,
    )

    ideal_ttc = ideal_result.ttc_history
    delayed_ttc = delayed_result.ttc_history

    # Both simulations must produce telemetry.
    assert ideal_ttc
    assert delayed_ttc

    # The delayed sensor must change the controller's TTC history.
    assert ideal_ttc != delayed_ttc


def test_zero_sensor_latency_preserves_baseline_behavior():
    """
    Verify that explicitly setting zero latency preserves the baseline
    simulation behavior.
    """
    default_engine = SimulationEngine(
        dt=0.05,
        max_time=3.0,
    )

    zero_latency_engine = SimulationEngine(
        dt=0.05,
        max_time=3.0,
        sensor_latency=0.0,
    )

    default_result = default_engine.run(
        initial_ego_speed=15.0,
        initial_target_speed=0.0,
        initial_distance=30.0,
    )

    zero_latency_result = zero_latency_engine.run(
        initial_ego_speed=15.0,
        initial_target_speed=0.0,
        initial_distance=30.0,
    )

    assert default_result.ttc_history == zero_latency_result.ttc_history
    assert (
        default_result.aeb_state_history
        == zero_latency_result.aeb_state_history
    )
    assert (
        default_result.deceleration_history
        == zero_latency_result.deceleration_history
    )
