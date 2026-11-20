"""
Automated Unit Tests for Sensitivity Analysis Package.

Verifies:
- Count of FSM state transitions
- Active braking episode count and total active braking duration computation
- Deterministic boundary chattering detection logic (synthetic results)
- Execution of MIN_BRAKE_DURATION sweep
- Execution of operating condition sweeps
- Deterministic vehicle stop status verification ("STOPPED" vs "NOT_STOPPED")
"""

import os
import pytest
from analysis.sensitivity_analysis import (
    count_fsm_state_transitions,
    compute_active_braking_metrics,
    check_boundary_chattering,
    count_triplet_reversal_patterns,
    run_min_brake_duration_sweep,
    run_operating_condition_sweep,
)
from src.simulation import SimulationEngine, SimulationResult, SimulationSample
from src.metrics import PerformanceEvaluator
from scenarios.scenarios import CCRScenario


def make_synthetic_result(states, decels=None, speeds=None):
    """Helper to create synthetic SimulationResult objects for deterministic testing."""
    samples = []
    if decels is None:
        decels = [0.0 if s in ("SAFE", "WARNING") else -2.0 for s in states]
    if speeds is None:
        speeds = [10.0 for _ in states]

    for i, (st, dec, spd) in enumerate(zip(states, decels, speeds)):
        samples.append(
            SimulationSample(
                time=round(i * 0.05, 2),
                ego_speed=spd,
                target_speed=0.0,
                distance=10.0,
                relative_velocity=spd,
                ttc=1.0,
                aeb_state=st,
                desired_deceleration=dec,
                raw_distance=10.0,
            )
        )
    return SimulationResult(scenario_name="Synthetic_Test", samples=samples)


def test_count_fsm_state_transitions():
    """Test counting FSM state transitions on simulation result."""
    engine = SimulationEngine()
    result = engine.run(scenario=CCRScenario(initial_ego_speed=15.0, initial_distance=30.0))
    transitions = count_fsm_state_transitions(result)
    assert isinstance(transitions, int)
    assert transitions >= 0


def test_compute_active_braking_metrics():
    """Test active braking episode count and duration calculation."""
    engine = SimulationEngine()
    result = engine.run(scenario=CCRScenario(initial_ego_speed=15.0, initial_distance=30.0))
    episodes, total_dur = compute_active_braking_metrics(result)
    assert episodes >= 1
    assert total_dur > 0.0


def test_chattering_case_a_l1_warning_l1():
    """Deterministic test a: AEB_LEVEL_1 -> WARNING -> AEB_LEVEL_1 is chattering."""
    states = ["AEB_LEVEL_1", "WARNING", "AEB_LEVEL_1"]
    res = make_synthetic_result(states)
    has_chattering, desc = check_boundary_chattering(res)
    assert has_chattering is True
    assert "Boundary Chattering Detected" in desc


def test_chattering_case_b_l2_l1_l2():
    """Deterministic test b: AEB_LEVEL_2 -> AEB_LEVEL_1 -> AEB_LEVEL_2 is chattering."""
    states = ["AEB_LEVEL_2", "AEB_LEVEL_1", "AEB_LEVEL_2"]
    res = make_synthetic_result(states)
    has_chattering, desc = check_boundary_chattering(res)
    assert has_chattering is True
    assert "Boundary Chattering Detected" in desc


def test_chattering_case_c_monotonic_no_chattering():
    """Deterministic test c: SAFE -> WARNING -> L1 -> L2 -> L3 -> SAFE has no chattering."""
    states = ["SAFE", "WARNING", "AEB_LEVEL_1", "AEB_LEVEL_2", "AEB_LEVEL_3", "SAFE"]
    res = make_synthetic_result(states)
    has_chattering, desc = check_boundary_chattering(res)
    assert has_chattering is False
    assert "No Boundary Chattering" in desc


def test_chattering_case_d_multiple_separated_episodes():
    """Deterministic test d: Multiple separated braking episodes are NOT automatically chattering."""
    states = ["SAFE", "AEB_LEVEL_1", "SAFE", "SAFE", "SAFE", "SAFE", "AEB_LEVEL_1", "SAFE"]
    decels = [0.0, -2.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0]
    res = make_synthetic_result(states, decels=decels)

    episodes, _ = compute_active_braking_metrics(res)
    has_chattering, desc = check_boundary_chattering(res)

    assert episodes == 2  # Recorded as multiple episodes
    assert has_chattering is False  # NOT automatically classified as chattering


def test_triplet_reversal_pattern_monotonic():
    """Test 1: Monotonic SAFE -> WARNING -> L1 -> L2 -> L3 has count = 0."""
    states = ["SAFE", "WARNING", "AEB_LEVEL_1", "AEB_LEVEL_2", "AEB_LEVEL_3"]
    res = make_synthetic_result(states)
    assert count_triplet_reversal_patterns(res) == 0


def test_triplet_reversal_pattern_single_reversal():
    """Test 2: Single reversal L1 -> WARNING -> L1 has count = 1."""
    states = ["AEB_LEVEL_1", "WARNING", "AEB_LEVEL_1"]
    res = make_synthetic_result(states)
    assert count_triplet_reversal_patterns(res) == 1


def test_triplet_reversal_pattern_alternating():
    """Test 3: Alternating L1 -> WARNING -> L1 -> WARNING -> L1 has count = 3."""
    states = ["AEB_LEVEL_1", "WARNING", "AEB_LEVEL_1", "WARNING", "AEB_LEVEL_1"]
    res = make_synthetic_result(states)
    assert count_triplet_reversal_patterns(res) == 3


def test_triplet_reversal_pattern_l2_l1_alternating():
    """Test 4: L2/L1 alternating L2 -> L1 -> L2 -> L1 -> L2 has count = 3."""
    states = ["AEB_LEVEL_2", "AEB_LEVEL_1", "AEB_LEVEL_2", "AEB_LEVEL_1", "AEB_LEVEL_2"]
    res = make_synthetic_result(states)
    assert count_triplet_reversal_patterns(res) == 3


def test_triplet_reversal_pattern_normal_escalation_release():
    """Test 5: Normal escalation/release SAFE -> WARNING -> L1 -> L2 -> L3 -> L2 -> L1 -> SAFE has count = 1."""
    states = ["SAFE", "WARNING", "AEB_LEVEL_1", "AEB_LEVEL_2", "AEB_LEVEL_3", "AEB_LEVEL_2", "AEB_LEVEL_1", "SAFE"]
    res = make_synthetic_result(states)
    assert count_triplet_reversal_patterns(res) == 1


def test_triplet_reversal_pattern_safe_return():
    """Test 6: SAFE return SAFE -> L1 -> SAFE has count = 0 (explicitly excluded)."""
    states = ["SAFE", "AEB_LEVEL_1", "SAFE"]
    res = make_synthetic_result(states)
    assert count_triplet_reversal_patterns(res) == 0


def test_deterministic_stopped_vehicle_status():
    """Test deterministic reporting of STOPPED when ego_speed reaches 0.0."""
    states = ["SAFE", "AEB_LEVEL_1", "SAFE"]
    speeds = [5.0, 1.0, 0.0]
    res = make_synthetic_result(states, speeds=speeds)

    metrics = PerformanceEvaluator.evaluate(res)
    assert metrics.vehicle_stop_status == "STOPPED"
    assert metrics.stopping_time == 0.10


def test_deterministic_not_stopped_vehicle_status():
    """Test deterministic reporting of NOT_STOPPED when residual ego_speed (0.2 m/s) remains."""
    states = ["SAFE", "AEB_LEVEL_1", "WARNING"]
    speeds = [5.0, 1.0, 0.2]
    res = make_synthetic_result(states, speeds=speeds)

    metrics = PerformanceEvaluator.evaluate(res)
    assert metrics.vehicle_stop_status == "NOT_STOPPED"
    assert metrics.stopping_time is None


def test_run_min_brake_duration_sweep(tmp_path):
    """Test MIN_BRAKE_DURATION sweep execution and CSV/plot output."""
    res_dir = str(tmp_path / "results")
    plots_dir = str(tmp_path / "plots")

    records = run_min_brake_duration_sweep(
        duration_values=[0.10, 0.20],
        results_dir=res_dir,
        plots_dir=plots_dir,
    )

    assert len(records) == 6  # 3 scenarios * 2 duration values
    assert os.path.exists(os.path.join(res_dir, "min_brake_duration_sweep.csv"))
    assert os.path.exists(os.path.join(plots_dir, "min_brake_duration_vs_state_transitions.png"))
    assert os.path.exists(os.path.join(plots_dir, "min_brake_duration_vs_minimum_distance.png"))
    assert os.path.exists(os.path.join(plots_dir, "min_brake_duration_vs_total_active_braking_duration.png"))


def test_run_operating_condition_sweep(tmp_path):
    """Test operating condition sweep execution and CSV output."""
    res_dir = str(tmp_path / "results")
    plots_dir = str(tmp_path / "plots")

    records = run_operating_condition_sweep(
        results_dir=res_dir,
        plots_dir=plots_dir,
    )

    assert len(records) == 10  # 5 speed points + 5 distance points
    assert os.path.exists(os.path.join(res_dir, "operating_condition_sweep.csv"))


def test_run_sensor_noise_sweep(tmp_path):
    """Test distance sensor noise sweep execution, CSV output, and plot generation."""
    import csv
    from analysis.sensor_noise_sweep import run_sensor_noise_sweep

    res_dir = str(tmp_path / "results")
    plots_dir = str(tmp_path / "plots")

    records = run_sensor_noise_sweep(
        noise_values=[0.0, 0.05],
        seed=42,
        results_dir=res_dir,
        plots_dir=plots_dir,
    )

    assert len(records) == 6  # 3 scenarios * 2 noise values
    csv_file = os.path.join(res_dir, "sensor_noise_sweep.csv")
    assert os.path.exists(csv_file)
    assert os.path.exists(
        os.path.join(plots_dir, "sensor_noise_vs_minimum_distance.png")
    )
    assert os.path.exists(
        os.path.join(plots_dir, "sensor_noise_vs_ttc_at_activation.png")
    )
    assert os.path.exists(
        os.path.join(plots_dir, "sensor_noise_vs_activation_time.png")
    )
    assert os.path.exists(
        os.path.join(plots_dir, "sensor_noise_vs_state_transitions.png")
    )
    assert os.path.exists(
        os.path.join(plots_dir, "sensor_noise_vs_chattering.png")
    )
    assert os.path.exists(
        os.path.join(plots_dir, "sensor_noise_vs_triplet_reversal_count.png")
    )

    with open(csv_file, "r") as f:
        reader = csv.DictReader(f)
        assert "boundary_chattering_detected" in reader.fieldnames
        assert "triplet_reversal_count" in reader.fieldnames
        for row in reader:
            assert row["boundary_chattering_detected"] in ["True", "False"]
            assert int(row["triplet_reversal_count"]) >= 0

    for r in records:
        assert r["noise_std"] in [0.0, 0.05]
        assert r["minimum_distance"] > 0.0
        assert r["seed"] == 42
        assert "boundary_chattering_detected" in r
        assert isinstance(r["boundary_chattering_detected"], bool)
        assert "triplet_reversal_count" in r
        assert isinstance(r["triplet_reversal_count"], int)
