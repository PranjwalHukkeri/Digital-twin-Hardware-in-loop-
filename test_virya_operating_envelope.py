"""
Unit Tests for VIRYA Operating-Envelope Validation (Step 10).

Validates:
1. Speed grid consistency [0.50, 1.00, 1.39] m/s
2. Initial separation distance grid [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0] m
3. Target speed relationships (CCR: 0.0, CCRm/CCRb: 0.5 * v_ego)
4. Scenario generation integrity for CCR, CCRm, and CCRb
5. Exact run-count formula (3 × 8 × 3 × 10 = 720)
6. Baseline parameter preservation in production controller
7. Deterministic reproducibility with fixed seed
8. Invalid speed and distance rejection
9. Data output schema and metric completeness
10. Reduced end-to-end execution test
"""

import pytest
import math
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
from src.aeb_controller import AEBController
from analysis.virya_operating_envelope import (
    VIRYA_SPEEDS,
    INITIAL_DISTANCES,
    SCENARIO_TYPES,
    SEEDS,
    CCRB_TARGET_DECEL,
    CCRB_BRAKE_START_TIME,
    create_scenario,
    compute_theoretical_initial_ttc,
    run_single_simulation,
    OperatingEnvelopeRunResult,
)


def test_operating_speed_grid():
    """Verify exact speed levels for low-speed VIRYA operating envelope."""
    assert VIRYA_SPEEDS == [0.50, 1.00, 1.39], "Operating speeds must match [0.50, 1.00, 1.39] m/s."
    assert all(v > 0 for v in VIRYA_SPEEDS)
    assert max(VIRYA_SPEEDS) == 1.39, "Upper bound must be the documented 1.39 m/s (~5 km/h)."


def test_initial_distance_grid():
    """Verify initial separation distance matrix spans 1.0 m to 6.0 m across 8 levels."""
    expected = [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0]
    assert INITIAL_DISTANCES == expected, "Distance grid must match the 8 specified points."
    assert len(INITIAL_DISTANCES) == 8


def test_total_run_count_calculation():
    """Verify the exact primary experiment size equals 720 simulation runs."""
    total_runs = len(VIRYA_SPEEDS) * len(INITIAL_DISTANCES) * len(SCENARIO_TYPES) * len(SEEDS)
    assert total_runs == 720, f"Expected exactly 720 runs, got {total_runs}."


def test_baseline_parameters_preserved():
    """Verify that production AEBController parameters match the Step 9 provisional baseline."""
    assert TTC_WARNING == 4.00
    assert TTC_LEVEL_1 == 3.00
    assert TTC_LEVEL_2 == 2.25
    assert TTC_LEVEL_3 == 1.75
    assert A_LEVEL_1 == -2.0
    assert A_LEVEL_2 == -4.0
    assert A_LEVEL_3 == -6.0
    assert MIN_BRAKE_DURATION == 0.20

    ctrl = AEBController()
    assert ctrl.min_brake_duration == 0.20
    assert ctrl.current_state == "SAFE"


def test_target_speed_relationships():
    """Verify target speed definition for CCR, CCRm, and CCRb."""
    for v in VIRYA_SPEEDS:
        # CCR: stationary
        s_ccr = create_scenario("CCR", v, 5.0)
        assert s_ccr.initial_target_speed == 0.0
        assert s_ccr.initial_ego_speed == v

        # CCRm: moving target at 0.5 * v_ego
        s_ccrm = create_scenario("CCRm", v, 5.0)
        assert math.isclose(s_ccrm.initial_target_speed, 0.5 * v, rel_tol=1e-5)
        assert s_ccrm.initial_ego_speed > s_ccrm.initial_target_speed >= 0.0

        # CCRb: target at 0.5 * v_ego braking at 1.0 s
        s_ccrb = create_scenario("CCRb", v, 5.0)
        assert math.isclose(s_ccrb.initial_target_speed, 0.5 * v, rel_tol=1e-5)
        assert s_ccrb.get_target_acceleration(0.5) == 0.0
        assert s_ccrb.get_target_acceleration(1.0) == CCRB_TARGET_DECEL
        assert s_ccrb.get_target_acceleration(2.0) == CCRB_TARGET_DECEL


def test_theoretical_initial_ttc_calculation():
    """Verify theoretical initial TTC mapping for stationary and moving targets."""
    # CCR at 1.39 m/s
    ttc_ccr_6m = compute_theoretical_initial_ttc("CCR", 1.39, 6.0)
    assert math.isclose(ttc_ccr_6m, 6.0 / 1.39, rel_tol=1e-4)

    # At 1.39 m/s, theoretical boundaries:
    # 4.00s -> 5.56m, 3.00s -> 4.17m, 2.25s -> 3.13m, 1.75s -> 2.43m
    assert math.isclose(5.56 / 1.39, 4.0, rel_tol=1e-2)
    assert math.isclose(4.17 / 1.39, 3.0, rel_tol=1e-2)
    assert math.isclose(3.13 / 1.39, 2.25, rel_tol=1e-2)
    assert math.isclose(2.43 / 1.39, 1.75, rel_tol=1e-2)

    # CCRm at 1.00 m/s: v_rel = 0.5 m/s, so d0=2.0m gives TTC = 2.0 / 0.5 = 4.0s
    ttc_ccrm = compute_theoretical_initial_ttc("CCRm", 1.00, 2.0)
    assert math.isclose(ttc_ccrm, 4.00, rel_tol=1e-4)


def test_invalid_speed_and_distance_rejection():
    """Verify input validation handles invalid speeds and distances."""
    with pytest.raises(ValueError):
        create_scenario("CCR", -1.0, 5.0)

    with pytest.raises(ValueError):
        create_scenario("CCR", 1.0, -5.0)

    with pytest.raises(ValueError):
        create_scenario("UnknownScenario", 1.0, 5.0)


def test_deterministic_reproducibility_with_seed():
    """Verify that identical simulation runs with the same seed yield identical results."""
    res1 = run_single_simulation("CCR", 1.00, 3.0, seed=42)
    res2 = run_single_simulation("CCR", 1.00, 3.0, seed=42)

    assert res1.min_distance == res2.min_distance
    assert res1.collision_status == res2.collision_status
    assert res1.aeb_activation_time == res2.aeb_activation_time
    assert res1.ttc_at_activation == res2.ttc_at_activation
    assert res1.max_actual_decel == res2.max_actual_decel
    assert res1.fsm_state_transitions == res2.fsm_state_transitions
    assert res1.triplet_reversal_count == res2.triplet_reversal_count


def test_output_schema_completeness():
    """Verify that all required Step 10 metrics are present in the output dictionary."""
    res = run_single_simulation("CCR", 1.00, 2.0, seed=42)
    d = res.to_dict()

    required_keys = [
        "scenario_type",
        "scenario_name",
        "ego_speed",
        "initial_distance",
        "target_speed_initial",
        "seed",
        "theoretical_initial_ttc",
        "min_distance",
        "collision_status",
        "aeb_activation_time",
        "ttc_at_activation",
        "max_commanded_decel",
        "max_actual_decel",
        "response_time_90",
        "stopping_time",
        "stopping_distance",
        "final_ego_speed",
        "final_relative_velocity",
        "min_relative_velocity",
        "vehicle_stop_status",
        "fsm_state_transitions",
        "triplet_reversal_count",
        "max_fsm_level_reached",
        "activated_braking",
        "stopping_feasible",
    ]

    for k in required_keys:
        assert k in d, f"Missing required key in output schema: {k}"

    assert isinstance(d["collision_status"], bool)
    assert isinstance(d["min_distance"], float)
    assert isinstance(d["fsm_state_transitions"], int)
    assert isinstance(d["triplet_reversal_count"], int)
    assert d["vehicle_stop_status"] in {"STOPPED", "NOT_STOPPED"}
    assert d["max_fsm_level_reached"] in {"SAFE", "WARNING", "AEB_LEVEL_1", "AEB_LEVEL_2", "AEB_LEVEL_3"}


def test_reduced_execution_all_scenarios():
    """Execute a reduced set of runs across CCR, CCRm, and CCRb to confirm pipeline execution."""
    for scen in ["CCR", "CCRm", "CCRb"]:
        res = run_single_simulation(scen, ego_speed=1.39, distance=4.0, seed=42)
        assert res.ego_speed == 1.39
        assert res.initial_distance == 4.0
        assert res.min_distance >= 0.0
        assert not res.collision_status
        assert res.max_fsm_level_reached in {"SAFE", "WARNING", "AEB_LEVEL_1", "AEB_LEVEL_2", "AEB_LEVEL_3"}
