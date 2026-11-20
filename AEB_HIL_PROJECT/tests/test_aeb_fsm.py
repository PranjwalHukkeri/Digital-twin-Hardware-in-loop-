"""
Automated Unit Tests for AEB Finite State Machine (FSM) Transitions.

Verifies FSM state transition precedence, dwell time retention, de-escalation,
threat clearance, and edge-case behavior.
"""

import math
import pytest
from src.aeb_controller import AEBController
from config.parameters import (
    TTC_WARNING,
    TTC_LEVEL_1,
    TTC_LEVEL_2,
    TTC_LEVEL_3,
    A_LEVEL_1,
    A_LEVEL_2,
    A_LEVEL_3,
)


@pytest.fixture
def fsm_controller():
    """Fixture providing a fresh AEBController FSM instance with 0.20s dwell time."""
    controller = AEBController(min_brake_duration=0.20)
    controller.reset()
    return controller


def test_transition_A_safe_to_warning(fsm_controller):
    """Test A: SAFE -> WARNING transition when 3.00 < TTC <= 4.00."""
    state, decel = fsm_controller.update_state(ttc=3.50, v_rel=2.0, dt=0.05)
    assert state == "WARNING"
    assert decel == 0.0


def test_transition_B_warning_to_level_1(fsm_controller):
    """Test B: WARNING -> AEB_LEVEL_1 transition when TTC <= 3.00."""
    fsm_controller.update_state(ttc=3.50, v_rel=2.0, dt=0.05)  # enters WARNING
    state, decel = fsm_controller.update_state(ttc=2.90, v_rel=2.0, dt=0.05)
    assert state == "AEB_LEVEL_1"
    assert decel == A_LEVEL_1


def test_transition_C_level_1_to_level_2(fsm_controller):
    """Test C: AEB_LEVEL_1 -> AEB_LEVEL_2 transition when TTC <= 2.25."""
    fsm_controller.update_state(ttc=2.90, v_rel=2.0, dt=0.05)  # enters LEVEL_1
    state, decel = fsm_controller.update_state(ttc=2.10, v_rel=2.0, dt=0.05)
    assert state == "AEB_LEVEL_2"
    assert decel == A_LEVEL_2


def test_transition_D_level_2_to_level_3(fsm_controller):
    """Test D: AEB_LEVEL_2 -> AEB_LEVEL_3 transition when TTC <= 1.75."""
    fsm_controller.update_state(ttc=2.10, v_rel=2.0, dt=0.05)  # enters LEVEL_2
    state, decel = fsm_controller.update_state(ttc=1.50, v_rel=2.0, dt=0.05)
    assert state == "AEB_LEVEL_3"
    assert decel == A_LEVEL_3


def test_transition_E_level_3_to_level_2(fsm_controller):
    """Test E: AEB_LEVEL_3 -> AEB_LEVEL_2 de-escalation after min_brake_duration."""
    # Enter LEVEL_3
    fsm_controller.update_state(ttc=1.50, v_rel=2.0, dt=0.05)  # t_state = 0.05
    fsm_controller.update_state(ttc=1.50, v_rel=2.0, dt=0.15)  # t_state = 0.20 >= min_brake_duration
    # De-escalate to LEVEL_2 (TTC > 1.75 and <= 2.25)
    state, decel = fsm_controller.update_state(ttc=2.00, v_rel=2.0, dt=0.05)
    assert state == "AEB_LEVEL_2"
    assert decel == A_LEVEL_2


def test_transition_F_level_2_to_level_1(fsm_controller):
    """Test F: AEB_LEVEL_2 -> AEB_LEVEL_1 de-escalation after min_brake_duration."""
    fsm_controller.update_state(ttc=2.10, v_rel=2.0, dt=0.05)
    fsm_controller.update_state(ttc=2.10, v_rel=2.0, dt=0.15)  # t_state = 0.20 >= min_brake_duration
    # De-escalate to LEVEL_1 (TTC > 2.25 and <= 3.00)
    state, decel = fsm_controller.update_state(ttc=2.50, v_rel=2.0, dt=0.05)
    assert state == "AEB_LEVEL_1"
    assert decel == A_LEVEL_1


def test_transition_G_level_1_to_warning(fsm_controller):
    """Test G: AEB_LEVEL_1 -> WARNING de-escalation after min_brake_duration."""
    fsm_controller.update_state(ttc=2.90, v_rel=2.0, dt=0.05)
    fsm_controller.update_state(ttc=2.90, v_rel=2.0, dt=0.15)  # t_state = 0.20 >= min_brake_duration
    # De-escalate to WARNING (3.00 < TTC <= 4.00)
    state, decel = fsm_controller.update_state(ttc=3.50, v_rel=2.0, dt=0.05)
    assert state == "WARNING"
    assert decel == 0.0


def test_transition_H_threat_clearance_to_safe(fsm_controller):
    """Test H: Retains active braking while t_state < min_brake_duration when TTC > 4.00, then releases to SAFE."""
    fsm_controller.update_state(ttc=2.90, v_rel=2.0, dt=0.05)  # Enters LEVEL_1 (t_state = 0.00 after transition)
    # TTC increases to 5.00 > 4.00 while v_rel > 0; dwell retention holds AEB_LEVEL_1
    state, decel = fsm_controller.update_state(ttc=5.00, v_rel=2.0, dt=0.05)  # t_state = 0.05 < 0.20
    assert state == "AEB_LEVEL_1"
    assert decel == A_LEVEL_1
    # Advance time to satisfy min_brake_duration (0.20s total)
    fsm_controller.update_state(ttc=5.00, v_rel=2.0, dt=0.10)  # t_state = 0.15 < 0.20
    state, decel = fsm_controller.update_state(ttc=5.00, v_rel=2.0, dt=0.05)  # t_state = 0.20 >= 0.20
    assert state == "SAFE"
    assert decel == 0.0


def test_transition_I_minimum_dwell_retention(fsm_controller):
    """Test I: Retains AEB_LEVEL_1 while t_state < min_brake_duration even if TTC increases."""
    fsm_controller.update_state(ttc=2.95, v_rel=2.0, dt=0.05)  # LEVEL_1, t_state = 0.05
    # Next step TTC increases to 3.25 (threat persists v_rel > 0, TTC <= 4.0), but t_state < 0.20
    state, decel = fsm_controller.update_state(ttc=3.25, v_rel=1.8, dt=0.05)  # t_state = 0.10
    assert state == "AEB_LEVEL_1"
    assert decel == A_LEVEL_1


def test_transition_J_boundary_crossing_consecutive_timesteps(fsm_controller):
    """Test J: Handles boundary crossings between consecutive timesteps cleanly."""
    fsm_controller.update_state(ttc=4.50, v_rel=2.0, dt=0.05)  # SAFE
    fsm_controller.update_state(ttc=3.50, v_rel=2.0, dt=0.05)  # WARNING
    fsm_controller.update_state(ttc=2.80, v_rel=2.0, dt=0.05)  # AEB_LEVEL_1
    assert fsm_controller.current_state == "AEB_LEVEL_1"


def test_transition_K_vrel_non_positive(fsm_controller):
    """Test K: V_rel <= 0 forces immediate Priority 1 release to SAFE regardless of dwell time."""
    fsm_controller.update_state(ttc=1.50, v_rel=5.0, dt=0.05)  # AEB_LEVEL_3, t_state = 0.05 < 0.20
    # Target accelerates away (V_rel <= 0)
    state, decel = fsm_controller.update_state(ttc=float("inf"), v_rel=-1.0, dt=0.05)
    assert state == "SAFE"
    assert decel == 0.0


def test_transition_L_zero_distance_positive_vrel(fsm_controller):
    """Test L: Separation distance zero with V_rel > 0 yields TTC = 0.0 and escalates/retains LEVEL_3."""
    ttc = AEBController.calculate_ttc(ego_speed=10.0, target_speed=0.0, distance=0.0)
    assert ttc == 0.0
    state, decel = fsm_controller.update_state(ttc=0.0, v_rel=10.0, dt=0.05)
    assert state == "AEB_LEVEL_3"
    assert decel == A_LEVEL_3


def test_transition_M_zero_ego_speed(fsm_controller):
    """Test M: Zero Ego speed against stationary target yields V_rel = 0 and SAFE state."""
    v_rel = AEBController.calculate_relative_velocity(ego_speed=0.0, target_speed=0.0)
    ttc = AEBController.calculate_ttc(ego_speed=0.0, target_speed=0.0, distance=5.0)
    assert v_rel == 0.0
    assert ttc == float("inf")
    state, decel = fsm_controller.update_state(ttc=ttc, v_rel=v_rel, dt=0.05)
    assert state == "SAFE"
    assert decel == 0.0


def test_transition_N_simultaneous_multi_boundary_escalation(fsm_controller):
    """Test N: Large TTC drop skips intermediate states directly to AEB_LEVEL_3."""
    state, decel = fsm_controller.update_state(ttc=1.20, v_rel=20.0, dt=0.05)
    assert state == "AEB_LEVEL_3"
    assert decel == A_LEVEL_3
