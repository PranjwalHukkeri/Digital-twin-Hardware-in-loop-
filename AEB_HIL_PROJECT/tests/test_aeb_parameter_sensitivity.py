"""
Focused Unit Tests for AEB Parameter Sensitivity Module.

Verifies:
1. ConfigurableAEBController parameter validation and ordering enforcement.
2. Behavioral equivalence of default ConfigurableAEBController to AEBController.
3. Integrity and ordering of all sensitivity configurations in the grid.
4. End-to-end execution of a reduced sensitivity sweep and statistical aggregation.
"""

import math
import pytest
from src.aeb_controller import AEBController
from analysis.aeb_parameter_sensitivity import (
    ConfigurableAEBController,
    SensitivityConfig,
    get_sensitivity_configurations,
    run_aeb_parameter_sensitivity,
    aggregate_sensitivity_results,
)
from scenarios.scenarios import CCRScenario


def test_configurable_aeb_controller_validation():
    """Verify strict validation and ordering checks on controller initialization."""
    # Valid default construction
    ctrl = ConfigurableAEBController()
    assert ctrl.ttc_warning == 4.00
    assert ctrl.a_level_3 == -6.00

    # Invalid TTC ordering: Warning <= L1
    with pytest.raises(ValueError, match="Invalid TTC ordering"):
        ConfigurableAEBController(ttc_warning=3.0, ttc_level_1=3.0)

    # Invalid TTC ordering: L1 <= L2
    with pytest.raises(ValueError, match="Invalid TTC ordering"):
        ConfigurableAEBController(ttc_level_1=2.0, ttc_level_2=2.25)

    # Invalid TTC ordering: L2 <= L3
    with pytest.raises(ValueError, match="Invalid TTC ordering"):
        ConfigurableAEBController(ttc_level_2=1.5, ttc_level_3=1.75)

    # Negative TTC
    with pytest.raises(ValueError, match="Invalid TTC ordering"):
        ConfigurableAEBController(ttc_level_3=-0.5)

    # Invalid deceleration ordering: positive deceleration
    with pytest.raises(ValueError, match="Invalid deceleration ordering"):
        ConfigurableAEBController(a_level_1=2.0)

    # Invalid deceleration ordering: |L1| >= |L2|
    with pytest.raises(ValueError, match="Invalid deceleration ordering"):
        ConfigurableAEBController(a_level_1=-4.0, a_level_2=-3.0)

    # Invalid deceleration ordering: |L2| >= |L3|
    with pytest.raises(ValueError, match="Invalid deceleration ordering"):
        ConfigurableAEBController(a_level_2=-6.0, a_level_3=-5.0)

    # Non-finite values
    with pytest.raises(ValueError, match="must be finite"):
        ConfigurableAEBController(ttc_warning=float("nan"))


def test_configurable_aeb_controller_baseline_equivalence():
    """Verify default ConfigurableAEBController produces identical outputs to AEBController."""
    base_ctrl = AEBController()
    cfg_ctrl = ConfigurableAEBController()

    test_ttc_values = [5.0, 4.0, 3.5, 3.0, 2.5, 2.25, 2.0, 1.75, 1.0, 0.5]
    v_rel = 10.0
    dt = 0.05

    for ttc in test_ttc_values:
        base_state, base_decel = base_ctrl.update_state(ttc, v_rel, dt)
        cfg_state, cfg_decel = cfg_ctrl.update_state(ttc, v_rel, dt)

        assert base_state == cfg_state, f"State mismatch at TTC={ttc}: {base_state} vs {cfg_state}"
        assert math.isclose(base_decel, cfg_decel, abs_tol=1e-6), (
            f"Decel mismatch at TTC={ttc}: {base_decel} vs {cfg_decel}"
        )


def test_sensitivity_configurations_validity():
    """Verify all 21 sensitivity configurations satisfy ordering and can instantiate controllers."""
    configs = get_sensitivity_configurations()
    assert len(configs) == 21, f"Expected 21 configurations, got {len(configs)}"

    names = set()
    for cfg in configs:
        assert cfg.name not in names, f"Duplicate configuration name: {cfg.name}"
        names.add(cfg.name)

        # TTC Ordering: Warning > L1 > L2 > L3 > 0
        assert cfg.ttc_warning > cfg.ttc_level_1 > cfg.ttc_level_2 > cfg.ttc_level_3 > 0.0, (
            f"TTC ordering violated in {cfg.name}"
        )

        # Deceleration Ordering: 0 > L1 > L2 > L3
        assert 0.0 > cfg.a_level_1 > cfg.a_level_2 > cfg.a_level_3, (
            f"Deceleration ordering violated in {cfg.name}"
        )

        # Controller instantiation succeeds
        ctrl = cfg.create_controller()
        assert isinstance(ctrl, ConfigurableAEBController)


def test_parameter_sensitivity_reduced_execution(tmp_path):
    """Verify end-to-end execution of a reduced sensitivity sweep with temporary outputs."""
    results_dir = str(tmp_path / "results")
    plots_dir = str(tmp_path / "plots")

    test_configs = [
        SensitivityConfig(
            name="Baseline",
            category="baseline",
            param_changed="None",
            baseline_value="Baseline",
            new_value="Baseline",
        ),
        SensitivityConfig(
            name="TTC_Early",
            category="comb_ttc",
            param_changed="TTC_all",
            baseline_value="4/3/2.25/1.75",
            new_value="4.5/3.5/2.5/2.0",
            ttc_warning=4.5,
            ttc_level_1=3.5,
            ttc_level_2=2.5,
            ttc_level_3=2.0,
        ),
    ]
    test_scenarios = [
        CCRScenario(name="CCR_Stationary_Target", initial_ego_speed=15.0, initial_distance=30.0)
    ]
    test_seeds = [42, 43]

    raw, summary = run_aeb_parameter_sensitivity(
        configs=test_configs,
        scenarios=test_scenarios,
        seeds=test_seeds,
        results_dir=results_dir,
        plots_dir=plots_dir,
    )

    assert len(raw) == 2 * 1 * 2  # 2 configs * 1 scenario * 2 seeds = 4 runs
    assert len(summary) == 2      # 2 configs * 1 scenario = 2 summary rows

    # Baseline delta for Baseline configuration should be 0.0
    base_summary = next(s for s in summary if s["config_name"] == "Baseline")
    assert base_summary["delta_min_distance_mean"] == 0.0
    assert base_summary["pct_min_distance_mean"] == 0.0

    # Early configuration should have positive delta min distance (greater safety buffer)
    early_summary = next(s for s in summary if s["config_name"] == "TTC_Early")
    assert early_summary["delta_min_distance_mean"] > 0.0
