"""
AEB Performance Evaluation Framework Module.

Provides a unified programmatic registry and evaluation interface for the
14 established AEB performance metrics across:
- Safety
- Timeliness
- Braking Effectiveness
- Controller Stability
- Robustness

Note:
This module provides evaluation structure and reporting for software simulation.
It distinguishes strictly between:
- WHAT WE MEASURE
- WHAT WE OBSERVE IN SIMULATION
- WHAT CONSTITUTES PASS/FAIL (with explicit TBD markings for unjustified thresholds)
"""

from typing import Dict, Any, Optional
from dataclasses import dataclass

from src.simulation import SimulationResult
from src.metrics import PerformanceEvaluator, AEBPerformanceMetrics
from analysis.sensitivity_analysis import (
    count_fsm_state_transitions,
    count_triplet_reversal_patterns,
    compute_active_braking_metrics,
)


@dataclass(frozen=True)
class MetricSpecification:
    """Metadata specification for an AEB evaluation metric."""
    name: str
    category: str  # Safety, Timeliness, Braking Effectiveness, Controller Stability, Robustness
    unit: str
    definition: str
    physical_meaning: str
    current_software_status: str
    current_threshold: str
    threshold_source_needed: str


METRIC_REGISTRY: Dict[str, MetricSpecification] = {
    "collision_status": MetricSpecification(
        name="collision_status",
        category="Safety",
        unit="bool",
        definition="raw_distance <= 0.0 and relative_velocity > 1e-4 m/s",
        physical_meaning="Physical contact/impact between ego and target vehicle with closing speed",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="False (Observed: 0 collisions in nominal & multi-seed sets)",
        threshold_source_needed="Test protocol (e.g., Euro NCAP / ISO 22839 / UNECE R152) & Engineering safety goal",
    ),
    "minimum_distance": MetricSpecification(
        name="minimum_distance",
        category="Safety",
        unit="m",
        definition="min(distance[k]) for k in 0..N-1",
        physical_meaning="Smallest clearance buffer between ego bumper and target during the scenario",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="TBD (Observed > 0.825m in simulations; minimum safe margin requires justification)",
        threshold_source_needed="Vehicle/platform specification & Safety margin standard",
    ),
    "aeb_activation_time": MetricSpecification(
        name="aeb_activation_time",
        category="Timeliness",
        unit="s",
        definition="Timestamp t of first sample where desired_deceleration < 0.0",
        physical_meaning="Elapsed simulation time before autonomous active braking interventions begin",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="TBD (Scenario-dependent; observed 0.00s for CCR/CCRm and 1.85s for CCRb)",
        threshold_source_needed="Engineering design assumption & Threat horizon definition",
    ),
    "ttc_at_activation": MetricSpecification(
        name="ttc_at_activation",
        category="Timeliness",
        unit="s",
        definition="distance[k_act] / relative_velocity[k_act] at activation sample k_act",
        physical_meaning="Estimated time remaining before collision at the instant active braking is triggered",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="TBD (Design threshold: <= 3.00s for LEVEL_1; observed 2.00s-2.99s across scenarios)",
        threshold_source_needed="Literature / Euro NCAP protocol & Human factors (warning before braking)",
    ),
    "max_braking_deceleration": MetricSpecification(
        name="max_braking_deceleration",
        category="Braking Effectiveness",
        unit="m/s²",
        definition="abs(min(desired_deceleration[k])) for k in 0..N-1",
        physical_meaning="Peak deceleration magnitude demanded by the FSM controller logic",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="TBD (Controller ceiling is 6.00 m/s² for LEVEL_3; requires vehicle limit validation)",
        threshold_source_needed="Vehicle/platform specification & Actuator authority limit",
    ),
    "max_actual_deceleration": MetricSpecification(
        name="max_actual_deceleration",
        category="Braking Effectiveness",
        unit="m/s²",
        definition="abs(min(actual_deceleration[k])) for k in 0..N-1",
        physical_meaning="Peak deceleration magnitude physically delivered by the brake actuator model",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="TBD (Matches commanded when tau=0; attenuated/delayed when tau>0)",
        threshold_source_needed="Vehicle/platform specification (Tire-road friction & Actuator hardware limit)",
    ),
    "ninety_percent_braking_response_time": MetricSpecification(
        name="ninety_percent_braking_response_time",
        category="Timeliness",
        unit="s",
        definition="Time interval from initial brake command until abs(actual_deceleration) >= 0.90 * abs(initial_commanded_deceleration)",
        physical_meaning="Delay in delivering 90% of the initial commanded step deceleration",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="TBD (Observed 0.00s at tau=0, scaling up to 0.46s at tau=0.30s)",
        threshold_source_needed="Actuator hardware specification & ISO 22839 response time requirements",
    ),
    "stopping_time": MetricSpecification(
        name="stopping_time",
        category="Braking Effectiveness",
        unit="s",
        definition="Timestamp t of first sample where math.isclose(ego_speed, 0.0, abs_tol=1e-5)",
        physical_meaning="Total elapsed scenario time required to bring ego vehicle to a full standstill",
        current_software_status="Evaluated dynamically per simulation run (None if vehicle does not stop)",
        current_threshold="TBD (Scenario-dependent; observed 2.50s-4.00s in stopped CCR runs)",
        threshold_source_needed="Test protocol & Initial velocity / deceleration profile",
    ),
    "stopping_distance": MetricSpecification(
        name="stopping_distance",
        category="Braking Effectiveness",
        unit="m",
        definition="sum(ego_speed[k] * dt[k]) for k in 0..N_stop-1",
        physical_meaning="Total ground displacement traveled by ego vehicle from scenario start to standstill",
        current_software_status="Evaluated via numerical trapezoidal/Euler integration over sample timestamps",
        current_threshold="TBD (Theoretical kinematic limit based on initial speed and average deceleration)",
        threshold_source_needed="Vehicle/platform specification & Kinematic stopping capability",
    ),
    "vehicle_stop_status": MetricSpecification(
        name="vehicle_stop_status",
        category="Safety",
        unit="string",
        definition="'STOPPED' if ego_speed reaches 0.0 m/s else 'NOT_STOPPED'",
        physical_meaning="Categorical indication of whether ego vehicle achieved complete standstill",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="TBD (Scenario-dependent: CCR requires stop; CCRm may clear threat without ego stopping)",
        threshold_source_needed="Test protocol definition (e.g., CCR vs CCRm requirements)",
    ),
    "final_relative_velocity": MetricSpecification(
        name="final_relative_velocity",
        category="Braking Effectiveness",
        unit="m/s",
        definition="ego_speed[N-1] - target_speed[N-1]",
        physical_meaning="Residual velocity difference between ego and target at scenario termination",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="TBD (Target is <= 0.0 m/s for full threat mitigation)",
        threshold_source_needed="Engineering design assumption & Protocol criteria",
    ),
    "min_relative_velocity": MetricSpecification(
        name="min_relative_velocity",
        category="Braking Effectiveness",
        unit="m/s",
        definition="min(relative_velocity[k]) for k in 0..N-1",
        physical_meaning="Most negative relative velocity achieved (indicates target moving away faster than ego)",
        current_software_status="Evaluated dynamically per simulation run",
        current_threshold="TBD (Informational metric)",
        threshold_source_needed="Engineering design assumption",
    ),
    "state_transition_count": MetricSpecification(
        name="state_transition_count",
        category="Controller Stability",
        unit="count",
        definition="sum(I(aeb_state[k] != aeb_state[k+1])) for k in 0..N-2",
        physical_meaning="Total number of discrete state changes executed by the FSM controller",
        current_software_status="Evaluated dynamically via count_fsm_state_transitions(result)",
        current_threshold="TBD (Observed 17-24 at zero noise, escalating to 35-46 under noise)",
        threshold_source_needed="Controller stability guidelines & Actuator cycling limits",
    ),
    "triplet_reversal_count": MetricSpecification(
        name="triplet_reversal_count",
        category="Controller Stability",
        unit="count",
        definition="sum(I(s[i] != s[i+1] and s[i] == s[i+2])) for i in 0..N-3, excluding SAFE -> X -> SAFE",
        physical_meaning="Number of 3-sample sliding FSM reversal patterns (structural boundary switching frequency)",
        current_software_status="Evaluated dynamically via count_triplet_reversal_patterns(result)",
        current_threshold="TBD (Observed 1-2 at zero noise due to peak turnaround; scales to 5-8 under 0.20m noise)",
        threshold_source_needed="Software robustness limits & FSM hysteresis design requirements",
    ),
}


def evaluate_comprehensive_performance(
    result: SimulationResult,
) -> Dict[str, Any]:
    """
    Compute full evaluation metrics and map them into the project evaluation framework.

    Args:
        result (SimulationResult): Simulation trajectory dataset.

    Returns:
        Dict[str, Any]: Comprehensive evaluation report dictionary.
    """
    raw_metrics: AEBPerformanceMetrics = PerformanceEvaluator.evaluate(result)
    state_transitions = count_fsm_state_transitions(result)
    triplet_reversals = count_triplet_reversal_patterns(result)
    episodes, active_duration = compute_active_braking_metrics(result)

    metrics_values: Dict[str, Any] = {
        "collision_status": raw_metrics.collision_status,
        "minimum_distance": raw_metrics.min_distance,
        "aeb_activation_time": raw_metrics.aeb_activation_time,
        "ttc_at_activation": raw_metrics.ttc_at_activation,
        "max_braking_deceleration": raw_metrics.max_braking_deceleration,
        "max_actual_deceleration": raw_metrics.max_actual_deceleration,
        "ninety_percent_braking_response_time": raw_metrics.ninety_percent_braking_response_time,
        "stopping_time": raw_metrics.stopping_time,
        "stopping_distance": raw_metrics.stopping_distance,
        "vehicle_stop_status": raw_metrics.vehicle_stop_status,
        "final_relative_velocity": raw_metrics.final_relative_velocity,
        "min_relative_velocity": raw_metrics.min_relative_velocity,
        "state_transition_count": state_transitions,
        "triplet_reversal_count": triplet_reversals,
        # Supplementary metrics
        "braking_episode_count": episodes,
        "total_braking_duration": active_duration,
        "final_ego_speed": raw_metrics.final_ego_speed,
    }

    report_entries = {}
    for key, spec in METRIC_REGISTRY.items():
        val = metrics_values.get(key)
        report_entries[key] = {
            "value": val,
            "unit": spec.unit,
            "category": spec.category,
            "definition": spec.definition,
            "physical_meaning": spec.physical_meaning,
            "current_threshold": spec.current_threshold,
            "threshold_source_needed": spec.threshold_source_needed,
            "status": "TBD" if "TBD" in spec.current_threshold else "ESTABLISHED",
        }

    return {
        "scenario_name": result.scenario_name,
        "metrics": report_entries,
        "summary_values": metrics_values,
    }
