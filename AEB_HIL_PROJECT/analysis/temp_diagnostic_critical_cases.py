"""
TEMPORARY DIAGNOSTIC SCRIPT: AEB FSM State Transitions Investigation
Path: /home/cim/Desktop/IUP26/AEB_HIL_PROJECT/analysis/temp_diagnostic_critical_cases.py

This is a temporary diagnostic tool designed to investigate AEB FSM state transitions,
chattering, and stopping feasibility across 5 specific low-speed operating cases:
1. CCR,  v_ego = 0.5 m/s, d0 = 1.5 m
2. CCRm, v_ego = 0.5 m/s, d0 = 1.5 m (evaluated across seeds, with special focus on seed 48)
3. CCRb, v_ego = 0.5 m/s, d0 = 1.5 m
4. CCRm, v_ego = 1.0 m/s, d0 = 1.5 m
5. CCRm, v_ego = 2.0 m/s, d0 = 3.0 m

Uses strictly the existing project APIs. Does not modify any production code.
"""

import math
from typing import List, Dict, Any, Optional
from src.simulation import SimulationEngine, SimulationResult, SimulationSample
from src.metrics import PerformanceEvaluator, AEBPerformanceMetrics
from scenarios.scenarios import (
    CCRScenario,
    CCRmScenario,
    CCRbScenario,
    Scenario,
)
from analysis.sensitivity_analysis import (
    count_fsm_state_transitions,
    count_triplet_reversal_patterns,
)


def build_scenario(scenario_type: str, ego_speed: float, distance: float) -> Scenario:
    """Instantiate standard project scenario using established target-speed relationships."""
    if scenario_type == "CCR":
        return CCRScenario(
            name=f"CCR_{ego_speed:.2f}mps_{distance:.1f}m",
            initial_ego_speed=ego_speed,
            initial_distance=distance,
            simulation_duration=10.0,
        )
    elif scenario_type == "CCRm":
        target_speed = 0.5 * ego_speed
        return CCRmScenario(
            name=f"CCRm_{ego_speed:.2f}mps_{distance:.1f}m",
            initial_ego_speed=ego_speed,
            initial_target_speed=target_speed,
            initial_distance=distance,
            simulation_duration=10.0,
        )
    elif scenario_type == "CCRb":
        target_speed = 0.5 * ego_speed
        return CCRbScenario(
            name=f"CCRb_{ego_speed:.2f}mps_{distance:.1f}m",
            initial_ego_speed=ego_speed,
            initial_target_speed=target_speed,
            initial_distance=distance,
            target_decel=-2.0,
            brake_start_time=1.0,
            simulation_duration=10.0,
        )
    else:
        raise ValueError(f"Unknown scenario type: {scenario_type}")


def extract_fsm_transitions(result: SimulationResult) -> List[Dict[str, Any]]:
    """Extract list of all FSM state changes with timestamp, from_state, to_state, and context."""
    transitions = []
    samples = result.samples
    if not samples:
        return transitions

    prev_state = samples[0].aeb_state
    for s in samples[1:]:
        if s.aeb_state != prev_state:
            transitions.append({
                "time": s.time,
                "from_state": prev_state,
                "to_state": s.aeb_state,
                "ego_speed": s.ego_speed,
                "target_speed": s.target_speed,
                "distance": s.distance,
                "v_rel": s.relative_velocity,
                "ttc": s.ttc,
                "actual_decel": s.actual_deceleration,
                "desired_decel": s.desired_deceleration,
            })
            prev_state = s.aeb_state
    return transitions


def run_case_diagnostic(
    case_name: str,
    scenario_type: str,
    ego_speed: float,
    distance: float,
    seed: int = 42,
    verbose_samples: bool = False,
) -> Dict[str, Any]:
    """Execute a single diagnostic run and print comprehensive telemetry."""
    scenario = build_scenario(scenario_type, ego_speed, distance)
    engine = SimulationEngine(
        dt=0.05,
        actuator_tau=0.10,
        distance_noise_std=0.05,
        distance_seed=seed,
        sensor_latency=0.0,
    )
    result = engine.run(scenario=scenario)
    metrics: AEBPerformanceMetrics = PerformanceEvaluator.evaluate(result)
    transitions = extract_fsm_transitions(result)
    triplets = count_triplet_reversal_patterns(result)

    samples = result.samples
    final_sample = samples[-1]
    final_aeb_state = final_sample.aeb_state
    final_distance = final_sample.distance
    min_ttc = min((s.ttc for s in samples if s.ttc is not None and not math.isinf(s.ttc)), default=float("inf"))

    # Stopping feasibility (scenario-aware):
    # CCR/CCRb requires vehicle standstill without collision.
    # CCRm requires speed matching or falling below target speed (v_rel <= 0.0) without collision.
    if scenario_type in {"CCR", "CCRb"}:
        stopping_feasible = (metrics.vehicle_stop_status == "STOPPED") and (not metrics.collision_status)
    elif scenario_type == "CCRm":
        stopping_feasible = (metrics.final_relative_velocity <= 0.0) and (not metrics.collision_status)
    else:
        stopping_feasible = (metrics.vehicle_stop_status == "STOPPED") and (not metrics.collision_status)

    data = {
        "case_name": case_name,
        "scenario_type": scenario_type,
        "ego_speed": ego_speed,
        "initial_distance": distance,
        "seed": seed,
        "min_distance": metrics.min_distance,
        "final_distance": final_distance,
        "final_ego_speed": metrics.final_ego_speed,
        "final_aeb_state": final_aeb_state,
        "vehicle_stop_status": metrics.vehicle_stop_status,
        "min_ttc": min_ttc,
        "transitions": transitions,
        "transition_count": len(transitions),
        "triplet_reversal_count": triplets,
        "first_brake_time": metrics.aeb_activation_time,
        "ttc_at_activation": metrics.ttc_at_activation,
        "max_brake_command": metrics.max_braking_deceleration,
        "max_actual_decel": metrics.max_actual_deceleration,
        "collision_status": metrics.collision_status,
        "stopping_feasible": stopping_feasible,
        "simulation_end_time": final_sample.time,
        "samples_count": len(samples),
    }

    print(f"\n{'='*75}")
    print(f"DIAGNOSTIC CASE: {case_name} (Seed {seed})")
    print(f"{'='*75}")
    print(f"  Scenario Type           : {scenario_type}")
    print(f"  Initial Ego Speed       : {ego_speed:.2f} m/s")
    print(f"  Initial Distance        : {distance:.2f} m")
    print(f"  Minimum Separation      : {metrics.min_distance:.4f} m")
    print(f"  Final Separation        : {final_distance:.4f} m")
    print(f"  Final Ego Speed         : {metrics.final_ego_speed:.4f} m/s")
    print(f"  Vehicle Stop Status     : {metrics.vehicle_stop_status}")
    print(f"  Final AEB State (FSM)   : {final_aeb_state}")
    print(f"  Minimum Finite TTC      : {min_ttc:.4f} s" if not math.isinf(min_ttc) else "  Minimum Finite TTC      : inf")
    print(f"  First Brake Time        : {metrics.aeb_activation_time} s" if metrics.aeb_activation_time is not None else "  First Brake Time        : None")
    print(f"  TTC at Activation       : {metrics.ttc_at_activation} s" if metrics.ttc_at_activation is not None else "  TTC at Activation       : None")
    print(f"  Max Commanded Brake     : {metrics.max_braking_deceleration:.2f} m/s²")
    print(f"  Max Actual Decel        : {metrics.max_actual_deceleration:.4f} m/s²")
    print(f"  Collision Occurred      : {metrics.collision_status}")
    print(f"  Stopping Feasible       : {stopping_feasible}")
    print(f"  Total FSM Transitions   : {len(transitions)}")
    print(f"  Triplet Reversals       : {triplets}")
    print(f"  Sim End Time / Duration : {final_sample.time:.2f} s ({len(samples)} samples)")

    print(f"\n  --- FSM STATE TRANSITIONS SEQUENCE ({len(transitions)} total) ---")
    if not transitions:
        print("    [No transitions occurred - remained in initial state]")
    else:
        for idx, tr in enumerate(transitions, 1):
            ttc_str = f"{tr['ttc']:.2f}s" if math.isfinite(tr['ttc']) else "inf"
            print(
                f"    {idx:02d}. t={tr['time']:0.2f}s: {tr['from_state']:11s} -> {tr['to_state']:11s} | "
                f"d={tr['distance']:0.3f}m, v_ego={tr['ego_speed']:0.3f}m/s, v_rel={tr['v_rel']:0.3f}m/s, "
                f"TTC={ttc_str:>6s}, actual_a={tr['actual_decel']:0.2f}m/s²"
            )

    if verbose_samples:
        print(f"\n  --- DETAILED TELEMETRY SAMPLES AROUND ACTIVATION ---")
        for s in samples:
            if s.desired_deceleration < 0.0 or s.aeb_state != "SAFE" or s.time <= 1.0:
                ttc_str = f"{s.ttc:.2f}s" if math.isfinite(s.ttc) else "inf"
                print(
                    f"    t={s.time:0.2f}s: FSM={s.aeb_state:11s} | d_true={s.distance:0.3f}m, "
                    f"d_meas={s.measured_distance:0.3f}m, v_ego={s.ego_speed:0.3f}m/s, "
                    f"v_tgt={s.target_speed:0.3f}m/s, v_rel={s.relative_velocity:0.3f}m/s, "
                    f"TTC={ttc_str:>6s}, a_cmd={s.desired_deceleration:0.1f}, a_act={s.actual_deceleration:0.2f}"
                )

    return data


def main():
    print("===========================================================================")
    print("      AEB FSM TRANSITION DIAGNOSTIC: 5 CRITICAL OPERATING CASES            ")
    print("===========================================================================")

    cases = [
        ("Case 1: CCR (Stationary)",  "CCR",  0.5, 1.5, 42),
        ("Case 2a: CCRm (Seed 42)",   "CCRm", 0.5, 1.5, 42),
        ("Case 2b: CCRm (Seed 48*)",  "CCRm", 0.5, 1.5, 48),  # Critical case noted by user
        ("Case 3: CCRb (Braking)",    "CCRb", 0.5, 1.5, 42),
        ("Case 4: CCRm (1.0 m/s)",    "CCRm", 1.0, 1.5, 42),
        ("Case 5: CCRm (2.0 m/s)",    "CCRm", 2.0, 3.0, 42),
    ]

    all_results = []
    for name, scen, v, d, s in cases:
        verbose = (name == "Case 2b: CCRm (Seed 48*)")
        res = run_case_diagnostic(name, scen, v, d, seed=s, verbose_samples=verbose)
        all_results.append(res)

    # Multi-seed sweep for Case 2 (CCRm 0.5 m/s, 1.5 m)
    print("\n" + "="*75)
    print("SEED SENSITIVITY CHECK: CCRm, v_ego = 0.5 m/s, d0 = 1.5 m (Seeds 42..51)")
    print("="*75)
    print(f"{'Seed':<6} {'MinDist':<10} {'FinalV':<10} {'FSM Level':<12} {'Trans':<8} {'Triplets':<10} {'StopStat':<12} {'ActTime':<10}")
    for seed in range(42, 52):
        s_res = run_case_diagnostic(
            f"CCRm_0.5mps_1.5m_seed{seed}", "CCRm", 0.5, 1.5, seed=seed, verbose_samples=False
        )
        print(
            f"{seed:<6} {s_res['min_distance']:<10.4f} {s_res['final_ego_speed']:<10.4f} "
            f"{s_res['final_aeb_state']:<12} {s_res['transition_count']:<8} "
            f"{s_res['triplet_reversal_count']:<10} {s_res['vehicle_stop_status']:<12} "
            f"{str(s_res['first_brake_time']):<10}"
        )

    print("\nDiagnostic complete.")


if __name__ == "__main__":
    main()
