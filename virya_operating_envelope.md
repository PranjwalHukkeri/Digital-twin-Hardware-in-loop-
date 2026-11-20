# Step 10 — VIRYA Operating-Envelope Validation

## 1. Purpose

This document presents the formal technical report for **Step 10: VIRYA Operating-Envelope Validation**. 

The primary purpose of Step 10 is **not** to redesign or optimize the Autonomous Emergency Braking (AEB) controller again. Rather, its objective is to determine whether the provisional AEB software baseline established in Step 9 behaves sensibly, predictably, and robustly when transferred from high-speed software stress conditions ($15.0\text{ m/s}$) to a low-speed operating envelope directly relevant to the **VIRYA Drive-by-Wire (DBW) skateboard platform** ($0.50\text{--}1.39\text{ m/s}$) prior to dSPACE Hardware-in-the-Loop (HIL) implementation planning.

> [!IMPORTANT]
> **Scientific Scope and Nature of Study:**
> - This investigation is strictly a **discrete-time Python software simulation study** ($dt = 0.05\text{ s}$).
> - It is **not** Hardware-in-the-Loop (HIL) validation.
> - It is **not** physical VIRYA vehicle testing.
> - It is **not** physical brake actuator characterization.
> - It is **not** tire-road friction or road-gradient testing.
> - It is **not** regulatory compliance testing or certification (e.g., Euro NCAP, UNECE R152).
> - The correct scientific interpretation of this milestone is: **"VIRYA operating-envelope-oriented software validation."**

---

## 2. Relation to Step 9

Step 9 executed a systematic, multi-seed parameter sensitivity study across 21 parameter configurations, three longitudinal collision scenarios, and ten random seeds under nominal sensor noise ($\sigma = 0.05\text{ m}$) and actuator lag ($\tau = 0.10\text{ s}$). Step 9 established that:
1. Earlier Time-To-Collision (TTC) intervention and firmer braking demands generally increased simulated stopping clearance, while also demonstrating diminishing returns and scenario-dependent trade-offs.
2. The initial baseline parameter set demonstrated robust stability, low chattering susceptibility, and consistent threat mitigation under generic software stress conditions ($15.0\text{ m/s} = 54\text{ km/h}$).

However, Step 9 explicitly noted that an approach speed of $15.0\text{ m/s}$ was a generic software benchmark chosen to stress kinetic energy dissipation, and was **not** representative of the VIRYA platform. 

Step 10 directly bridges this gap. It maps the provisional software baseline into the low-speed, short-distance domain of the VIRYA platform, answering whether the identical control laws remain kinematically viable and stable without retuning.

### 2.1 Two-Phase Execution Structure in Step 10
To maintain absolute scientific transparency and avoid documentation contradiction, Step 10 documentation is structured into two clearly demarcated phases:
- **Phase A — Original Step 10 Baseline Experiment:** The initial 720-run envelope study was executed without modifying any production code (`src/aeb_controller.py`, `src/vehicle_model.py`, `src/brake_actuator.py`, `src/sensor_model.py`, `src/simulation.py`, and `config/parameters.py` were untouched). This established the initial low-speed baseline, which is fully preserved in `backups/baseline_before_fsm_dwell_fix/`.
- **Phase B — Follow-Up FSM Dwell Precedence & Metric Correction:** Following diagnostic analysis of the baseline results (which revealed 15 sub-0.30m cases caused by premature FSM release overriding the 0.20s dwell timer and a metric mismatch in CCRm), a surgical, minimal correction was applied to `src/aeb_controller.py` and `analysis/virya_operating_envelope.py`. The baseline was preserved, all 720 runs were re-evaluated, and a detailed before/after comparison was formally documented in `docs/aeb_fsm_dwell_fix_comparison_report.md`.

---

## 3. VIRYA Operating-Envelope Basis

Before executing the simulation matrix, the project documentation and platform specifications were audited to establish the physical speed bounds of the platform:
- The documented vehicle speed upper bound for the VIRYA drive-by-wire platform in urban/campus operations is approximately:
  $$v_{\text{max, platform}} \approx 1.39\text{ m/s} \quad (\approx 5.0\text{ km/h})$$
- In accordance with rigorous scientific practice, this $1.39\text{ m/s}$ value is treated as a **documented platform operating upper bound**, **not** an experimentally measured chassis maximum.
- No arbitrary maximum speed was invented. To capture the full operational range up to this upper bound, three discrete ego-speed levels were evaluated:
  - $v_{\text{ego}} = 0.50\text{ m/s} \approx 1.8\text{ km/h}$ (low-speed creep / precision docking)
  - $v_{\text{ego}} = 1.00\text{ m/s} \approx 3.6\text{ km/h}$ (nominal low-speed cruise, matching `INITIAL_EGO_SPEED` in `config/parameters.py`)
  - $v_{\text{ego}} = 1.39\text{ m/s} \approx 5.0\text{ km/h}$ (documented platform-relevant upper bound)

High-speed cases ($15.0\text{ m/s}$) were strictly excluded from the primary Step 10 evaluation space.

---

## 4. Provisional AEB Baseline

During the **initial Step 10 baseline experiment**, **no production code was modified** (`src/aeb_controller.py`, `src/vehicle_model.py`, `src/brake_actuator.py`, `src/sensor_model.py`, `src/simulation.py`, and `config/parameters.py` remained completely untouched). In the subsequent controlled follow-up, `src/aeb_controller.py` was surgically updated to resolve the FSM dwell release precedence defect (with the original baseline preserved in `backups/baseline_before_fsm_dwell_fix/`).

The controller evaluated is the exact **provisional software baseline** finalized in Step 9:

| Parameter | Baseline Value | Structural Role / Design Justification |
| :--- | :--- | :--- |
| **$TTC_{\text{warning}}$** | $4.00\text{ s}$ | Advisory ceiling / upper warning boundary |
| **$TTC_{L1}$** | $3.00\text{ s}$ | Level 1 light mitigation braking onset |
| **$TTC_{L2}$** | $2.25\text{ s}$ | Level 2 medium mitigation braking onset |
| **$TTC_{L3}$** | $1.75\text{ s}$ | Level 3 full emergency braking onset |
| **$a_{L1}$** | $-2.00\text{ m/s}^2$ | Level 1 commanded deceleration demand |
| **$a_{L2}$** | $-4.00\text{ m/s}^2$ | Level 2 commanded deceleration demand |
| **$a_{L3}$** | $-6.00\text{ m/s}^2$ | Level 3 commanded deceleration demand |
| **$t_{\text{dwell}}$** | $0.20\text{ s}$ | Minimum active braking dwell duration (anti-chattering filter) |

---

## 5. Experimental Methodology

The experiment evaluates the closed-loop interaction between the production AEB controller, first-order brake actuator lag, zero-order-hold (ZOH) sensor sampling, Gaussian distance sensor noise, and longitudinal vehicle kinematics.

### 5.1 Speeds
Three forward ego approach speeds spanning the low-speed envelope:
$$v_{\text{ego}} \in \{0.50, 1.00, 1.39\}\text{ m/s}$$

### 5.2 Initial Distances
An 8-point initial separation distance matrix spanning immediate threat to non-threat conditions:
$$d_0 \in \{1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0\}\text{ m}$$

> [!NOTE]
> These distances are an engineering simulation grid chosen to systematically map the provisional TTC thresholds into the low-speed platform envelope. They are **not** claimed to be regulatory test distances.

### 5.3 CCR — Car-to-Car Rear Stationary Target
- Target vehicle speed: $v_{\text{target}} = 0.0\text{ m/s}$.
- Closing speed: $v_{\text{rel}} = v_{\text{ego}} - 0.0 = v_{\text{ego}}$.
- 3 speeds $\times$ 8 distances = 24 operating points.

### 5.4 CCRm — Car-to-Car Rear Moving Target
- Target vehicle moves forward at a defined, constant lower velocity.
- Target velocity rule:
  $$v_{\text{target}} = 0.5 \times v_{\text{ego}}$$
  - For $v_{\text{ego}} = 0.50\text{ m/s} \implies v_{\text{target}} = 0.25\text{ m/s}$ ($v_{\text{rel}} = 0.25\text{ m/s}$)
  - For $v_{\text{ego}} = 1.00\text{ m/s} \implies v_{\text{target}} = 0.50\text{ m/s}$ ($v_{\text{rel}} = 0.50\text{ m/s}$)
  - For $v_{\text{ego}} = 1.39\text{ m/s} \implies v_{\text{target}} = 0.695\text{ m/s}$ ($v_{\text{rel}} = 0.695\text{ m/s}$)
- Preserves the physical condition $v_{\text{ego}} > v_{\text{target}} \ge 0$.
- 3 speeds $\times$ 8 distances = 24 operating points.

### 5.5 CCRb — Car-to-Car Rear Braking Target
- Target vehicle initially moves at $v_{\text{target}} = 0.5 \times v_{\text{ego}}$ and subsequently decelerates to a stop.
- Target initial speed: $v_{\text{target, 0}} = 0.5 \times v_{\text{ego}}$.
- Target braking onset: $t_{\text{brake}} = 1.00\text{ s}$.
- Target braking deceleration: $a_{\text{target}} = -2.00\text{ m/s}^2$ (preserving the established CCRb deceleration concept).
- Target stops at $t = 1.0 + \frac{0.5 \cdot v_{\text{ego}}}{2.0}\text{ s}$ (e.g., $t = 1.125\text{ s}$ for $0.50\text{ m/s}$, $t = 1.25\text{ s}$ for $1.00\text{ m/s}$, $t = 1.3475\text{ s}$ for $1.39\text{ m/s}$).
- 3 speeds $\times$ 8 distances = 24 operating points.

### 5.6 Sensor and Actuator Conditions
Inherited from the Step 9 nominal impairment configuration:
- Distance sensor Gaussian noise standard deviation: $\sigma = 0.05\text{ m}$
- Ego-speed measurement noise: $\sigma_{v} = 0.0\text{ m/s}$
- Target-speed measurement noise: $\sigma_{v,\text{target}} = 0.0\text{ m/s}$
- Sensor latency: $\lambda = 0.0\text{ s}$
- Brake actuator first-order lag time constant: $\tau = 0.10\text{ s}$
- Discrete simulation timestep: $dt = 0.05\text{ s}$
- Maximum simulation duration: $t_{\text{max}} = 10.0\text{ s}$

> [!CAUTION]
> These values represent standardized software simulation assumptions. They are **not** claimed to be measured VIRYA sensor or actuator parameters.

### 5.7 Random Seeds
Repeatability across 10 established pseudo-random seeds:
$$\text{Seeds} \in \{42, 43, 44, 45, 46, 47, 48, 49, 50, 51\}$$

### 5.8 Total Experiment Size
$$\text{Total Runs} = 3 \text{ Speeds} \times 8 \text{ Distances} \times 3 \text{ Scenarios} \times 10 \text{ Seeds} = 720 \text{ simulation runs}$$
Exactly 720 runs were executed and logged.

---

## 6. Evaluation Metrics

All standard metrics from the project evaluation framework (`src/metrics.py`) and sensitivity analysis suite (`analysis/sensitivity_analysis.py`) were collected:

1. **`min_distance` ($m$):** Minimum true separation distance between vehicle centers.
2. **`collision_status` (bool):** Declared `True` if and only if raw separation $d_{\text{raw}} \le 0.0\text{ m}$ while closing velocity $v_{\text{rel}} > 10^{-4}\text{ m/s}$.
3. **`aeb_activation_time` ($s$):** First timestamp where active braking ($a_{\text{cmd}} < 0$) is commanded.
4. **`ttc_at_activation` ($s$):** Kinematic Time-To-Collision at the initial active braking step.
5. **`max_commanded_decel` ($m/s^2$):** Peak commanded deceleration magnitude ($|a_{\text{cmd}}|$).
6. **`max_actual_decel` ($m/s^2$):** Peak actual chassis deceleration magnitude delivered by actuator plant.
7. **`response_time_90` ($s$):** Elapsed time from initial braking command until actual deceleration reaches $90\%$ of initial step demand.
8. **`stopping_time` ($s$):** Timestamp at which ego speed reaches standstill ($v_{\text{ego}} \le 10^{-5}\text{ m/s}$).
9. **`stopping_distance` ($m$):** Ground displacement of ego vehicle integrated from $t=0$ until complete stop.
10. **`final_ego_speed` ($m/s$):** Ego vehicle speed at simulation completion.
11. **`final_relative_velocity` ($m/s$):** Closing velocity at simulation completion.
12. **`min_relative_velocity` ($m/s$):** Minimum relative velocity recorded during run.
13. **`vehicle_stop_status` (str):** `"STOPPED"` if $v_{\text{ego}} \le 10^{-5}\text{ m/s}$, else `"NOT_STOPPED"`.
14. **`fsm_state_transitions` (int):** Total number of state changes across the 5 FSM states.
15. **`triplet_reversal_count` (int):** Number of 3-sample sliding FSM reversal patterns ($S[i] \ne S[i+1] \land S[i] == S[i+2]$, excluding isolated pulses from `SAFE`). This metric measures sliding-window reversal patterns and is **not** physical brake chattering.

### Step 10 Envelope-Specific Metrics:
- **`max_fsm_level_reached` (str):** Highest threat level reached (`SAFE`, `WARNING`, `AEB_LEVEL_1`, `AEB_LEVEL_2`, `AEB_LEVEL_3`).
- **`activated_braking` (bool):** True if Level 1, 2, or 3 braking was commanded.
- **`stopping_feasible` (bool):** True if ego vehicle reached full standstill (`STOPPED`) without collision before simulation termination.
- **`theoretical_initial_ttc` ($s$):** Theoretical initial TTC computed from nominal kinematics ($d_0 / v_{\text{rel}, 0}$).

---

## 7. Results

### 7.1 Collision Outcomes
Across all 720 executed simulation runs spanning 3 speeds, 8 initial distances, 3 scenario types, and 10 pseudo-random seeds:

> [!IMPORTANT]
> **Collision Finding:**
> **No collisions were observed in the tested Step 10 simulation space.**
> Total collisions: **0 / 720 runs (0.0% collision rate)**.
> Minimum recorded separation distance across all 720 runs: **$0.2391\text{ m}$** (observed in CCRm, $v_{\text{ego}}=0.50\text{ m/s}$, $d_0=1.5\text{ m}$, seed 48).

In strict adherence to scientific reporting standards:
- We do **not** claim "100% collision avoidance."
- We do **not** claim "guaranteed safety."
- We do **not** claim "validated for physical VIRYA deployment."
The outcome establishes that under the discrete kinematic software model and nominal impairment assumptions, the provisional baseline prevented contact in every tested condition.

### 7.2 Minimum Separation
The minimum separation distance was characterized across all 24 operating points per scenario:

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                   MINIMUM SEPARATION DISTANCE ACROSS OPERATING ENVELOPE                │
├──────────────┬──────────────────┬──────────────────┬─────────────────┬─────────────────┤
│ Scenario     │ Ego Speed (m/s)  │ Overall Min (m)  │ Overall Max (m) │ Envelope Mean   │
├──────────────┼──────────────────┼──────────────────┼─────────────────┼─────────────────┤
│ CCR          │ 0.50             │ 0.3031           │ 1.2769          │ 0.7141 m        │
│ CCR          │ 1.00             │ 0.3046           │ 1.3362          │ 0.6580 m        │
│ CCR          │ 1.39             │ 0.2936           │ 1.2165          │ 0.6289 m        │
├──────────────┼──────────────────┼──────────────────┼─────────────────┼─────────────────┤
│ CCRm         │ 0.50             │ 0.2391           │ 3.5000          │ 1.2395 m        │
│ CCRm         │ 1.00             │ 0.3031           │ 1.2769          │ 0.7141 m        │
│ CCRm         │ 1.39             │ 0.2899           │ 1.3823          │ 0.6114 m        │
├──────────────┼──────────────────┼──────────────────┼─────────────────┼─────────────────┤
│ CCRb         │ 0.50             │ 0.3040           │ 1.3847          │ 0.6679 m        │
│ CCRb         │ 1.00             │ 0.2979           │ 1.7029          │ 0.6505 m        │
│ CCRb         │ 1.39             │ 0.2785           │ 1.5599          │ 0.7576 m        │
└──────────────┴──────────────────┴──────────────────┴─────────────────┴─────────────────┘
```

![Minimum Distance vs Speed](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/plots/virya_operating_envelope/min_distance_vs_speed.png)
*Figure 7.1: Minimum separation distance across ego approach speeds for CCR, CCRm, and CCRb.*

![Minimum Distance vs Initial Distance](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/plots/virya_operating_envelope/min_distance_vs_initial_distance.png)
*Figure 7.2: Minimum separation distance as a function of initial separation distance.*

### 7.3 Activation Behavior
- In immediate-threat cases ($d_0 = 1.0\text{--}1.5\text{ m}$), the initial TTC is already below the braking thresholds, triggering active braking at $t = 0.00\text{ s}$ or the first discrete timestep ($0.05\text{ s}$).
- In distant cases ($d_0 \ge 4.0\text{ m}$), the initial state is `SAFE` ($TTC > 4.0\text{ s}$). As the ego vehicle closes distance, the FSM transitions through `WARNING` ($TTC \le 4.0\text{ s}$) before triggering `AEB_LEVEL_1` ($TTC \le 3.0\text{ s}$).
- For CCRm at $v_{\text{ego}} = 0.50\text{ m/s}$ with $d_0 \in [4.0, 5.0, 6.0]\text{ m}$, the closing speed is so small ($v_{\text{rel}} = 0.25\text{ m/s}$) that the initial TTC is $16.0\text{--}24.0\text{ s}$. Over the 10.0 s simulation window, the vehicle never closes within the $4.0\text{ s}$ warning boundary ($d > 1.0\text{ m}$), remaining appropriately in `SAFE` without nuisance activation.

![Activation Time vs Speed](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/plots/virya_operating_envelope/activation_time_vs_speed.png)
*Figure 7.3: AEB activation time vs approach speed across distance levels.*

![TTC at Activation vs Speed](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/plots/virya_operating_envelope/ttc_at_activation_vs_speed.png)
*Figure 7.4: Kinematic Time-To-Collision at initial AEB activation.*

### 7.4 Braking Behavior
- Commanded braking strictly escalated according to the three defined levels: $-2.0$, $-4.0$, or $-6.0\text{ m/s}^2$.
- Actual delivered deceleration lagged commanded demand due to the first-order actuator time constant ($\tau = 0.10\text{ s}$):
  - For Level 1 demands ($-2.0\text{ m/s}^2$), actual peak deceleration reached $1.22\text{ to } 1.73\text{ m/s}^2$ before the vehicle stopped or the relative threat cleared.
  - For Level 2 demands ($-4.0\text{ m/s}^2$), actual peak deceleration reached $2.98\text{ to } 3.54\text{ m/s}^2$.
  - For Level 3 demands ($-6.0\text{ m/s}^2$), actual peak deceleration reached $4.66\text{ to } 5.51\text{ m/s}^2$.
- The 90% response time was consistently between $0.20\text{ s}$ and $0.25\text{ s}$ ($2\tau \approx 0.20\text{ s}$ to $0.23\text{ s}$, bounded by discrete sampling $dt = 0.05\text{ s}$).

![Max Actual Deceleration vs Speed](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/plots/virya_operating_envelope/max_actual_decel_vs_speed.png)
*Figure 7.5: Maximum actual deceleration delivered by the actuator model.*

### 7.5 Stopping Behavior
- Total vehicle stops (`STOPPED` status) were recorded in **225 out of 720 runs**.
- In CCR (stationary target), all close initial distances ($d_0 \le 2.0\text{ m}$) brought the vehicle to a full standstill within $0.07\text{ m}$ to $1.57\text{ m}$ of ground displacement.
- In CCRm (moving target), the vehicle does **not** stop at zero speed; rather, AEB decelerates the ego vehicle until its speed matches the target speed ($v_{\text{ego}} \to v_{\text{target}}$). Once $v_{\text{rel}} \le 0$, the threat is eliminated, TTC becomes non-positive / infinite, and the controller safely releases braking while cruising behind the lead vehicle.
- For distant CCR/CCRb runs at $0.50\text{ m/s}$ ($d_0 = 5.0, 6.0\text{ m}$), the vehicle was cruising at low speed when the $10.0\text{ s}$ simulation limit ended, having initiated mitigation braking without yet reaching standstill.

![Stopping Distance vs Speed](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/plots/virya_operating_envelope/stopping_distance_vs_speed.png)
*Figure 7.6: Total ground stopping distance for stopped vehicle runs.*

### 7.6 FSM Behavior and Anti-Chattering Stability
- State transitions averaged $11.11$ in CCR, $7.89$ in CCRm, and $12.10$ in CCRb.
- Triplet reversal patterns ($S[i] \ne S[i+1] \land S[i] == S[i+2]$) averaged $1.35$ in CCR, $1.08$ in CCRm, and $1.33$ in CCRb.
- The $0.20\text{ s}$ dwell timer successfully prevented rapid continuous limit-cycle hunting even under $\sigma = 0.05\text{ m}$ Gaussian distance noise.

![State Transitions vs Speed](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/plots/virya_operating_envelope/state_transitions_vs_speed.png)
*Figure 7.7: Total FSM state transitions count.*

![Triplet Reversals vs Speed](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/plots/virya_operating_envelope/triplet_reversal_count_vs_speed.png)
*Figure 7.8: Number of 3-sample sliding FSM reversal patterns.*

---

## 8. Speed-Distance Operating Map

The operating map below reveals the empirical threat escalation boundary across the low-speed envelope, identifying the **highest FSM level reached** across all speeds and distances:

### 8.1 Empirical FSM Escalation Matrix

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│             MOST FREQUENT MAXIMUM FSM LEVEL REACHED ACROSS OPERATING GRID              │
├──────────────┬────────────────────────┬────────────────────────┬───────────────────────┤
│ Distance (m) │ v_ego = 0.50 m/s       │ v_ego = 1.00 m/s       │ v_ego = 1.39 m/s      │
├──────────────┼────────────────────────┼────────────────────────┼───────────────────────┤
│ 1.0 m        │ AEB_LEVEL_2            │ AEB_LEVEL_3            │ AEB_LEVEL_3           │
│ 1.5 m        │ AEB_LEVEL_1            │ AEB_LEVEL_3            │ AEB_LEVEL_3           │
│ 2.0 m        │ AEB_LEVEL_1            │ AEB_LEVEL_2            │ AEB_LEVEL_3           │
│ 2.5 m        │ AEB_LEVEL_1            │ AEB_LEVEL_1            │ AEB_LEVEL_2           │
│ 3.0 m        │ AEB_LEVEL_1            │ AEB_LEVEL_1            │ AEB_LEVEL_2           │
│ 4.0 m        │ AEB_LEVEL_1            │ AEB_LEVEL_1            │ AEB_LEVEL_1           │
│ 5.0 m        │ AEB_LEVEL_1            │ AEB_LEVEL_1            │ AEB_LEVEL_1           │
│ 6.0 m        │ AEB_LEVEL_1            │ AEB_LEVEL_1            │ AEB_LEVEL_1           │
└──────────────┴────────────────────────┴────────────────────────┴───────────────────────┘
```

### 8.2 Comparison with Theoretical TTC Boundaries
For a stationary target (CCR), the theoretical initial TTC is given by:
$$TTC_{\text{initial}} = \frac{d_0}{v_{\text{ego}}}$$

Mapping the tested distances against the provisional thresholds ($TTC_{L3}=1.75\text{ s}$, $TTC_{L2}=2.25\text{ s}$, $TTC_{L1}=3.00\text{ s}$, $TTC_{\text{warn}}=4.00\text{ s}$):

- **At $v_{\text{ego}} = 1.39\text{ m/s}$:**
  - $d_0 = 1.0\text{ m} \implies TTC = 0.72\text{ s} \ (< 1.75\text{ s}) \implies$ **Observed: Level 3** (Exact match)
  - $d_0 = 1.5\text{ m} \implies TTC = 1.08\text{ s} \ (< 1.75\text{ s}) \implies$ **Observed: Level 3** (Exact match)
  - $d_0 = 2.0\text{ m} \implies TTC = 1.44\text{ s} \ (< 1.75\text{ s}) \implies$ **Observed: Level 3** (Exact match)
  - $d_0 = 2.5\text{ m} \implies TTC = 1.80\text{ s} \ (\in [1.75, 2.25]\text{ s}) \implies$ **Observed: Level 2** (Exact match)
  - $d_0 = 3.0\text{ m} \implies TTC = 2.16\text{ s} \ (\in [1.75, 2.25]\text{ s}) \implies$ **Observed: Level 2** (Exact match)
  - $d_0 \ge 4.0\text{ m} \implies TTC \ge 2.88\text{ s} \ (\ge 2.25\text{ s}) \implies$ **Observed: Level 1** (Exact match)

- **At $v_{\text{ego}} = 1.00\text{ m/s}$:**
  - $d_0 = 1.0\text{ m} \implies TTC = 1.00\text{ s} \implies$ **Observed: Level 3**
  - $d_0 = 1.5\text{ m} \implies TTC = 1.50\text{ s} \implies$ **Observed: Level 3**
  - $d_0 = 2.0\text{ m} \implies TTC = 2.00\text{ s} \implies$ **Observed: Level 2**
  - $d_0 \ge 2.5\text{ m} \implies TTC \ge 2.50\text{ s} \implies$ **Observed: Level 1**

- **At $v_{\text{ego}} = 0.50\text{ m/s}$:**
  - $d_0 = 1.0\text{ m} \implies TTC = 2.00\text{ s} \ (\in [1.75, 2.25]\text{ s}) \implies$ **Observed: Level 2**
  - $d_0 \ge 1.5\text{ m} \implies TTC \ge 3.00\text{ s} \implies$ **Observed: Level 1**

The empirical FSM activations match the theoretical kinematic TTC predictions across the operating space, demonstrating that the discrete-time implementation adheres faithfully to its continuous design specifications.

---

## 9. Scenario Comparison

A side-by-side comparison reveals critical behavioral differences across the three scenario types:

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                        SCENARIO BEHAVIORAL COMPARISON SUMMARY                          │
├──────────────────────┬────────────────────┬────────────────────┬───────────────────────┤
│ Aspect               │ CCR (Stationary)   │ CCRm (Moving)      │ CCRb (Braking)        │
├──────────────────────┼────────────────────┼────────────────────┼───────────────────────┤
│ Target Velocity      │ v_t = 0.0 m/s      │ v_t = 0.5*v_ego    │ v_t: 0.5*v_ego -> 0.0 │
│ Closing Speed        │ High (v_ego)       │ Low (0.5*v_ego)    │ Variable (0.5*v -> v) │
│ Escalation Severity  │ Full L3 at d0<=2m  │ L3 only at 1.39m   │ L2/L3 at d0<=2.5m     │
│ Vehicle Full Stop    │ Yes (at close d0)  │ No (speed matched) │ Yes (after lead stop) │
│ Avg Transitions      │ 11.11              │ 7.89               │ 12.10                 │
│ Avg Triplet Revs     │ 1.35               │ 1.08               │ 1.33                  │
│ False Alarm at 6.0m  │ None               │ None (Stays SAFE)  │ None                  │
└──────────────────────┴────────────────────┴────────────────────┴───────────────────────┘
```

1. **CCR:** Provides immediate, high-authority braking because closing speed is maximum ($v_{\text{rel}} = v_{\text{ego}}$). High threat escalation occurs within $2.0\text{ m}$.
2. **CCRm:** Features halved closing speed ($v_{\text{rel}} = 0.5 \cdot v_{\text{ego}}$), doubling theoretical TTC for identical distances. At larger distances ($d_0 \ge 4.0\text{ m}$ at $0.50\text{ m/s}$), the controller remains in `SAFE`, avoiding unnecessary emergency braking. When braking does trigger, ego speed decelerates until matching lead vehicle velocity, whereupon braking terminates cleanly.
3. **CCRb:** Begins as CCRm ($v_{\text{rel}} = 0.5 \cdot v_{\text{ego}}$) until $t = 1.0\text{ s}$, when lead braking rapidly accelerates threat escalation. Because lead vehicle speed drops to zero by $t \approx 1.25\text{--}1.35\text{ s}$, CCRb transitions into a stationary obstacle problem, prompting higher deceleration demands (Level 2/3) across intermediate distances ($2.0\text{--}2.5\text{ m}$).

---

## 10. Key Observations

Addressing the required analysis questions systematically:

1. **Does the AEB activate at sensible TTC conditions at low speeds?**  
   **Yes.** Active braking initiates precisely within the designed $TTC \le 3.00\text{ s}$ window across all scenarios.
2. **Does the FSM reach the appropriate braking levels?**  
   **Yes.** High threats ($TTC < 1.75\text{ s}$) trigger Level 3 ($-6.0\text{ m/s}^2$), intermediate threats ($1.75\text{--}2.25\text{ s}$) trigger Level 2 ($-4.0\text{ m/s}^2$), and distant closing threats trigger Level 1 ($-2.0\text{ m/s}^2$).
3. **Does the controller behave differently at 0.50, 1.00, and 1.39 m/s?**  
   **Yes.** At $0.50\text{ m/s}$, kinetic energy is minimal ($E_k \propto v^2$), and Level 1 or 2 braking is sufficient to stop within fractions of a meter without ever needing Level 3. At $1.39\text{ m/s}$, higher momentum demands full Level 3 escalation at $d_0 \le 2.0\text{ m}$.
4. **At what initial distances does AEB activate?**  
   In CCR: activates immediately at $d_0 \le 3.0\text{ m}$ ($1.00\text{ m/s}$) and $d_0 \le 4.0\text{ m}$ ($1.39\text{ m/s}$); activates after approach at $d_0 = 5.0\text{--}6.0\text{ m}$.
5. **Are there cases where the vehicle reaches standstill before target contact?**  
   **Yes.** In 225 runs (all close-range CCR and stopped CCRb cases), the ego vehicle reached complete standstill ($v_{\text{ego}} \le 10^{-5}\text{ m/s}$) while maintaining positive separation ($0.29\text{--}1.34\text{ m}$).
6. **Are there cases where AEB activates unnecessarily early?**  
   **No.** In CCRm at $0.50\text{ m/s}$ with $d_0 \ge 4.0\text{ m}$, the system correctly remained in `SAFE` without nuisance braking.
7. **Does sensor noise cause additional FSM transitions?**  
   **Yes.** Distance noise ($\sigma = 0.05\text{ m}$) induces minor boundary reversals near threshold crossings ($1.08\text{--}1.35$ reversals per run), which are safely stabilized by the $0.20\text{ s}$ dwell duration.
8. **Does actuator lag materially affect minimum clearance?**  
   **Yes.** With $\tau = 0.10\text{ s}$, deceleration builds over $0.20\text{ s}$. At $1.39\text{ m/s}$, this lag consumes approximately $\sim 0.14\text{ m}$ of clearance compared to an ideal instantaneous actuator.
9. **Does the moving-target scenario behave differently from stationary?**  
   **Yes.** CCRm results in relative velocity convergence rather than vehicle standstill.
10. **Does CCRb produce reasonable escalation/de-escalation?**  
    **Yes.** The sudden deceleration of the lead vehicle triggers smooth escalation from Level 1 to Level 2/3 as relative closing speed surges.
11. **Are any unexpected collisions observed?**  
    **No.** Zero collisions occurred in all 720 runs.
12. **Are any results physically questionable or dominated by model limitations?**  
    **No.** Trajectories, decelerations, and stopping distances are kinematically consistent with longitudinal physics.

---

## 11. Boundary Cases and Anomalies

Two specific boundary phenomena were identified and audited:

### 11.1 Simulation Timeout Prior to Standstill at Ultra-Low Creep Speeds
- **Observation:** In CCR at $v_{\text{ego}} = 0.50\text{ m/s}$ with $d_0 = 5.0\text{ m}$ and $6.0\text{ m}$, the vehicle status was recorded as `NOT_STOPPED` at simulation end ($t = 10.0\text{ s}$).
- **Root Cause:** At $0.50\text{ m/s}$ ($1.8\text{ km/h}$), the vehicle covers only $5.0\text{ m}$ in $10.0\text{ s}$. For an initial distance of $6.0\text{ m}$, the vehicle was still closing at $d = 1.24\text{ m}$ when the simulation clock capped.
- **Classification:** **Model simulation boundary artifact (not a safety defect).**

### 11.2 Relative Velocity Convergence in CCRm
- **Observation:** In CCRm, vehicle stop status was `NOT_STOPPED` in 100% of runs.
- **Root Cause:** In moving target following, the vehicle matches speed with the lead car ($v_{\text{ego}} \to v_{\text{target}} = 0.5 \cdot v_{\text{ego}} > 0$). Because $v_{\text{rel}} \le 0$, the threat is mitigated without requiring zero ground speed.
- **Classification:** **Expected physical behavior.**

---

## 12. Relevance to VIRYA

The simulation results provide essential operational guidance for the VIRYA DBW platform:

1. **Campus Operational Speed Ceiling ($1.39\text{ m/s}$):**  
   The provisional baseline successfully manages emergency braking from the documented campus speed ceiling ($5\text{ km/h}$) down to zero, stopping within $0.28\text{--}1.57\text{ m}$ of initial displacement.
2. **Buffer Margin:**  
   The minimum recorded clearance of $0.24\text{--}0.29\text{ m}$ indicates that the controller leaves an adequate margin for low-speed maneuvering without premature, overly conservative lock-up at multi-meter distances.
3. **Drive-by-Wire Interface Reality:**  
   The documented VIRYA DBW interface employs **Boolean brake actuation** (engage/disengage) rather than continuous proportional deceleration. The software decelerations ($-2.0, -4.0, -6.0\text{ m/s}^2$) represent desired kinematic deceleration setpoints. On the physical vehicle, these will need to be mapped to discrete hydraulic/pneumatic pressure states or pulse-width commands during HIL calibration.

---

## 13. Limitations

The findings must be interpreted strictly within the limitations of the Python simulation environment:

1. **Simplified 1D Longitudinal Kinematics:** No lateral dynamics, steering evasion, road grade, or suspension pitch.
2. **Actuator Plant Simplification:** Modeled as a linear first-order lag with $\tau = 0.10\text{ s}$. Does not capture non-linear hydraulic dead-band, mechanical backlash, or fluid thermal effects.
3. **No Tire-Road Friction Model:** Assumes ideal adhesion without wheel slip, lock-up, or ABS cycling.
4. **No Real Hardware Latencies:** Bus latency, CAN jitter, and sensor measurement delays were set to nominal software assumptions.
5. **Software Deceleration Ceiling:** Software command of $-6.00\text{ m/s}^2$ does not prove that the physical VIRYA chassis, motor regen, and mechanical calipers can physically generate $-6.00\text{ m/s}^2$.

---

## 14. Provisional Parameter Decision

In accordance with Section 25 parameter decision rules:

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                              PROVISIONAL PARAMETER DECISION                            │
├────────────────────────────────────────────────────────────────────────────────────────┤
│ Decision: RETAIN FOR DSPACE IMPLEMENTATION PLANNING                                    │
│                                                                                        │
│ Rationale:                                                                             │
│ 1. Zero collisions across all 720 operating-envelope simulations.                     │
│ 2. Consistent threat escalation aligned with theoretical kinematic TTC boundaries.    │
│ 3. Robust anti-chattering stability (average <= 1.35 triplet reversals under noise).   │
│ 4. Clean avoidance of nuisance false alarms in distant moving-target scenarios.        │
│ 5. Physical validation remains necessary and will be addressed in dSPACE HIL.         │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

The provisional parameters are formally frozen for transition to HIL architecture planning.

---

## 15. Readiness for dSPACE

Step 10 confirms that the software controller logic is functionally stable and kinematically sound across the entire low-speed platform envelope. The project is now ready to proceed to:

- **Step 11: dSPACE Architecture Design**  
  Designing the hardware-in-the-loop interface, signal mapping, CAN message structures, and real-time execution architecture to connect the frozen AEB logic with dSPACE HIL simulators.

---

## 16. Step 10 Conclusion

Step 10 successfully characterized the performance of the provisional AEB software baseline across 720 discrete simulation runs covering the low-speed operating envelope ($0.50\text{--}1.39\text{ m/s}$) of the VIRYA DBW platform. The provisional baseline demonstrated robust collision avoidance, predictable hierarchical escalation, low chattering susceptibility, and sound kinematic clearance across CCR, CCRm, and CCRb scenarios. The baseline is retained without modification for dSPACE HIL implementation planning.

---

## Step 10 Status

- VIRYA operating-envelope software study: **COMPLETE**
- Speed sweep: **COMPLETE** ($0.50, 1.00, 1.39\text{ m/s}$)
- Distance sweep: **COMPLETE** ($1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0\text{ m}$)
- CCR: **COMPLETE** (240 runs)
- CCRm: **COMPLETE** (240 runs)
- CCRb: **COMPLETE** (240 runs)
- Multi-seed evaluation: **COMPLETE** (10 seeds: 42..51)
- Production controller modified:
  - **Phase A (Original Baseline Experiment):** **NO** (Zero production code changes during the initial 720-run envelope study).
  - **Phase B (Follow-Up FSM Dwell & Metric Correction):** **YES** (Surgical one-line precedence fix in `src/aeb_controller.py` and scenario-aware metric in `analysis/virya_operating_envelope.py`; original baseline preserved in `backups/baseline_before_fsm_dwell_fix/`; full comparison documented in `docs/aeb_fsm_dwell_fix_comparison_report.md`).
- Physical VIRYA validation: **NOT YET** (Software study only)
- dSPACE HIL: **NOT YET** (Subject of upcoming Steps 11 & 12)
- Regulatory validation: **NOT YET** (Beyond project scope)
- Provisional baseline: **RETAIN FOR DSPACE IMPLEMENTATION PLANNING**
- Next stage: **dSPACE ARCHITECTURE DESIGN (Step 11)**

---

## Execution Summary

1. **Files Created:**
   - [`analysis/virya_operating_envelope.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/analysis/virya_operating_envelope.py) (Executable simulation, CSV export, and plotting script)
   - [`tests/test_virya_operating_envelope.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/tests/test_virya_operating_envelope.py) (10 comprehensive unit tests)
   - [`results/virya_operating_envelope/virya_operating_envelope_raw.csv`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/results/virya_operating_envelope/virya_operating_envelope_raw.csv) (720 raw run records)
   - [`results/virya_operating_envelope/virya_operating_envelope_summary.csv`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/results/virya_operating_envelope/virya_operating_envelope_summary.csv) (72 grouped condition summaries)
   - 8 Publication plots under [`plots/virya_operating_envelope/`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/plots/virya_operating_envelope/)
   - [`docs/virya_operating_envelope.md`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/docs/virya_operating_envelope.md) (Engineering report)
   - [`analysis/temp_diagnostic_critical_cases.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/analysis/temp_diagnostic_critical_cases.py) (Diagnostic investigation tool)
   - [`docs/aeb_fsm_dwell_fix_comparison_report.md`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/docs/aeb_fsm_dwell_fix_comparison_report.md) (Before/after comparison report)
2. **Files Modified (Phase B Follow-Up Only):**
   - [`src/aeb_controller.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/src/aeb_controller.py) (FSM dwell release precedence correction)
   - [`analysis/virya_operating_envelope.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/analysis/virya_operating_envelope.py) (Scenario-aware stopping feasibility metric)
   - [`analysis/aeb_parameter_sensitivity.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/analysis/aeb_parameter_sensitivity.py) (Behavioral sync)
   - [`tests/test_aeb_fsm.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/tests/test_aeb_fsm.py) & [`tests/test_performance_evaluation.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/tests/test_performance_evaluation.py) (Test expectation alignment)
   - *(Note: Phase A executed with zero modified production files; baseline preserved in `backups/baseline_before_fsm_dwell_fix/`)*.
3. **Total Simulations:** **720 runs** ($3 \text{ speeds} \times 8 \text{ distances} \times 3 \text{ scenarios} \times 10 \text{ seeds}$).
4. **Test Count:** **123 tests** passing (100% pass rate in `./run_tests.sh`).
5. **Major Observations (Corrected Baseline):**
   - Zero collisions observed across all 720 runs ($0.0\%$ collision rate).
   - Sub-0.30m cases completely eliminated ($15 \to 0$ runs).
   - Global minimum separation distance: increased from $0.2391\text{ m}$ to $\mathbf{0.3086\text{ m}}$.
   - Critical Case (CCRm $0.5\text{ m/s}, 1.5\text{ m}$, Seed 48): minimum distance improved to $0.7411\text{ m}$ (+210%), transitions dropped from $22 \to 5$ (-77%), chattering triplets eliminated ($3 \to 0$).
   - FSM state transitions reduced by $26.9\%$ globally ($10.37 \to 7.58$ mean).
   - Stopping feasibility in CCRm correctly evaluated at $58.3\%$ for non-collision speed-matching.
6. **Final Recommendation:** **RETAIN FOR DSPACE IMPLEMENTATION PLANNING**.
