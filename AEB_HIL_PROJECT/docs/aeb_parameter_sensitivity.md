# Step 9 — AEB Threshold and Braking Parameter Sensitivity Study

## 1. Purpose and Scope

This report documents the **AEB Threshold and Braking Parameter Sensitivity Study** conducted under Step 9 of the project.

The objective of this study is to evaluate the sensitivity, performance trade-offs, and closed-loop behavioral stability of the candidate Autonomous Emergency Braking (AEB) parameters using the established Python simulation environment. Specifically, the study assesses how perturbations around the provisional Time-To-Collision (TTC) thresholds and commanded braking deceleration levels affect collision avoidance, minimum separation distance, intervention timeliness, deceleration authority, and Finite State Machine (FSM) transition dynamics.

The progression of parameter justification and verification across the engineering lifecycle is structured as follows:

```
Literature-Supported Candidate Parameters (Step 8)
                     ↓
Software Sensitivity Study & Trade-Off Analysis (Step 9)
                     ↓
Provisional Baseline Selection
                     ↓
VIRYA Operating-Envelope Validation (Step 10)
                     ↓
dSPACE Real-Time Hardware-in-the-Loop (HIL) Validation (Subsequent Steps)
```

This study does not establish physical vehicle certification, nor does it select an "optimal" tuning. Instead, it systematically identifies trade-offs to determine whether the provisional software baseline is a sound and justifiable candidate to carry forward into platform-specific operating envelope validation (Step 10) and subsequent dSPACE HIL integration.

---

## 2. Scientific Scope and Integrity Statement

> [!IMPORTANT]
> **Scientific Integrity & Simulation Boundary Statement:**
> 
> - **Discrete-Time Simulation Context:** This study is a discrete-time Python software sensitivity investigation with a fixed timestep of $dt = 0.05\text{ s}$. It is **not** Hardware-in-the-Loop (HIL) validation, physical VIRYA vehicle validation, vehicle platform certification, or regulatory compliance testing.
> - **Passenger-Car Literature Limits:** The passenger-car literature values used to motivate candidate parameters provide engineering precedent and initial design bounds, but they do **not** establish that the same values are physically achievable, appropriate, or optimal for the VIRYA platform.
> - **Simulation Space Validity:** The results demonstrate behavior within the tested simulation space only. The absence of simulated collisions does not constitute a formal guarantee or proof of safety across unmodeled operating conditions.
> - **Absence of Optimization:** No parameter optimization or curve-fitting was conducted to bias results toward specific performance targets.

---

## 3. Baseline AEB Configuration

The controller under evaluation is the hierarchical 5-state Finite State Machine (FSM) implemented in `AEBController`. The baseline parameter configuration evaluated throughout this study is defined as:

### Provisional Software Baseline Parameters
- **TTC Activation Thresholds:**
  - $TTC_{\text{warning}} = 4.00\text{ s}$
  - $TTC_{L1} = 3.00\text{ s}$
  - $TTC_{L2} = 2.25\text{ s}$
  - $TTC_{L3} = 1.75\text{ s}$
- **Commanded Braking Deceleration Levels:**
  - $a_{L1} = -2.00\text{ m/s}^2$ ($\approx 0.20\text{ g}$)
  - $a_{L2} = -4.00\text{ m/s}^2$ ($\approx 0.41\text{ g}$)
  - $a_{L3} = -6.00\text{ m/s}^2$ ($\approx 0.61\text{ g}$)
- **Minimum Active Braking Dwell Duration:**
  - $\text{MIN\_BRAKE\_DURATION} = 0.20\text{ s}$

These parameters are explicitly classified as **provisional software baseline parameters**. They represent defensible modeling candidates for simulation analysis and bench preparation, not final physical parameters.

---

## 4. Experimental Methodology

### 4.1 Simulation Model
The closed-loop simulation couples the discrete-time vehicle plant (`VehicleModel`), the first-order brake actuator lag model (`BrakeActuator`), the discrete sensor measurement model (`SensorModel`), and the parameterizable FSM controller (`ConfigurableAEBController`) operating synchronously at $dt = 0.05\text{ s}$. The integration timestep matches the standard $20\text{ Hz}$ execution loop typical of automotive radar and ADAS electronic control units.

### 4.2 Scenarios
The parameter sensitivity analysis is evaluated across three standardized Car-to-Car longitudinal scenarios:
1. **CCR (Car-to-Car Rear Stationary Target):**
   - Ego initial speed: $v_{\text{ego},0} = 15.0\text{ m/s}$ ($54.0\text{ km/h}$)
   - Target speed: $v_{\text{target}} = 0.0\text{ m/s}$ (stationary lead obstacle)
   - Initial separation distance: $d_0 = 30.0\text{ m}$
2. **CCRm (Car-to-Car Rear Moving Target):**
   - Ego initial speed: $v_{\text{ego},0} = 15.0\text{ m/s}$
   - Target initial speed: $v_{\text{target}} = 5.0\text{ m/s}$ ($18.0\text{ km/h}$)
   - Initial separation distance: $d_0 = 25.0\text{ m}$
3. **CCRb (Car-to-Car Rear Braking Target):**
   - Ego initial speed: $v_{\text{ego},0} = 15.0\text{ m/s}$
   - Target initial speed: $v_{\text{target},0} = 10.0\text{ m/s}$ ($36.0\text{ km/h}$)
   - Initial separation distance: $d_0 = 30.0\text{ m}$
   - Target braking profile: $a_{\text{target}} = -2.0\text{ m/s}^2$ initiated at $t = 1.0\text{ s}$

The initial approach speed of $15.0\text{ m/s}$ is a **generic software-level AEB stress condition** designed to evaluate high kinetic energy dissipation and rapid threat escalation. It should **not** be interpreted as representative of the documented low-speed VIRYA operating envelope.

### 4.3 Sensor and Actuator Impairments
To prevent evaluating sensitivity purely under idealized conditions, all runs incorporate nominal closed-loop impairments:
- **Distance Sensor Noise:** Zero-mean Gaussian perturbation with standard deviation $\sigma = 0.05\text{ m}$ ($5\text{ cm}$).
- **Sensor Measurement Latency:** $\lambda = 0.00\text{ s}$ (latency held at baseline to isolate noise-induced boundary switching).
- **Brake Actuator First-Order Lag:** Time constant $\tau = 0.10\text{ s}$ ($100\text{ ms}$ rise time), modeling hydraulic and mechanical pressure build-up dynamics.

These nominal values represent standardized software stress conditions and are not claimed to be measured VIRYA sensor or actuator characteristics.

### 4.4 Random Seeds
To account for stochastic measurement noise variance, every configuration and scenario combination was evaluated across **10 pseudo-random seeds**:
$$\text{Seeds} = [42, 43, 44, 45, 46, 47, 48, 49, 50, 51]$$
The full set of 10 seeds was applied uniformly across all 21 configurations without exception.

### 4.5 Parameter Configurations
The experimental grid evaluates 21 distinct parameter configurations:
- **1 Baseline Configuration**
- **8 One-At-A-Time (OAT) TTC Configurations:** Independently perturbing Warning, L1, L2, and L3 thresholds.
- **2 Combined TTC Configurations:** Multi-boundary Early Conservative and Late Aggressive sets.
- **6 One-At-A-Time (OAT) Braking Configurations:** Independently perturbing L1, L2, and L3 deceleration demands.
- **2 Combined Braking Configurations:** Mild Gentle and Firm Aggressive sets.
- **2 Joint Combined Configurations:** Early-Firm and Late-Mild joint sets.

Total simulation matrix:
$$\text{Total Runs} = 21 \text{ configurations} \times 3 \text{ scenarios} \times 10 \text{ seeds} = 630 \text{ simulation runs}$$

### 4.6 Evaluation Metrics
For every simulation run, 14 standard metrics were recorded and statistically aggregated:
1. `collision_status`: Boolean indicator of physical contact with closing velocity ($d_{\text{raw}} \le 0 \land v_{\text{rel}} > 10^{-4}\text{ m/s}$).
2. `minimum_distance`: Minimum bumper-to-bumper clearance ($d_{\min}$).
3. `aeb_activation_time`: Timestamp of initial active braking command ($a_{\text{cmd}} < 0$).
4. `ttc_at_activation`: Kinematic TTC at the instant of active intervention.
5. `max_braking_deceleration`: Peak commanded deceleration magnitude ($|a_{\text{cmd},\max}|$).
6. `max_actual_deceleration`: Peak delivered deceleration magnitude ($|a_{\text{act},\max}|$).
7. `ninety_percent_braking_response_time`: Rise time to reach 90% of initial commanded deceleration.
8. `stopping_time`: Total simulation time to complete ego standstill ($v_{\text{ego}} \approx 0$).
9. `stopping_distance`: Cumulative longitudinal distance traversed to standstill.
10. `vehicle_stop_status`: Categorical standstill status (`"STOPPED"` vs. `"NOT_STOPPED"`).
11. `final_relative_velocity`: Residual speed difference at scenario termination.
12. `minimum_relative_velocity`: Maximum opening speed between vehicles.
13. `state_transition_count`: Cumulative discrete FSM state switches.
14. `triplet_reversal_count`: **"Number of 3-sample sliding FSM reversal patterns"**, defined by:
    $$\text{state}[k] \neq \text{state}[k+1] \land \text{state}[k] == \text{state}[k+2]$$
    excluding $\text{SAFE} \to X \to \text{SAFE}$. This metric serves strictly as a software-level indicator of boundary switching frequency and is **not** a direct measure of physical brake chattering, mechanical vibration, or vehicle instability.

---

## 5. TTC Threshold Sensitivity Study

The TTC threshold grid evaluates sensitivity around the baseline while strictly enforcing the monotonic ordering constraint:
$$TTC_{\text{warning}} > TTC_{L1} > TTC_{L2} > TTC_{L3} > 0$$

```
                                    TTC THRESHOLD GRID
                       
         [ Warning ]                 [ Level 1 ]                [ Level 2 ]                [ Level 3 ]
       3.50 s  (-0.50 s)           2.50 s  (-0.50 s)          2.00 s  (-0.25 s)          1.50 s  (-0.25 s)
       4.00 s  (Baseline)          3.00 s  (Baseline)         2.25 s  (Baseline)         1.75 s  (Baseline)
       4.50 s  (+0.50 s)           3.50 s  (+0.50 s)          2.50 s  (+0.25 s)          2.00 s  (+0.25 s)
```

In addition to One-At-A-Time perturbations, two combined boundary configurations were evaluated:
- **TTC Early Conservative:** $(4.50, 3.50, 2.50, 2.00)\text{ s}$ — represents early intervention oriented toward maximizing spatial safety margins.
- **TTC Late Aggressive:** $(3.50, 2.50, 2.00, 1.50)\text{ s}$ — represents delayed intervention oriented toward minimizing false alarms and driver nuisance.

---

## 6. Braking Deceleration Sensitivity Study

The braking deceleration grid evaluates sensitivity around the baseline demands while strictly enforcing the magnitude ordering constraint:
$$0 > a_{L1} > a_{L2} > a_{L3} \quad \iff \quad |a_{L1}| < |a_{L2}| < |a_{L3}|$$

```
                                 BRAKING DECELERATION GRID
                       
          [ Level 1 ]                        [ Level 2 ]                        [ Level 3 ]
     -1.50 m/s² (+0.50 m/s²)            -3.00 m/s² (+1.00 m/s²)            -5.00 m/s² (+1.00 m/s²)
     -2.00 m/s² (Baseline)              -4.00 m/s² (Baseline)              -6.00 m/s² (Baseline)
     -2.50 m/s² (-0.50 m/s²)            -5.00 m/s² (-1.00 m/s²)            -7.00 m/s² (-1.00 m/s²)
```

Combined braking configurations evaluated:
- **Brake Mild Gentle:** $(-1.50, -3.00, -5.00)\text{ m/s}^2$ — lower deceleration demand prioritizing occupant ride comfort.
- **Brake Firm Aggressive:** $(-2.50, -5.00, -7.00)\text{ m/s}^2$ — high deceleration demand prioritizing rapid kinetic energy dissipation.

---

## 7. Joint Parameter Sensitivity

Two joint configurations were tested to explore compounding parameter interactions:
1. **Joint Early Firm:** Early TTC thresholds $(4.50, 3.50, 2.50, 2.00)\text{ s}$ paired with Firm braking demands $(-2.50, -5.00, -7.00)\text{ m/s}^2$.
2. **Joint Late Mild:** Late TTC thresholds $(3.50, 2.50, 2.00, 1.50)\text{ s}$ paired with Mild braking demands $(-1.50, -3.00, -5.00)\text{ m/s}^2$.

---

## 8. Results

### 8.1 Collision Outcome
**Zero collisions were observed in all 630 simulated runs under the tested scenarios, parameter configurations, impairment conditions, and random seeds.**

This observation confirms that the controller avoided impact under the specific initial kinematics, noise realizations, and lag dynamics tested. However, as established in the scientific integrity statement, zero collisions within this simulation space does **not** constitute proof of universal collision avoidance.

### 8.2 Minimum Separation Distance ($d_{\min}$)
The mean minimum separation distance across all runs ranged from **$0.5431\text{ m}$** to **$1.8507\text{ m}$**:
- **Baseline Clearance:**
  - CCR: $d_{\min} = 0.9774\text{ m}$ ($\pm 0.015\text{ m}$)
  - CCRm: $d_{\min} = 1.0861\text{ m}$ ($\pm 0.012\text{ m}$)
  - CCRb: $d_{\min} = 1.0825\text{ m}$ ($\pm 0.021\text{ m}$)
- **Threshold Effects:** Advancing TTC thresholds monotonically increased minimum clearance. In CCR, `TTC_Early_Conservative` expanded clearance to **$1.4701\text{ m}$** ($+50.4\%$), whereas `TTC_Late_Aggressive` contracted clearance to **$0.6145\text{ m}$** ($-37.1\%$).
- **Braking Effects:** Softening deceleration demands in `Brake_Mild_Gentle` contracted CCR clearance to **$0.6781\text{ m}$** ($-30.6\%$). Conversely, `Brake_Firm_Aggressive` expanded clearance to **$1.1533\text{ m}$** ($+18.0\%$).

### 8.3 AEB Activation Timing
In CCR and CCRm, because initial speed and distance place the vehicles immediately within the threat envelope, active braking initiates at $t = 0.00\text{ s}$ across nearly all configurations (with minor $0.025\text{ s}$ latency in aggressive late-triggering sets).

In CCRb, where the threat develops dynamically after target braking starts at $t = 1.0\text{ s}$:
- Baseline activation occurs at $t_{\text{act}} = 1.870\text{ s}$ ($TTC_{\text{act}} = 2.958\text{ s}$).
- Early Level 1 TTC ($3.50\text{ s}$) advances activation to $t_{\text{act}} = 1.605\text{ s}$ ($TTC_{\text{act}} = 3.483\text{ s}$).
- Late Level 1 TTC ($2.50\text{ s}$) delays activation to $t_{\text{act}} = 2.150\text{ s}$ ($TTC_{\text{act}} = 2.467\text{ s}$).

### 8.4 Braking Effectiveness and Delivered Deceleration
The first-order actuator lag ($\tau = 0.10\text{ s}$) directly constrained peak delivered deceleration relative to commanded levels:
- In CCR, commanded $-6.00\text{ m/s}^2$ demands achieved exactly $-6.00\text{ m/s}^2$ actual delivered deceleration.
- In `Brake_L3_-7.0`, commanded $-7.00\text{ m/s}^2$ achieved $-6.997\text{ m/s}^2$, but yielded only $+0.016\text{ m}$ of additional clearance compared to $-6.00\text{ m/s}^2$ because the ego vehicle reached complete standstill shortly after peak pressure was established.
- In CCRm, peak actual deceleration saturated at $-4.00\text{ m/s}^2$ across baseline, because Level 2 braking resolved the speed difference without requiring Level 3 intervention.

### 8.5 FSM State Transitions
State transitions across all configurations remained bounded:
- In CCR: mean transitions ranged between **$13.3$** (`TTC_Warn_4.5s`) and **$23.0$** (`TTC_Warn_3.5s`), with the baseline at **$19.6$**.
- In CCRb: mean transitions ranged between **$10.2$** (`Brake_L1_-2.5`) and **$24.3$** (`TTC_Warn_3.5s`), with the baseline at **$17.2$**. Stronger initial braking ($a_{L1} = -2.5\text{ m/s}^2$) stabilized following distance earlier, cutting transitions by over $40\%$.

### 8.6 Triplet Reversal Patterns
**No runaway FSM switching was observed within the tested parameter space, and the 0.20 s dwell mechanism limited rapid state transitions under the tested conditions.**

The `triplet_reversal_count` remained minimal across all 630 runs:
- CCR baseline mean: **$1.2$** reversals per run.
- CCRm baseline mean: **$0.4$** reversals per run.
- CCRb baseline mean: **$1.2$** reversals per run.
- Worst-case mean across all 21 configurations was **$2.8$** reversals in CCRm under `Joint_Late_Mild`.

---

## 9. Baseline Comparison

The table below compiles the empirical performance of all 21 configurations in the CCR stationary target scenario, directly extracted from `aeb_parameter_sensitivity_summary.csv`:

| Configuration Name | Category | $d_{\min}$ Mean | $\Delta d_{\min}$ (m) | $\% \Delta d_{\min}$ | Delivered Decel | Transitions | Triplet Reversals |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Baseline** | baseline | **$0.9774\text{ m}$** | **$+0.0000$** | **$0.0\%$** | **$6.00\text{ m/s}^2$** | **$19.6$** | **$1.2$** |
| **TTC_Warn_3.5s** | oat_ttc | $0.8860\text{ m}$ | $-0.0914$ | $-9.3\%$ | $6.00\text{ m/s}^2$ | $23.0$ | $1.8$ |
| **TTC_Warn_4.5s** | oat_ttc | $1.1006\text{ m}$ | $+0.1232$ | $+12.6\%$ | $6.00\text{ m/s}^2$ | $13.3$ | $0.5$ |
| **TTC_L1_2.5s** | oat_ttc | $0.7816\text{ m}$ | $-0.1958$ | $-20.0\%$ | $6.00\text{ m/s}^2$ | $20.8$ | $1.0$ |
| **TTC_L1_3.5s** | oat_ttc | $1.1353\text{ m}$ | $+0.1579$ | $+16.2\%$ | $6.00\text{ m/s}^2$ | $20.5$ | $1.5$ |
| **TTC_L2_2.0s** | oat_ttc | $0.9059\text{ m}$ | $-0.0715$ | $-7.3\%$ | $6.00\text{ m/s}^2$ | $18.9$ | $1.4$ |
| **TTC_L2_2.5s** | oat_ttc | $0.9870\text{ m}$ | $+0.0096$ | $+1.0\%$ | $6.00\text{ m/s}^2$ | $18.3$ | $1.2$ |
| **TTC_L3_1.5s** | oat_ttc | $0.8254\text{ m}$ | $-0.1520$ | $-15.6\%$ | $6.00\text{ m/s}^2$ | $18.0$ | $1.2$ |
| **TTC_L3_2.0s** | oat_ttc | $1.0620\text{ m}$ | $+0.0847$ | $+8.7\%$ | $6.00\text{ m/s}^2$ | $17.4$ | $1.4$ |
| **TTC_Early_Conservative** | comb_ttc | $1.4701\text{ m}$ | $+0.4927$ | $+50.4\%$ | $6.00\text{ m/s}^2$ | $16.5$ | $0.8$ |
| **TTC_Late_Aggressive** | comb_ttc | $0.6145\text{ m}$ | $-0.3629$ | $-37.1\%$ | $6.00\text{ m/s}^2$ | $21.6$ | $2.2$ |
| **Brake_L1_-1.5** | oat_brake | $0.9073\text{ m}$ | $-0.0701$ | $-7.2\%$ | $6.00\text{ m/s}^2$ | $19.0$ | $1.1$ |
| **Brake_L1_-2.5** | oat_brake | $1.0931\text{ m}$ | $+0.1157$ | $+11.8\%$ | $6.00\text{ m/s}^2$ | $16.0$ | $0.7$ |
| **Brake_L2_-3.0** | oat_brake | $0.9122\text{ m}$ | $-0.0652$ | $-6.7\%$ | $6.00\text{ m/s}^2$ | $17.5$ | $1.5$ |
| **Brake_L2_-5.0** | oat_brake | $1.0463\text{ m}$ | $+0.0689$ | $+7.1\%$ | $6.00\text{ m/s}^2$ | $18.3$ | $0.7$ |
| **Brake_L3_-5.0** | oat_brake | $0.7666\text{ m}$ | $-0.2108$ | $-21.6\%$ | $5.00\text{ m/s}^2$ | $18.8$ | $1.1$ |
| **Brake_L3_-7.0** | oat_brake | $0.9935\text{ m}$ | $+0.0161$ | $+1.6\%$ | $7.00\text{ m/s}^2$ | $18.9$ | $1.1$ |
| **Brake_Mild_Gentle** | comb_brake | $0.6781\text{ m}$ | $-0.2993$ | $-30.6\%$ | $5.00\text{ m/s}^2$ | $19.1$ | $1.9$ |
| **Brake_Firm_Aggressive** | comb_brake | $1.1533\text{ m}$ | $+0.1759$ | $+18.0\%$ | $6.95\text{ m/s}^2$ | $18.7$ | $0.9$ |
| **Joint_Early_Firm** | joint | $1.6861\text{ m}$ | $+0.7087$ | $+72.5\%$ | $7.00\text{ m/s}^2$ | $18.7$ | $1.5$ |
| **Joint_Late_Mild** | joint | $0.6832\text{ m}$ | $-0.2942$ | $-30.1\%$ | $5.00\text{ m/s}^2$ | $6.3$ | $0.3$ |

---

## 10. Cross-Scenario Interpretation

### 10.1 CCR (Car-to-Car Rear Stationary Target)
CCR represents a severe kinematic demand requiring the ego vehicle to dissipate all kinetic energy ($15.0\text{ m/s} \to 0\text{ m/s}$) before impacting a stationary obstacle. Key findings:
- All three braking levels ($L1 \to L2 \to L3$) are exercised sequentially in every run.
- Minimum separation is highly sensitive to $TTC_{L1}$ and $TTC_{L3}$. Delaying $TTC_{L1}$ to $2.50\text{ s}$ reduces clearance by $0.196\text{ m}$, while delaying $TTC_{L3}$ to $1.50\text{ s}$ reduces clearance by $0.152\text{ m}$.
- The baseline configuration delivers an average clearance of **$0.9774\text{ m}$**, maintaining a safe positive margin without excessive overshoot.

### 10.2 CCRm (Car-to-Car Rear Moving Target)
CCRm involves a lower relative closing speed ($v_{\text{rel},0} = 10.0\text{ m/s}$) as the lead vehicle travels continuously at $5.0\text{ m/s}$. Key observations:
- **Threat Resolution in Level 2:** In the baseline and all OAT Level 3 variations, **Level 3 emergency braking is never activated**. The moderate deceleration of Level 2 ($-4.00\text{ m/s}^2$) reduces relative velocity to near zero before TTC drops below $1.75\text{ s}$.
- Consequently, varying Level 3 TTC ($1.50\text{ s}$ vs. $2.00\text{ s}$) or Level 3 deceleration ($-5.0\text{ m/s}^2$ vs. $-7.0\text{ m/s}^2$) produces **identical metrics** ($d_{\min} = 1.0861\text{ m}$).
- This confirms that in moderate-speed following scenarios, the hierarchical FSM successfully mitigates collisions without resorting to severe full emergency braking.

### 10.3 CCRb (Car-to-Car Rear Braking Target)
CCRb introduces dynamic threat escalation because the lead vehicle decelerates at $-2.00\text{ m/s}^2$ starting at $t = 1.0\text{ s}$. Key observations:
- **Early Dissipation Avoids Level 3 Escalation:** When Level 1 deceleration is strengthened to $-2.50\text{ m/s}^2$ (`Brake_L1_-2.5`), the controller extracts kinetic energy early, preventing TTC from dropping into Level 3 entirely. This reduces total state transitions from $17.2$ to **$10.2$** and triplet reversals from $1.2$ to **$0.3$**.
- This demonstrates a distinct advantage of staged multi-level braking: effective early low-g braking frequently resolves dynamic threats before acute intervention is required.

---

## 11. Parameter Trade-Offs

The sensitivity analysis highlights three fundamental engineering trade-offs:

1. **Safety Margin vs. False-Alarm Nuisance Horizon:**
   - Expanding TTC thresholds (e.g., $TTC_{\text{warn}} = 4.50\text{ s}$, $TTC_{L1} = 3.50\text{ s}$) increases physical clearance by up to $+50\%$.
   - However, at low speeds or in urban following, large TTC thresholds cause premature interventions and false alarms on benign road objects, leading drivers to disable the system.
2. **Kinetic Dissipation vs. Ride Severity and Pitch Dynamics:**
   - Demanding higher deceleration (e.g., $-7.00\text{ m/s}^2$) increases clearance, but subjects occupants to severe longitudinal jerk and chassis pitch.
   - Due to actuator lag ($\tau = 0.10\text{ s}$), the marginal clearance gain between $-6.00\text{ m/s}^2$ and $-7.00\text{ m/s}^2$ in CCR was only $+0.016\text{ m}$ ($+1.6\%$), indicating diminishing returns for aggressive deceleration demands.
3. **FSM Transition Activity vs. Responsiveness:**
   - Relaxing thresholds can increase transition hunting if boundary crossings coincide with sensor noise variance.
   - The $0.20\text{ s}$ dwell time provided robust hysteresis across all tested configurations, keeping triplet reversals below $3$ patterns per run.

---

## 12. Provisional Parameter Assessment

The table below synthesizes the experimental sensitivity findings against the literature grounding established in Step 8 to provide an engineering status for each parameter:

| Parameter | Baseline Value | Evidence Foundation | Sensitivity Observation | Project Status | Technical Justification |
| :--- | :---: | :--- | :--- | :---: | :--- |
| **Warning TTC** | **$4.00\text{ s}$** | ISO 22839 mitigation ceiling; van der Horst (1990) driver reaction upper bound. | Perturbations ($3.50\text{--}4.50\text{ s}$) shift CCR clearance by $\pm 0.10\text{ m}$. Transitions drop to $13.3$ at $4.50\text{ s}$. | **RETAIN PROVISIONALLY** | Operates as a conservative advisory envelope ceiling; candidate for reduction if campus testing shows nuisance. |
| **Level 1 TTC** | **$3.00\text{ s}$** | Honda CMBS Stage 1 production benchmark (Sugimoto & Sauer, 2005). | High sensitivity: decreasing to $2.50\text{ s}$ cuts clearance by $-20\%$; increasing to $3.50\text{ s}$ gains $+16\%$. Baseline $3.00\text{ s}$ provides balanced margin. | **RETAIN PROVISIONALLY** | Well-grounded in literature and exhibits balanced closed-loop sensitivity without premature stopping. |
| **Level 2 TTC** | **$2.25\text{ s}$** | Engineering design midpoint between L1 ($3.0\text{ s}$) and L3 ($1.75\text{ s}$). | Modest sensitivity: $2.00\text{ s}$ vs. $2.50\text{ s}$ shifts clearance by $-0.07\text{ m}$ to $+0.01\text{ m}$. Transitions and reversals remain stable. | **RETAIN PROVISIONALLY** | Provides smooth transition between preliminary and emergency braking; robust against noise. |
| **Level 3 TTC** | **$1.75\text{ s}$** | Upper bound of CAMP critical Last-Second Braking (Kiefer et al., 1999). | Lowering to $1.50\text{ s}$ reduces clearance by $-15.6\%$ ($0.825\text{ m}$). Baseline $1.75\text{ s}$ preserves near-$1\text{ m}$ margin under lag. | **RETAIN PROVISIONALLY** | Critical buffer required to compensate for physical actuator pressure build-up delay. |
| **Level 1 Decel** | **$-2.00\text{ m/s}^2$** | CAMP haptic alert standard ($0.2\text{--}0.3\text{ g}$); ISO 22839 light mitigation. | Moderate sensitivity: $-1.5\text{ m/s}^2$ loses $-7.2\%$ clearance; $-2.5\text{ m/s}^2$ gains $+11.8\%$ and prevents L3 escalation in CCRb. | **RETAIN PROVISIONALLY** | Well-suited as tactile warning deceleration that avoids unrestrained passenger pitch. |
| **Level 2 Decel** | **$-4.00\text{ m/s}^2$** | Mercedes-Benz PRE-SAFE Stage 1 autonomous braking (Schittenhelm, 2006). | Resolves CCRm scenario threats without requiring L3 emergency braking. Stable stopping profile. | **RETAIN PROVISIONALLY** | Delivers effective kinetic energy dissipation while avoiding excessive tire slip demands. |
| **Level 3 Decel** | **$-6.00\text{ m/s}^2$** | Honda CMBS Stage 3 ($\sim 0.6\text{ g}$); satisfies UNECE R152 deceleration floor. | Decreasing to $-5.0\text{ m/s}^2$ drops clearance by $-21.6\%$. Increasing to $-7.0\text{ m/s}^2$ delivers diminishing returns ($+1.6\%$). | **REQUIRES MORE EVIDENCE** | The $-6.00\text{ m/s}^2$ demand remains a software command candidate requiring physical VIRYA platform measurement. |

---

## 13. Limitations and Validity Boundaries

The conclusions of this sensitivity study are bounded by the following technical limitations:
1. **Discrete-Time Simulation:** Analysis is executed within an idealized numerical simulation environment ($dt = 0.05\text{ s}$).
2. **One-Dimensional Longitudinal Model:** Vehicle motion is restricted to longitudinal translation without lateral tire forces, roll/pitch suspension dynamics, or steering avoidance maneuvers.
3. **Road Surface Idealization:** Flat terrain with constant infinite adhesion is assumed; tire slip, brake fade, and variable surface friction ($\mu$) are not modeled.
4. **Actuator Representation:** The braking plant is modeled as a linear first-order lag ($\tau = 0.10\text{ s}$) without dead-time, hydraulic non-linearities, or anti-lock braking (ABS) cycling.
5. **Statistical Sensor Noise:** Sensor noise is modeled as zero-mean Gaussian noise ($\sigma = 0.05\text{ m}$) rather than raw empirical radar/camera point clouds or multi-path clutter.
6. **Generic Speed Stress Condition:** The $15.0\text{ m/s}$ approach speed is a software benchmark, not representative of the VIRYA operating envelope.
7. **Absence of HIL Bench Execution:** No real-time bus communication (CAN bus latency, message jitter) or dSPACE processor execution has yet occurred.
8. **No Physical Platform Validation:** The VIRYA drive-by-wire vehicle has not executed these deceleration profiles.
9. **Finite Noise Sampling:** While 10 seeds provide statistical confidence against random noise variance, they do not encompass all real-world environmental uncertainties.
10. **Non-Optimality:** Parameter ranges were bounded to local neighborhoods; global optimality cannot be claimed.
11. **Observation vs. Proof:** Zero collisions in the tested runs do not establish universal collision avoidance.

### Relevance to the VIRYA Platform
The current Step 9 experiment is a **software-level algorithm sensitivity study**. Its purpose is to understand parameter trade-offs before hardware integration. 

The $15.0\text{ m/s}$ approach condition should **not** be interpreted as the operating speed of the VIRYA platform. VIRYA-specific suitability requires a separate operating-envelope study using the documented platform constraints and eventually measured actuator and sensor behavior.

---

## 14. Decision for Step 10

### Definitive Operational Summary:
- **Step 9 is complete as a software sensitivity study.**
- **The results are sufficient to carry the baseline configuration forward as a provisional candidate.**
- **No further Step 9 parameter sweep is required at this stage.**
- **The next required stage is Step 10: VIRYA Operating-Envelope Validation.**

### Mandate for Step 10:
Step 10 must investigate the provisional software baseline at speeds and initial-distance conditions relevant to the actual VIRYA platform (e.g., campus/urban low-speed operational design domain). Step 10 should **not** immediately optimize the controller again; its primary objective is to establish whether the provisional software baseline behaves sensibly within the platform-relevant operating envelope.

---

## 15. Final Step 9 Conclusion

Step 9 evaluated the sensitivity of the AEB TTC thresholds and braking deceleration demands using 630 discrete-time Python simulations spanning 21 parameter configurations, three longitudinal collision scenarios, and ten random seeds under nominal sensor-noise and actuator-lag conditions. Zero collisions were observed within this tested simulation space. Earlier intervention and stronger braking generally increased simulated separation, while the results also demonstrated diminishing returns and scenario-dependent behavior. The results therefore support retaining the 4.00/3.00/2.25/1.75 s TTC thresholds and -2.0/-4.0/-6.0 m/s² braking demands as a provisional software baseline. These values are not yet physically validated for VIRYA. The next required stage is VIRYA operating-envelope validation, followed by dSPACE HIL implementation and comparison.

---

## Step 9 Status

- Experimental study: **COMPLETE**
- 630 runs: **COMPLETE**
- Parameter sensitivity: **COMPLETE**
- Multi-seed analysis: **COMPLETE**
- Production controller modified: **NO**
- HIL validation: **NOT YET**
- VIRYA physical validation: **NOT YET**
- Parameter status: **PROVISIONAL**
- Next step: **STEP 10 — VIRYA OPERATING-ENVELOPE VALIDATION**
