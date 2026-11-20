# Literature and Parameter Justification for the AEB Algorithm

## 1. Executive Summary & Scope

This document provides the formal scientific and engineering justification for the Autonomous Emergency Braking (AEB) algorithm, its control architecture, and its numerical parameter selection within this project.

### Project Baseline State
The active software implementation evaluated through Steps 1 to 7 is characterized by:
- **Architecture:** Time-To-Collision (TTC) based hierarchical Finite State Machine (FSM) controller.
- **Discrete States:** `SAFE`, `WARNING`, `AEB_LEVEL_1`, `AEB_LEVEL_2`, `AEB_LEVEL_3`.
- **TTC Activation Thresholds:**
  - Warning: $TTC_{\text{warn}} = 4.00\text{ s}$
  - Level 1 Braking: $TTC_{L1} = 3.00\text{ s}$
  - Level 2 Braking: $TTC_{L2} = 2.25\text{ s}$
  - Level 3 Braking: $TTC_{L3} = 1.75\text{ s}$
- **Commanded Deceleration Ceilings:**
  - Level 1: $a_{\text{cmd}, L1} = -2.00\text{ m/s}^2$ ($\approx 0.20\text{ g}$)
  - Level 2: $a_{\text{cmd}, L2} = -4.00\text{ m/s}^2$ ($\approx 0.41\text{ g}$)
  - Level 3: $a_{\text{cmd}, L3} = -6.00\text{ m/s}^2$ ($\approx 0.61\text{ g}$)

### Core Scientific Position
> [!IMPORTANT]
> - **Simulation Context:** The current parameter set represents a verified, robust *simulation baseline*. It does not claim regulatory compliance (e.g., UNECE R152, Euro NCAP) or certified deployment on the physical VIRYA drive-by-wire (DBW) vehicle.
> - **Honest Grounding Policy:** Literature provides foundational principles, general operating bands, and precedent from passenger car original equipment manufacturers (OEMs). However, literature values cannot be uncritically retrofitted to declare that arbitrary project values are "proven."
> - **Categorical Distinctions:** Every parameter is explicitly distinguished as:
>   1. *Literature-Supported*: Grounded in published empirical studies, industrial benchmarks, or international standards.
>   2. *Engineering Design Assumption*: A defensible modeling choice adopted for simulation stability, pending physical platform data.
>   3. *Requires Platform Validation*: Dependent on physical hardware constraints (actuator bandwidth, tire-road adhesion, chassis pitch, sensor noise).

---

## 2. Theoretical Foundations of Threat Assessment & Architecture

### 2.1 Why Time-To-Collision (TTC) is Appropriate for AEB Threat Assessment

Time-To-Collision ($TTC$), originally formalized in traffic safety by Hayward (1972) and in visual psychophysics by Lee (1976), is defined as the time remaining before two vehicles collide assuming relative velocity remains constant:

$$TTC(t) = \frac{d(t)}{v_{\text{rel}}(t)} = \frac{x_{\text{target}}(t) - x_{\text{ego}}(t)}{v_{\text{ego}}(t) - v_{\text{target}}(t)}$$

valid for $v_{\text{rel}}(t) > 0$.

TTC is the most widely adopted primary threat assessment metric in active safety for the following kinematic and human-factors reasons:

1. **Direct Representation of Driver Reaction Horizon:**
   Human perception-reaction time ($t_{\text{PRT}}$) is a temporal quantity, typically ranging from $0.8\text{ s}$ to $1.5\text{ s}$ for alerted drivers and up to $2.5\text{ s}$ for unalerted drivers (van der Horst, 1990; ISO 15623). A temporal threat metric directly couples the vehicle’s warning and intervention timing to human cognitive and physiological reaction limits, whereas a pure distance threshold ($d_{\text{thresh}}$) fails to differentiate whether a $20\text{ m}$ gap represents immediate catastrophe ($v_{\text{rel}} = 30\text{ m/s} \implies TTC = 0.67\text{ s}$) or complete safety ($v_{\text{rel}} = 1\text{ m/s} \implies TTC = 20\text{ s}$).

2. **Ecological Visual Invariance ($\tau$-Theory):**
   Lee (1976) demonstrated that biological visual systems do not measure absolute metric distance or physical ground speed directly. Instead, animals and human drivers perceive the inverse of the relative rate of retinal image expansion ($\tau \approx \theta / \dot{\theta}$), which is mathematically identical to $TTC$. Utilizing $TTC$ in the automated controller mimics the perceptual cues that human drivers rely on, leading to intuitive and predictable escalation.

3. **Minimal Sensor Requirement:**
   Computing kinematic $TTC$ requires only range ($d$) and range rate ($\dot{d} = -v_{\text{rel}}$). These signals are directly available as primary outputs from standard automotive millimeter-wave radar (via Doppler shift) and monocular/stereo cameras without requiring knowledge of absolute ego vehicle mass, chassis inertia, road incline, or tire-road friction coefficient ($\mu$).

4. **Speed-Scaled Spatial Margin:**
   Because $d_{\text{trigger}} = TTC_{\text{thresh}} \cdot v_{\text{rel}}$, a constant TTC threshold automatically scales the required spatial intervention distance proportionally with closing speed.

---

### 2.2 Why a Hierarchical / Multi-Level Braking FSM is Appropriate

Single-stage AEB controllers apply maximum braking deceleration (e.g., $-8\text{ to } -10\text{ m/s}^2$) as soon as an imminent collision threshold is crossed. While optimal for extreme kinematic avoidance when crash is physically unavoidable, single-stage emergency intervention suffers severe real-world operational penalties:

1. **Nuisance Mitigation and Driver Acceptance:**
   NHTSA/CAMP studies (Kiefer et al., 1999) and ISO 22839 establish that abrupt high-g braking interventions caused by false positives, radar clutter, or aggressive cut-ins induce strong driver irritation, leading to drivers disabling the safety system. A multi-level strategy provides a grace period: early acoustic/visual warning alerts the driver, followed by mild haptic/comfort braking, reserving maximum deceleration exclusively for acute, non-responsive emergencies.

2. **Occupant and Follower Safety (Secondary Collisions):**
   Sudden full emergency braking in dense traffic creates severe rear-end collision risk from following vehicles. Multi-stage deceleration ($L1 \to L2 \to L3$) illuminates brake lamps early and establishes moderate preliminary deceleration, alerting following traffic before maximal emergency force is developed.

3. **Brake Actuator Pre-Filling and Hydraulic Priming:**
   Physical hydraulic and electromechanical brake systems possess internal mechanical play (pad-to-rotor air gaps) and fluid inertia. As demonstrated by Tamura et al. (2001) for Honda and Schittenhelm (2006) for Mercedes-Benz, applying an initial low-deceleration command ($L1 \approx -2.0\text{ m/s}^2$) closes pad clearances, pre-charges hydraulic lines, and primes the brake booster. When the severe threshold ($L3$) is reached, the plant can ramp to full deceleration with minimal delay.

4. **Kinetic Energy Dissipation Under Degradation:**
   Even if full avoidance cannot be achieved due to low road friction or severe initial closing speed, early moderate braking ($L1$ and $L2$) dissipates significant kinetic energy ($E_k \propto v^2$), drastically reducing impact severity ($\Delta V$) and occupant injury risk (Sugimoto & Sauer, 2005).

---

## 3. Algorithm Selection & Comparative Analysis

To justify selecting a **TTC-based Hierarchical FSM** over alternative collision avoidance paradigms, this section evaluates five major algorithmic approaches reported in academic and industrial literature:

1. **Approach 1: Time-To-Collision (TTC) Hierarchical FSM** (Current Project Architecture)
2. **Approach 2: Time Headway / Constant Time Gap (THW)**
3. **Approach 3: Kinematic Distance-Based Threshold Models** (e.g., Berkeley PATH, Mazda, Honda Safety Distance Models)
4. **Approach 4: Model Predictive Control (MPC)**
5. **Approach 5: Artificial Intelligence / Fuzzy Logic / Reinforcement Learning**

### 3.1 Algorithmic Comparison Matrix

| Evaluation Dimension | 1. TTC Hierarchical FSM (Selected) | 2. Time Headway (THW) | 3. Kinematic Distance Models | 4. Model Predictive Control (MPC) | 5. Fuzzy Logic / AI Policy |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Mathematical Formulation** | Discrete state transitions triggered by $d / v_{\text{rel}}$ against tiered thresholds | $THW = d / v_{\text{ego}}$ compared against headway margin | $d_{\text{safe}} = v_{\text{ego}} \tau + \frac{v_{\text{ego}}^2}{2 a_{\text{ego}}} - \frac{v_{\text{target}}^2}{2 a_{\text{target}}}$ | $\min_u \sum (x - x_{\text{ref}})^2 + u^2$ subject to vehicle dynamics | Fuzzy inference rules or deep neural network policies |
| **Computational Complexity** | $O(1)$ constant time; single division and comparator tree | $O(1)$ constant time; single division | $O(1)$ constant time; algebraic quadratic calculation | $O(N^3)$ to $O(N^4)$ per cycle; iterative quadratic/nonlinear solver | $O(R)$ for fuzzy rules; $O(M \times K)$ matrix mults for DNNs |
| **Sensor Input Requirements** | Relative distance $d$, relative velocity $v_{\text{rel}}$ | Relative distance $d$, ego speed $v_{\text{ego}}$ | $d$, $v_{\text{ego}}$, $v_{\text{target}}$, target acceleration $a_{\text{target}}$ | Full vehicle state vector ($x, y, v, \psi, \dot{\psi}$), obstacle states | Radar/camera feature vectors, occupancy grids, or raw telemetry |
| **Physical Interpretability** | **Complete**: Transparent state boundaries, deterministic execution | **High**: Clear operational time headway | **High**: Grounded in Newtonian stopping equations | **Moderate**: Implicit in cost weightings and optimization constraints | **Low**: "Black-box" policy, difficult verification |
| **Real-Time Execution Determinism** | **Guaranteed**: Fixed execution path ($< 10\text{ }\mu\text{s}$), zero iteration jitter | **Guaranteed**: Fixed execution path | **Guaranteed**: Fixed closed-form evaluation | **Uncertain**: Potential non-convergence or solver timeout on embedded ECU | **Variable**: Bounded for fuzzy; GPU/NPU memory bandwidth constraints for DNN |
| **Ease of HIL Implementation** | **High**: Trivially implemented on dSPACE RTI, CAN bus, MicroAutoBox | **High**: Simple CAN bus block | **Moderate**: Requires clean target acceleration estimation | **Difficult**: High CPU load on real-time target; complex solver porting | **Difficult**: Hardware accelerator integration required |
| **Suitability for Low-Speed DBW** | **High**: Tunable to low-speed campus/urban ODD | **Poor**: Fails when target is stationary ($v_{\text{ego}} > 0, v_{\text{target}} = 0$) | **Moderate**: Sensitive to target acceleration sensor noise | **Excessive**: Over-engineered for 1D longitudinal safety envelope | **Low**: Overly complex, difficult to certify |
| **Graded Braking Support** | **Native**: Multi-level states mapped directly to deceleration steps | **Poor**: Primarily car-following (ACC), non-escalating | **Possible**: Multiple distance buffers required | **Continuous**: Smooth continuous brake profile optimization | **Continuous**: Smooth fuzzy interpolation |
| **Robustness to Sensor Uncertainty** | **High**: Easily stabilized using dwell-time and hysteresis filtering | **Moderate**: Sensitive to target cut-ins | **Poor**: Highly sensitive to differentiated acceleration noise $\ddot{d}$ | **Moderate**: Sensitive to plant mismatch and state estimation covariance | **Unpredictable**: Edge-case instability under distribution shifts |

---

### 3.2 Rationale for Project Selection

The **TTC-based Hierarchical FSM** was selected for this project based on four engineering imperatives:

1. **Deterministic State Accountability for Safety Systems:**
   Functional safety standards (ISO 26262) mandate that safety-critical intervention decisions be traceable, verifiable, and verifiable under all input combinations. The hierarchical FSM defines strict, exhaustive, non-overlapping operating regimes (`SAFE` $\to$ `WARNING` $\to$ `L1` $\to$ `L2` $\to$ `L3`) where the exact trigger conditions can be formally audited and tested.

2. **Independence from Target Acceleration Estimation ($\ddot{d}$):**
   Full kinematic distance models (e.g., Berkeley PATH model) require estimating lead vehicle acceleration ($a_{\text{target}}$). In production radar and camera systems, acceleration is obtained by double differentiating noisy position measurements, introducing severe phase lag and high-frequency noise amplification. In contrast, standard $TTC = d / v_{\text{rel}}$ relies strictly on range and Doppler range-rate, providing clean, low-latency threat assessment without numerical derivative explosion.

3. **Deterministic dSPACE HIL Execution:**
   The ultimate project milestone is physical Hardware-in-the-Loop (HIL) execution on a dSPACE real-time platform connected over CAN bus to an electronic brake unit. Optimization-based algorithms (such as non-linear MPC) introduce cycle-time variations and non-convergence failure modes that jeopardize real-time execution at $20\text{--}50\text{ Hz}$ on automotive target processors. The FSM algorithm runs in fixed $O(1)$ cycle time with negligible CPU overhead.

4. **Direct Mapping to Graded Drive-By-Wire Actuation:**
   The VIRYA platform utilizes an electronically commanded brake actuator. A discrete multi-level FSM matches the physical capabilities of drive-by-wire hydraulic/mechanical systems, which execute calibrated pressure setpoints ($2.0$, $4.0$, $6.0\text{ m/s}^2$) far more robustly than continuously oscillating infinitesimal force demands.

---

## 4. Literature Comparison Table

The following table compiles primary, peer-reviewed papers, SAE technical papers, OEM technical disclosures, and international standards reporting numerical TTC thresholds and braking deceleration strategies.

| Paper / Source | Year | AEB Algorithm | Threat Metric | Reported TTC Threshold(s) | Braking Strategy | Reported Deceleration Values | Platform / Test Type | Relevance to Our Project | Source Citation / DOI |
| :--- | :---: | :--- | :--- | :--- | :--- | :--- | :---: | :--- | :--- |
| **Hayward, J. C.** | 1972 | Near-Miss Analysis | $TTC = d / v_{\text{rel}}$ | $TTC \le 1.0\text{--}1.5\text{ s}$ classified as danger threshold | Manual braking analysis | Observational traffic data | Traffic field camera studies | Foundational derivation of the $d/v_{\text{rel}}$ threat equation | *Highway Research Record*, No. 384, pp. 24–34 |
| **Lee, D. N.** | 1976 | Visual Tau Control | $\tau \approx TTC$ | Behavioral braking onset at $\tau \approx 2.0\text{--}3.5\text{ s}$ | Driver-regulated deceleration | Continuous driver-modulated braking | Visual perception experiments | Psychological basis for human-like warning & braking timing | *Perception*, Vol. 5, No. 4, pp. 437–459. [DOI: 10.1068/p050437](https://doi.org/10.1068/p050437) |
| **van der Horst, R.** | 1990 | Driver Behavior Modeling | Minimum TTC ($TTC_{\min}$) | Normal braking: $2.5\text{--}4.0\text{ s}$; Critical: $< 1.5\text{ s}$ | Empirical driver braking | Variable human deceleration ($2\text{--}7\text{ m/s}^2$) | Instrumented test vehicles & field studies | Justifies Warning $\le 4.0\text{ s}$ and Emergency $\le 1.75\text{ s}$ bands | PhD Thesis / *TNO Human Factors Research Institute*, Soesterberg, NL |
| **Seiler, Song, & Hedrick** | 1998 | California PATH CAS | Warning / Critical Distance | $TTC_{\text{equiv}} \approx 1.5\text{--}2.5\text{ s}$ depending on speed | Multi-level warning & partial braking | Commanded deceleration: $0.2\text{--}0.5\text{ g}$ ($2.0\text{--}4.9\text{ m/s}^2$) | Simulation & PATH test vehicle | Early hierarchical warning and avoidance architecture | *SAE Technical Paper* 980612. [DOI: 10.4271/980612](https://doi.org/10.4271/980612) |
| **Kiefer et al. (CAMP / NHTSA)** | 1999 | Forward Collision Warning (CAMP) | Inverse TTC & Kinematic Model | Warning alert: $2.1\text{--}3.5\text{ s}$; Last-second braking: $1.2\text{--}1.8\text{ s}$ | Graded alert + driver brake assist | Normal alert: $0.3\text{ g}$ ($3.0\text{ m/s}^2$); Hard brake: $0.55\text{--}0.75\text{ g}$ ($5.4\text{--}7.4\text{ m/s}^2$) | Extensive human-subject track testing (NHTSA) | Primary benchmark for driver warning budgets and deceleration bands | *NHTSA Report* DOT HS 808 964, Washington, D.C. |
| **Tamura et al. (Honda R&D)** | 2001 | Preview Brake Assist | Radar Range & Doppler TTC | Stage 1: $2.5\text{--}3.0\text{ s}$; Stage 2: $1.5\text{--}2.0\text{ s}$; Stage 3: $\le 1.0\text{ s}$ | Preview pad clearance take-up + progressive braking | Pad pre-fill ($\approx 0.05\text{ g}$), light braking ($0.25\text{ g}$), full assist ($0.6\text{ g}$) | Instrumented passenger car track tests | Demonstrates physical hydraulic latency reduction via staged braking | *SAE Technical Paper* 2001-01-0357. [DOI: 10.4271/2001-01-0357](https://doi.org/10.4271/2001-01-0357) |
| **Sugimoto, Y. & Sauer, C. (Honda)** | 2005 | Collision Mitigation Brake System (CMBS) | TTC Multi-Stage FSM | **Stage 1:** $\sim 3.0\text{ s}$<br>**Stage 2:** $\sim 2.0\text{ s}$<br>**Stage 3:** $\sim 1.0\text{ s}$ | 3-Stage: Audio/visual warning $\to$ Tactile seatbelt tug + light brake $\to$ Full emergency brake | **Stage 2:** $\sim 0.25\text{ g}$ ($2.5\text{ m/s}^2$)<br>**Stage 3:** $\sim 0.6\text{ g}$ ($5.9\text{--}6.0\text{ m/s}^2$) | Production vehicle (Honda Inspire/Legend) & Simulation | **Direct precedent** for our $3.0\text{ s}$ L1 threshold and $-6.0\text{ m/s}^2$ max deceleration | *19th ESV Conference*, Washington D.C., Paper No. 05-0148 |
| **Schittenhelm, H. (Mercedes-Benz)** | 2006 | PRE-SAFE Brake | Multi-Stage Autonomous Brake | **Warning:** $2.6\text{ s}$<br>**Stage 1:** $1.6\text{ s}$<br>**Stage 2:** $0.6\text{ s}$ | Cascaded: Warning $\to$ Partial Autonomous Braking $\to$ Full Autonomous Emergency Braking | **Stage 1:** $0.4\text{ g}$ ($4.0\text{ m/s}^2$)<br>**Stage 2:** Up to $1.0\text{ g}$ ($9.8\text{ m/s}^2$) | Production vehicle (Mercedes S-Class / CL-Class) | **Direct precedent** for our $-4.0\text{ m/s}^2$ Stage 2 deceleration level | *5th Int. Conf. on European Road Safety*, Berlin, 2006 |
| **ISO 15623** | 2013 | Forward Vehicle Collision Warning (FVCWS) | TTC & Kinematic Range Rate | Preliminary Warning: $2.5\text{--}3.5\text{ s}$; Critical Warning: $1.4\text{--}2.0\text{ s}$ | Multi-level acoustic/visual/haptic alerts | N/A (Warning only; braking assist recommended) | International Standard (Bench & Track) | Sets formal timing standards for multi-level warning before braking | *ISO 15623:2013*, Geneva, Switzerland |
| **ISO 22839** | 2013 | Forward Vehicle Collision Mitigation (FVCMS) | TTC & Extended TTC (ETTC) | Mitigation braking **prohibited** above $4.0\text{ s}$; active intervention $\le 2.0\text{--}2.5\text{ s}$ | Graded mitigation braking prior to unavoidable crash | Partial: $0.2\text{--}0.4\text{ g}$ ($2.0\text{--}3.9\text{ m/s}^2$); Full: vehicle limit | International Standard (Light & Heavy Vehicles) | **Direct standard ceiling**: Defines $4.0\text{ s}$ as maximum upper bound for mitigation | *ISO 22839:2013*, Geneva, Switzerland |
| **Falcone et al.** | 2008 | Model Predictive Control (MPC) | Optimization cost over predicted distance | Dynamic horizon ($TTC_{\text{eff}} \approx 1.2\text{--}2.2\text{ s}$) | Continuous closed-loop deceleration & steering | Continuous profile bounded by tire friction ($|a| \le 8.0\text{ m/s}^2$) | Simulation & dSPACE rapid prototyping car | Benchmark for advanced predictive control vs discrete FSM | *IEEE Trans. Control Systems Tech.*, 16(5), pp. 994–1004. [DOI: 10.1109/TCST.2008.912117](https://doi.org/10.1109/TCST.2008.912117) |
| **UNECE Regulation No. 152** | 2020 | Advanced Emergency Braking Systems (AEBS) | Car-to-Car Scenario Speed & Clearance | Warning $\ge 0.8\text{ s}$ prior to emergency brake onset | Two-phase: Warning phase followed by Emergency Braking phase | Mean fully developed deceleration $\ge 5.0\text{ m/s}^2$ on high-adhesion surface | Type approval vehicle track testing (CCR, CCRm) | Regulatory benchmark for passenger cars; defines emergency braking phase criteria | *UN Regulation No. 152*, E/ECE/TRANS/505/Rev.3/Add.151 |

---

## 5. Parameter-by-Parameter Justification

This section systematically reviews all seven core numerical parameters in the project controller against the published literature.

### 5.1 Parameter Justification Table

| Parameter | Current Value | Literature Evidence | Evidence Strength | Project Status | What Must Be Validated |
| :--- | :---: | :--- | :---: | :---: | :--- |
| **Warning TTC** | **$4.00\text{ s}$** | ISO 22839 explicitly establishes that mitigation braking shall not trigger above $4.0\text{ s}$ TTC, marking $4.0\text{ s}$ as the international standard upper boundary for active intervention and advisory alerts. van der Horst (1990) observes normal unalerted driver braking onset occurs between $2.5\text{ s}$ and $4.0\text{ s}$. In contrast, production passenger cars (Mercedes-Benz PRE-SAFE: $2.6\text{ s}$; Euro NCAP: $2.1\text{--}2.5\text{ s}$) initiate warnings later to minimize driver nuisance in dense urban traffic. | **Moderate** *(Supported as an advisory upper bound by ISO 22839 and human reaction studies, but conservative relative to OEM production calibrations).* | **Retain provisionally** | Validate driver false-alarm and nuisance rate in urban/campus following on the VIRYA platform. |
| **Level 1 TTC** | **$3.00\text{ s}$** | Directly supported by Honda's Collision Mitigation Brake System (CMBS) production architecture (Sugimoto & Sauer, 2005), which triggers Stage 1 audio/visual alerts and tactile pre-braking at $TTC = 3.0\text{ s}$. Tamura et al. (2001) also benchmark preliminary preview braking between $2.5\text{ s}$ and $3.0\text{ s}$. | **Strong** *(Directly matches Honda CMBS Stage 1 production calibration).* | **Retain provisionally** | Verify that $3.00\text{ s}$ activation does not cause premature braking during normal cut-ins or turning maneuvers. |
| **Level 2 TTC** | **$2.25\text{ s}$** | **Not directly supported as an explicit $2.25\text{ s}$ value in the reviewed literature.** Literature defines moderate partial braking within a broader band: CAMP/NHTSA (Kiefer et al., 1999) identifies alert-to-braking transitions at $2.1\text{--}2.4\text{ s}$; Honda CMBS Stage 2 triggers at $2.00\text{ s}$; Mercedes PRE-SAFE initiates partial braking at $1.60\text{ s}$. The project value of $2.25\text{ s}$ was selected as an *engineering design midpoint* between $3.00\text{ s}$ and $1.75\text{ s}$. | **Not directly supported** *(Falls within the plausible literature window of $1.6\text{--}2.4\text{ s}$, but the exact figure $2.25\text{ s}$ is an internal design assumption).* | **Requires sensitivity study** | Assess whether adjusting $TTC_{L2}$ between $2.0\text{ s}$ and $2.4\text{ s}$ optimizes stopping clearance vs deceleration jerk. |
| **Level 3 TTC** | **$1.75\text{ s}$** | Corresponds to the upper boundary of critical "Last-Second Braking" defined by CAMP (Kiefer et al., 1999: $1.2\text{--}1.8\text{ s}$) and ISO 15623 imminent threat threshold ($1.4\text{--}2.0\text{ s}$). However, production passenger cars delay full autonomous emergency braking significantly longer (Honda CMBS Stage 3: $1.0\text{ s}$; Mercedes-Benz PRE-SAFE: $0.6\text{ s}$) to guarantee that a human driver has completely failed to steer or brake before taking full vehicle control. | **Moderate** *(Supported as an imminent danger threshold in research studies, but earlier than commercial passenger car last-resort autonomous braking).* | **Retain provisionally** | Confirm whether $1.75\text{ s}$ provides sufficient stopping budget under platform-specific actuator build-up lag ($\tau$). |
| **Level 1 Deceleration** | **$-2.00\text{ m/s}^2$** ($\approx 0.20\text{ g}$) | Strongly supported across literature and OEM systems. Sugimoto & Sauer (2005) report Honda CMBS Stage 2 tactile deceleration of $\sim 0.25\text{ g}$ ($2.5\text{ m/s}^2$). CAMP studies (Kiefer et al., 1999) classify $0.2\text{--}0.3\text{ g}$ ($2.0\text{--}3.0\text{ m/s}^2$) as the ideal haptic alert deceleration that notifies the driver without causing loss of control or unrestrained occupant pitch. ISO 22839 defines light mitigation braking within $0.2\text{--}0.4\text{ g}$. | **Strong** *(Widely corroborated across standards, academic testing, and production systems as optimal haptic/light braking).* | **Retain provisionally** | Validate minimum controllable brake pressure resolution on the VIRYA drive-by-wire hydraulic unit. |
| **Level 2 Deceleration** | **$-4.00\text{ m/s}^2$** ($\approx 0.41\text{ g}$) | Directly supported by Mercedes-Benz PRE-SAFE Brake Stage 1 partial braking specifications (Schittenhelm, 2006; Breuer et al., 2007), which autonomously commands exactly $0.4\text{ g}$ ($3.92\text{ m/s}^2 \approx 4.0\text{ m/s}^2$) at $TTC = 1.6\text{ s}$. CAMP (1999) defines $0.35\text{--}0.45\text{ g}$ as the limit of comfortable emergency braking for belted occupants. | **Strong** *(Exact match to Mercedes-Benz PRE-SAFE partial autonomous braking calibration).* | **Retain provisionally** | Evaluate chassis suspension pitch and passenger comfort during $-4.0\text{ m/s}^2$ step commands. |
| **Level 3 Deceleration** | **$-6.00\text{ m/s}^2$** ($\approx 0.61\text{ g}$) | Directly supported by Honda CMBS Stage 3 emergency braking specifications (Sugimoto & Sauer, 2005), which deploys $\sim 0.6\text{ g}$ ($5.88\text{ m/s}^2 \approx 6.0\text{ m/s}^2$) emergency braking. UNECE R152 requires a mean fully developed deceleration $\ge 5.0\text{ m/s}^2$ on dry roads. While passenger cars with modern ABS on dry asphalt can achieve $8.0\text{--}10.0\text{ m/s}^2$, $6.0\text{ m/s}^2$ is widely recognized as a robust emergency ceiling that maintains directional stability across variable road surface friction ($\mu \approx 0.7\text{--}0.85$). | **Strong** *(Directly aligns with Honda CMBS Stage 3 and satisfies UNECE R152 deceleration capability floors).* | **Requires platform measurement** | Measure maximum achievable chassis deceleration on the VIRYA platform given motor/friction brake blending and tire adhesion. |

---

## 6. Limitations of Literature Transfer to the VIRYA Platform

A critical failure mode in autonomous system development is the direct, unvalidated transfer of published passenger car parameters to specialized, lightweight, or low-speed platforms. 

The published parameters reviewed in Section 4 and 5 were calibrated for **conventional passenger cars** (e.g., Mercedes-Benz S-Class, Honda Legend, Ford Taurus) operating at highway speeds ($50\text{--}130\text{ km/h}$) with gross vehicle mass of $1500\text{--}2200\text{ kg}$, hydraulic vacuum/electro-hydraulic brake boosters, and professional belted adult occupants.

The **VIRYA platform** presents fundamental physical and operational divergences that prevent automatic parameter adoption:

```
[ Conventional Passenger Car Literature ]         [ VIRYA Low-Speed DBW Platform ]
  • Mass: 1500 - 2200 kg                            • Mass: Significantly lighter electric chassis
  • High-Speed ODD: 50 - 130 km/h                   • Low-Speed ODD: 15 - 30 km/h (Campus/Urban)
  • Vacuum / High-Power Hydraulic ESC               • Custom Drive-by-Wire Actuation / Regen Blending
  • Long Stopping Distance: 30 - 80 m               • Short Stopping Distance: 2 - 8 m
  • Severe High-Speed Occupant Inertia              • Lower Impact Kinetic Energy (E_k proportional to v^2)
```

### 6.1 Key Divergence Factors

1. **Mass and Inertia Discrepancies:**
   Passenger cars possess significant longitudinal inertia. Lightweight electric/shuttle platforms exhibit much faster deceleration onset for equivalent hydraulic clamping pressures, but may also suffer severe wheel lock-up or pitch instability if large deceleration steps are commanded abruptly without anti-lock braking modulation.

2. **Drive-by-Wire Actuator Dynamics:**
   Passenger car literature assumes mature, highly optimized Electronic Stability Control (ESC) hydraulic units capable of building $100\text{ bar}$ of brake pressure within $150\text{--}250\text{ ms}$ ($\tau \approx 0.08\text{--}0.12\text{ s}$). A custom drive-by-wire platform utilizing linear electric actuators, electro-hydraulic proportional valves, or pneumatic cylinders may exhibit significantly higher time constants ($\tau = 0.20\text{--}0.30\text{ s}$) and noticeable dead-time ($t_d$). Our Step 6 combined impairment study demonstrated that $\tau = 0.30\text{ s}$ introduces up to $0.46\text{ s}$ of delay in achieving 90% braking force, directly consuming the safety margin provided by early TTC thresholds.

3. **Operational Design Domain (ODD) Speed Range:**
   Literature thresholds ($TTC = 2.6\text{ s}, 1.6\text{ s}, 0.6\text{ s}$) are tuned for highway and rural driving where closing speeds exceed $20\text{ m/s}$ ($72\text{ km/h}$). At low operational speeds ($15\text{ km/h} \approx 4.17\text{ m/s}$), a $4.00\text{ s}$ TTC represents a physical separation distance of:
   $$d = TTC \cdot v_{\text{rel}} = 4.00\text{ s} \cdot 4.17\text{ m/s} = 16.68\text{ m}$$
   In campus or dense urban environments, a vehicle maintaining a $16.7\text{ m}$ gap at $15\text{ km/h}$ will trigger nuisance warnings on parked cars, pedestrians on sidewalks, and turning vehicles, degrading user acceptance.

4. **Regenerative-Mechanical Brake Blending:**
   Electric drive-by-wire platforms often blend electric motor regenerative braking with mechanical friction calipers. Regenerative torque exhibits rapid initial response but decreases at near-zero speeds, requiring seamless handoff to friction brakes to achieve complete vehicle standstill (`STOPPED` status). Literature passenger car models typically assume pure hydraulic friction braking with constant torque-to-pressure ratios.

5. **Structural Rigidity and Occupant Kinematics:**
   A full $-6.00\text{ m/s}^2$ emergency stop in a passenger car relies on pretensioned seatbelts, front airbags, and crumple zones. On low-speed autonomous pods, shuttles, or industrial utility vehicles where occupants may be unbelted or standing, deceleration rates exceeding $-3.5\text{ to } -4.5\text{ m/s}^2$ pose severe occupant fall and injury hazards.

---

### 6.2 Separation of Assumptions vs Ground Truth

To maintain complete scientific rigor, the project parameters are partitioned into three transparent categories:

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                PARAMETER CATEGORIZATION                                │
├───────────────────────────┬─────────────────────────────┬───────────────────────────────┤
│ 1. Literature-Supported   │ 2. Engineering Assumptions  │ 3. Requires Platform Valid.   │
├───────────────────────────┼─────────────────────────────┼───────────────────────────────┤
│ • L1 TTC = 3.00 s         │ • L2 TTC = 2.25 s           │ • Actual max deceleration     │
│   (Honda CMBS Stage 1)    │   (Midpoint interpolation)  │   (Tire mu & actuator limit)  │
│ • L1 Decel = -2.00 m/s²   │ • Linear 3-stage escalation │ • Actuator time constant tau  │
│   (CAMP / ISO 22839)      │   (Equidistant steps)       │   (Step response measurement) │
│ • L2 Decel = -4.00 m/s²   │ • Warning TTC = 4.00 s      │ • Sensor noise std & latency  │
│   (Mercedes PRE-SAFE)     │   (Upper boundary ceiling)  │   (Sensor bench telemetry)    │
│ • L3 Decel = -6.00 m/s²   │                             │ • Standstill holding torque   │
│   (Honda CMBS / UN R152)  │                             │   (Caliper clamp at rest)     │
└───────────────────────────┴─────────────────────────────┴───────────────────────────────┘
```

---

## 7. Recommended Parameter Status

Based on the synthesis of peer-reviewed literature, industrial standards, and the physical constraints of the VIRYA platform, the seven active controller parameters are formally classified into actionable engineering statuses:

### Classification Categories:
- **A. Retain provisionally:** Grounded in published literature or standards; performant in closed-loop simulation; retained as active baseline pending physical validation.
- **B. Requires sensitivity study:** Engineering assumption requiring parameter sweeps to determine performance sensitivity and boundary hunting behavior.
- **C. Requires platform measurement:** Physically constrained by vehicle hardware; cannot be finalized without physical bench or vehicle telemetry.
- **D. Requires further literature evidence:** Lacking theoretical or empirical backing in published literature.

---

### Parameter Classification Registry

```
======================================================================================================
PARAM NAME           VALUE       CLASSIFICATION                      JUSTIFICATION SUMMARY
======================================================================================================
Warning TTC          4.00 s      B. Requires sensitivity study       Represents the ISO 22839 upper
                                                                     mitigation ceiling; requires
                                                                     nuisance sweep across campus ODD.

Level 1 TTC          3.00 s      A. Retain provisionally             Directly grounded in Honda CMBS
                                                                     Stage 1 architecture.

Level 2 TTC          2.25 s      B. Requires sensitivity study       Engineering assumption midpoint;
                                                                     requires sweep in [1.8 s, 2.4 s].

Level 3 TTC          1.75 s      A. Retain provisionally             Supported by CAMP critical braking
                                                                     envelope; safe simulation margin.

Level 1 Decel       -2.00 m/s²   A. Retain provisionally             Strongly supported by CAMP, ISO,
                                                                     and OEM tactile alert data.

Level 2 Decel       -4.00 m/s²   A. Retain provisionally             Directly matches Mercedes PRE-SAFE
                                                                     Stage 1 autonomous braking.

Level 3 Decel       -6.00 m/s²   C. Requires platform measurement    Satisfies UN R152 emergency floor,
                                                                     but platform tire-road friction
                                                                     and DBW authority must be measured.
======================================================================================================
```

### Action Plan for Future HIL & Platform Integration
1. **Retain Provisionally ($TTC_{L1}=3.0\text{ s}, TTC_{L3}=1.75\text{ s}, a_{L1}=-2.0\text{ m/s}^2, a_{L2}=-4.0\text{ m/s}^2$):** Keep as fixed baseline parameters during dSPACE HIL interface development to ensure deterministic behavioral continuity.
2. **Execute Urban Sensitivity Sweeps ($TTC_{\text{warn}}=4.0\text{ s}, TTC_{L2}=2.25\text{ s}$):** Conduct simulated cut-in and nuisance testing at low operational speeds ($15\text{--}30\text{ km/h}$) to assess false-activation frequency.
3. **Conduct Hardware Bench Characterization ($a_{L3}=-6.0\text{ m/s}^2, \tau$):** Mount the physical VIRYA electronic brake actuator on the test bench to measure maximum steady-state clamping force, pressure build-up time constant ($\tau$), and CAN transmission latency prior to closed-loop HIL execution.
