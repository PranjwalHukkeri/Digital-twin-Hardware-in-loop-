# Step 11 — dSPACE HIL Architecture and Implementation Plan

**Project:** Digital Twin Assisted HIL Validation Framework for Autonomous Emergency Braking (AEB)  
**Platform Focus:** VIRYA Drive-by-Wire (DBW) Skateboard Platform  
**Document Status:** Architecture & Implementation Specification (Planning Stage — Pre-Implementation)  
**Execution Paradigm:** Strictly **No-MATLAB / No-Simulink** Toolchain  

---

## 1. Purpose of Step 11

The objective of Step 11 is to formulate the complete engineering specification, communication interface architecture, and real-time execution strategy required to transition the validated Python AEB digital twin to a **dSPACE Hardware-in-the-Loop (HIL)** validation environment without utilizing MATLAB or Simulink.

Step 11 bridges the gap between pure discrete-time software simulation (Steps 1–10) and physical real-time HIL execution (Step 12). It defines:
- The signal interchange protocols and physical/logical interfaces between Python and dSPACE.
- The allocation of computational responsibilities (controller vs. plant vs. virtual environment).
- The timing, synchronization, and determinism constraints necessary for hardware-in-the-loop integrity.
- The quantitative comparison framework used to evaluate dSPACE telemetry against the golden Python software baseline.

> [!IMPORTANT]
> **Planning Stage Declaration:**
> This document constitutes an **architecture and implementation plan**. It does **not** claim that dSPACE HIL has already been deployed, compiled, or executed. Real-time implementation, physical target flashing, and live closed-loop HIL testing belong strictly to Step 12.

---

## 2. Current Validated Python Software Baseline

The reference model ("golden digital twin") finalized in Steps 9 and 10 serves as the benchmark against which all dSPACE executions will be quantitatively audited:

### 2.1 Control Law and Architecture
- **Threat Assessment:** Time-To-Collision (TTC) computed as:
  $$\text{TTC} = \begin{cases} \dfrac{d_{\text{meas}}}{v_{\text{rel}}}, & v_{\text{rel}} > 0 \\ \infty, & v_{\text{rel}} \le 0 \end{cases}$$
  where $v_{\text{rel}} = v_{\text{ego}} - v_{\text{target}}$ and $d_{\text{meas}}$ is distance sensor feedback.
- **Hierarchical Finite State Machine (FSM):** 5 discrete states:
  - `SAFE`: Cruising, $0.0\text{ m/s}^2$ braking demand.
  - `WARNING`: Visual/acoustic alert ceiling ($TTC \le 4.00\text{ s}$), $0.0\text{ m/s}^2$ braking demand.
  - `AEB_LEVEL_1`: Light mitigation braking ($TTC \le 3.00\text{ s}$), $-2.00\text{ m/s}^2$ demand.
  - `AEB_LEVEL_2`: Medium mitigation braking ($TTC \le 2.25\text{ s}$), $-4.00\text{ m/s}^2$ demand.
  - `AEB_LEVEL_3`: Full emergency braking ($TTC \le 1.75\text{ s}$), $-6.00\text{ m/s}^2$ demand.
- **Anti-Chattering & Minimum Dwell:**
  - Active braking dwell timer: $t_{\text{dwell}} = 0.20\text{ s}$ ($4$ simulation timesteps at $dt = 0.05\text{ s}$).
  - Immediate release precedence: permitted **only** when $v_{\text{rel}} \le 0.0\text{ m/s}$ (target is no longer closing). If $TTC > TTC_{\text{warning}}$ while $v_{\text{rel}} > 0$, the controller strictly holds active braking until $t_{\text{state}} \ge 0.20\text{ s}$.

### 2.2 Dynamic Plant and Sensor Models
- **Vehicle Longitudinal Kinematics:** Discrete-time Euler forward integration ($dt = 0.05\text{ s}$) with non-reversal zero-speed latching:
  $$v(k+1) = \max\left(0, \, v(k) + a_{\text{actual}}(k) \cdot dt\right)$$
  $$d(k+1) = \max\left(0, \, d(k) - v_{\text{rel}}(k) \cdot dt\right)$$
- **Brake Actuator:** First-order lag model with time constant $\tau = 0.10\text{ s}$:
  $$a_{\text{actual}}(k+1) = a_{\text{actual}}(k) + \frac{dt}{\tau} \left(a_{\text{cmd}}(k) - a_{\text{actual}}(k)\right)$$
- **Sensor Impairment:** Zero-order-hold sampling, nominal Gaussian distance noise $\sigma = 0.05\text{ m}$, baseline latency $= 0.0\text{ s}$.

### 2.3 Software Verification State
- **Full Unit Test Suite:** **123 / 123 tests passing** via `./run_tests.sh`.
- **Operating-Envelope Validation:** **720 runs completed** across $0.50\text{--}1.39\text{ m/s}$, $1.0\text{--}6.0\text{ m}$, CCR, CCRm, and CCRb scenarios:
  - **Zero collisions** ($0 / 720$).
  - **Zero sub-0.30m separation cases** (global minimum separation: $0.3086\text{ m}$).
  - Mean FSM transitions: $7.58$; mean triplet reversals: $0.88$.

---

## 3. What is Already Known About the Available dSPACE Computer

Based strictly on previously established project evidence:
1. **Dedicated Host Workstation:** A dedicated laboratory PC running a 64-bit Windows operating system is physically connected to the dSPACE target environment.
2. **HIL Role:** The dSPACE hardware system is designated as the real-time simulation/I/O processing node for the VIRYA DBW validation framework.
3. **No-MATLAB Mandate:** The project development environment operates under an explicit constraint: **no MATLAB and no Simulink licenses or toolchains are to be utilized for code generation or model execution**.
4. **Signal Mapping Compatibility:** Standard automotive communication conventions (CAN bus, Ethernet, serial/UART, and analog/digital I/O channels) have already been defined at the signal level in `docs/aeb_performance_evaluation.md`.
5. **Python Reference Availability:** The Python digital twin is fully containerized/modularized within `/home/cim/Desktop/IUP26/AEB_HIL_PROJECT`, providing portable reference classes (`AEBController`, `VehicleModel`, `BrakeActuator`, `SensorModel`, `PerformanceEvaluator`) ready for standalone C porting, socket interfacing, or native API bindings.

---

## 4. What Remains Unknown (To Be Confirmed on the Connected dSPACE Setup)

In accordance with rigorous engineering standards, capabilities not explicitly confirmed in prior evidence are formally classified as **unconfirmed** until verified on the active machine:

| System Attribute | Status | Specific Items to Confirm on dSPACE Setup |
| :--- | :--- | :--- |
| **Real-Time Target Hardware Model** | *To be confirmed from existing setup* | Specific processor platform: e.g., SCALEXIO processing unit, MicroAutoBox (II or III), DS1104 R&D Controller Board, or DS1006 Processor Board. |
| **Installed dSPACE Software Release** | *To be confirmed from existing setup* | Installed management suite and version: e.g., ControlDesk (v7.x / v202x), ConfigurationDesk, or legacy dSPACE software tools. |
| **Installed Toolchains & Compilers** | *To be confirmed from existing setup* | Available standalone C/C++ cross-compilers (e.g., dSPACE GCC cross-compiler for QNX/Linux RTOS targets, DS1104 compiler tools) without MATLAB/Simulink coder. |
| **Python & Open-API Support** | *To be confirmed from existing setup* | Installed Python interpreters and presence of dSPACE Python packages (e.g., `dspace.controldesk`, Platform API, or ASAM XSD/XCP APIs). |
| **Physical I/O & Bus Interfaces** | *To be confirmed from existing setup* | Presence and cabling of physical CAN interfaces (e.g., high-speed CAN / CAN FD channels), Ethernet ports (real-time Ethernet vs. host LAN), and analog/digital breakout panels. |
| **Deployment Privileges** | *To be confirmed from existing setup* | Local administrative permissions, hardware flashing access rights, and target execution license status. |

---

## 5. Proposed No-MATLAB / No-Simulink HIL Architecture

To satisfy the strict no-MATLAB/no-Simulink requirement, the HIL framework relies on direct, open, standards-compliant software and communication architectures:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          PYTHON HOST ENVIRONMENT                            │
│                                                                             │
│   ┌───────────────────────────┐         ┌───────────────────────────────┐   │
│   │     Scenario Engine       │         │    Simulation Engine / Plant  │   │
│   │   (CCR / CCRm / CCRb)     │         │   (Vehicle, Actuator, Noise)  │   │
│   └─────────────┬─────────────┘         └───────────────▲───────────────┘   │
│                 │ (Trajectory State)                    │ (Brake Demand)    │
│                 ▼                                       │                   │
│   ┌─────────────────────────────────────────────────────┴───────────────┐   │
│   │           HIL Communication Bridge (Open Socket / CAN API)          │   │
│   │          (UDP/IP, Serial, or Vector/PEAK/dSPACE Python API)         │   │
│   └──────────────────────────────────┬──────────────────────────────────┘   │
└──────────────────────────────────────┼──────────────────────────────────────┘
                                       │ Real-Time Bus / Network
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                       dSPACE REAL-TIME ENVIRONMENT                          │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                  dSPACE Real-Time Target / I/O                      │   │
│   │                                                                     │   │
│   │   [Option A]: Standalone C AEB Controller Executing on Target RTOS  │   │
│   │      - Standalone C translation of `src/aeb_controller.py`          │   │
│   │      - Compiled via dSPACE target C compiler (No Simulink)          │   │
│   │                                                                     │   │
│   │   [Option C]: dSPACE Hardware I/O Loopback & Signal Condition HIL   │   │
│   │      - Hardware DAC/ADC or CAN bus loopback injection               │   │
│   │      - Physical latency, jitter, and electrical signal validation   │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 6. Possible Execution Arrangements

Three distinct execution architectures are evaluated based on possible laboratory hardware constraints:

### Architecture A: AEB Controller on dSPACE, Plant in Python (Recommended Target)
- **Concept:** The AEB state machine and threat assessment logic are compiled as a standalone C module and executed directly on the dSPACE real-time processor. The vehicle dynamics, brake actuator, sensor noise, and scenario generation execute in the Python digital twin.
- **Communication:** Python sends virtual radar/wheel-speed data ($d_{\text{meas}}, v_{\text{ego}}, v_{\text{target}}$) to dSPACE via CAN or UDP socket; dSPACE returns the commanded deceleration $a_{\text{cmd}}$ and FSM state.
- **Feasibility without MATLAB:** Highly feasible if the dSPACE environment provides a C compiler or Application C API for the real-time target.

### Architecture B: Plant and Controller Components Both on dSPACE
- **Concept:** The entire digital twin (plant kinematics, actuator lag, sensor model, and AEB controller) is ported to standalone C and deployed entirely onto the dSPACE real-time target. Python serves solely as the asynchronous supervisory logger and scenario orchestrator.
- **Feasibility without MATLAB:** Moderate to complex. Requires porting the entire dynamic plant and scenario state machines into C.

### Architecture C: Signal-Level HIL Using Available dSPACE Interfaces (Fallback / Hardware Verification)
- **Concept:** The complete closed-loop software remains in Python, but critical control signals (sensor distance, commanded brake pressure, wheel speed) are routed through physical dSPACE hardware I/O channels (e.g., physical CAN bus loopback, Analog Out $\to$ Analog In) to introduce physical bus latency, electrical jitter, and real-time hardware transmission dynamics.
- **Feasibility without MATLAB:** Extremely high. Requires only standard Python bus communication libraries (`python-can`, `socket`, `pyserial`) and basic dSPACE I/O routing.

---

## 7. Recommended Architecture Based Only on Available Evidence

### **Primary Recommendation: Phased Transition (Architecture C $\to$ Architecture A)**

1. **Phase 1 (Immediate Step 11/12 Setup): Architecture C (Signal-Level HIL Bridge)**
   - Maintain the validated Python digital twin intact.
   - Establish physical or network signal interchange through the dSPACE computer's communication hardware (CAN bus or high-speed UDP Ethernet socket).
   - Route sensor data and brake commands through the physical dSPACE communication hardware to quantify transmission latency, packet drop, and deterministic clock jitter.
2. **Phase 2 (Controller Porting): Architecture A (Standalone C AEB Controller)**
   - Translate [`src/aeb_controller.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/src/aeb_controller.py) into pure ANSI C (`aeb_controller.c` and `aeb_controller.h`) without external dependencies.
   - Compile directly using the dSPACE platform's native C compiler or execute as an external real-time C process managed by dSPACE I/O.
   - Close the loop with the Python plant.

---

## 8. Python-Side Components

The Python host architecture maintains the modular component structure established in the reference codebase:

| Component | Source Reference | Role in HIL Framework |
| :--- | :--- | :--- |
| **Scenario Generator** | [`scenarios/scenarios.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/scenarios/scenarios.py) | Generates ground-truth trajectories for CCR, CCRm, and CCRb test cases. |
| **Vehicle Model** | [`src/vehicle_model.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/src/vehicle_model.py) | Computes forward longitudinal velocity and position integration at $dt = 0.05\text{ s}$. |
| **Sensor Model** | [`src/sensor_model.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/src/sensor_model.py) | Injects zero-order-hold sampling, calibrated Gaussian noise ($\sigma = 0.05\text{ m}$), and transport delay. |
| **Brake Actuator** | [`src/brake_actuator.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/src/brake_actuator.py) | Models first-order electro-hydraulic / DBW brake lag ($\tau = 0.10\text{ s}$). |
| **HIL Interface Bridge** | *New Module in Step 12* (`src/hil_bridge.py`) | Serializes outgoing sensor packets and unpacks incoming dSPACE brake commands. |
| **Telemetry Logger** | [`src/simulation.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/src/simulation.py) | Synchronously logs millisecond timestamps, plant states, sensor values, and controller states. |
| **Performance Evaluator** | [`src/metrics.py`](file:///home/cim/Desktop/IUP26/AEB_HIL_PROJECT/src/metrics.py) | Evaluates the standard 14 project metrics and scenario-aware stopping feasibility. |

---

## 9. dSPACE-Side Components

The dSPACE environment will host the real-time target and hardware acquisition layers:

1. **Real-Time Execution Target (Hardware Layer):**
   - Executes real-time tasks at strict periodic clock rates ($T_s \le 10\text{ ms}$ or $50\text{ ms}$).
   - Guarantees deterministic task preemption and zero-jitter timing.
2. **I/O & Communication Interface Layer:**
   - Physical CAN Controller or Ethernet MAC managing signal transmission.
   - Transmit/Receive buffers for AEB control frames.
3. **Controller Processing Block:**
   - Evaluates TTC and executes the 5-state hierarchical FSM (`SAFE`, `WARNING`, `L1`, `L2`, `L3`) with $0.20\text{ s}$ dwell retention.
4. **Hardware Experiment & Monitoring Layer:**
   - Logs target-side execution telemetry, cycle overruns, task turnaround times, and bus traffic for post-test synchronization.

---

## 10. Conceptual Signal Flow

The end-to-end signal flow adheres strictly to the project control architecture:

```
         Scenario / Reference Input
                     ↓
  dSPACE Real-Time Plant or Plant Interface
                     ↓
               Sensor Signals
        (d_meas, v_ego, v_target)
                     ↓
               AEB Controller
          (TTC Threat Assessment + FSM)
                     ↓
               Brake Command
             (a_cmd, FSM_State)
                     ↓
         Brake Actuator / Plant
       (tau = 0.10s Lag + Kinematics)
                     ↓
         Vehicle State Feedback
            (v_ego, v_rel, d)
                     ↓
          Logging and Comparison
```

---

## 11. Required Signals and Direction Table

The HIL interface exchange requires 9 primary signals categorized by data direction:

| Signal Name | Physical Meaning | Data Type | Units | Direction | Python Source / Target | dSPACE Source / Target |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`timestamp`** | Master simulation time | `float64` | $\text{s}$ | Python $\to$ dSPACE | Master clock | Target sync reference |
| **`ego_speed`** | Measured longitudinal ego speed | `float32` | $\text{m/s}$ | Python $\to$ dSPACE | `SensorModel(v_ego)` | Controller input |
| **`target_speed`** | Measured lead vehicle speed | `float32` | $\text{m/s}$ | Python $\to$ dSPACE | `SensorModel(v_target)` | Controller input |
| **`distance`** | Measured longitudinal clearance | `float32` | $\text{m}$ | Python $\to$ dSPACE | `SensorModel(d)` | Controller input |
| **`relative_speed`** | Computed closing velocity | `float32` | $\text{m/s}$ | Internal / Derived | $v_{\text{ego}} - v_{\text{target}}$ | Target-computed $v_{\text{rel}}$ |
| **`ttc`** | Time-To-Collision threat | `float32` | $\text{s}$ | Internal / Derived | $d / v_{\text{rel}}$ | Target-computed TTC |
| **`brake_command`** | Commanded deceleration demand | `float32` | $\text{m/s}^2$ | dSPACE $\to$ Python | `BrakeActuator.step()` | Controller output ($a_{\text{cmd}}$) |
| **`aeb_state`** | Active FSM state enum (0..4) | `uint8` | `enum` | dSPACE $\to$ Python | Telemetry / Logger | Controller FSM state |
| **`actual_decel`** | Physical vehicle deceleration | `float32` | $\text{m/s}^2$ | Python $\to$ Logging | `BrakeActuator.actual` | Verification logger |

---

## 12. Timing, Sampling, and Real-Time Constraints

To prevent aliasing, numerical divergence, or actuator clipping, the HIL execution must strictly respect established project timing parameters:

1. **Fundamental Simulation Timestep ($dt = 0.05\text{ s} = 50\text{ ms}$):**
   - The Python digital twin advances in exact $50\text{ ms}$ increments ($20\text{ Hz}$).
   - dSPACE communication frames must synchronize at intervals of $50\text{ ms}$ (or an integer sub-multiple, e.g., $10\text{ ms}$ inner loop / $100\text{ Hz}$).
2. **Actuator Bandwidth Preservation ($\tau = 0.10\text{ s} = 100\text{ ms}$):**
   - The first-order brake lag has a time constant of $\tau = 0.10\text{ s}$.
   - Nyquist and control criteria require communication latency to remain significantly below $\tau$ (target transmission latency: $T_{\text{latency}} < 10\text{ ms}$).
3. **Minimum Dwell Retention ($t_{\text{dwell}} = 0.20\text{ s}$):**
   - The dSPACE controller must guarantee exact state dwell timing ($4$ timesteps of $50\text{ ms}$), verifying that no single-step ($0.05\text{ s}$) pulses escape to the actuator.
4. **Sensor Noise Robustness ($\sigma = 0.05\text{ m}$):**
   - The dSPACE threat calculation must process noisy distance readings without numeric overflow or infinite division errors when $v_{\text{rel}} \to 0$.

---

## 13. HIL Test Scenarios

The HIL test matrix will replicate the validated Python operational cases:

1. **CCR (Car-to-Car Rear Stationary Target):**
   - Speeds: $v_{\text{ego}} \in \{0.50, 1.00, 1.39\}\text{ m/s}$, stationary target ($v_{\text{target}} = 0$).
   - Distances: $d_0 \in [1.0, 6.0]\text{ m}$.
   - Criterion: Full vehicle standstill ($v_{\text{ego}} = 0.0$) with safe residual clearance and zero collisions.
2. **CCRm (Car-to-Car Rear Moving Target):**
   - Speeds: $v_{\text{ego}} \in \{0.50, 1.00, 1.39\}\text{ m/s}$, moving target ($v_{\text{target}} = 0.5 \cdot v_{\text{ego}}$).
   - Distances: $d_0 \in [1.0, 6.0]\text{ m}$.
   - Criterion: Non-collision speed matching ($v_{\text{rel}} \le 0.0\text{ m/s}$) and separation expansion.
3. **CCRb (Car-to-Car Rear Braking Target):**
   - Initial approach at identical speeds, followed by lead target braking at $-2.00\text{ m/s}^2$.
   - Criterion: Smooth escalation through FSM levels and safe avoidance.
4. **Low-Speed Critical Boundary Case:**
   - CCRm, $v_{\text{ego}} = 0.50\text{ m/s}$, $d_0 = 1.50\text{ m}$, Seed 48 equivalent.
   - Criterion: Minimum separation must remain $\ge 0.70\text{ m}$, FSM transitions $\le 6$, triplet reversals $= 0$.
5. **Sensor-Noise Impairment Case:**
   - Robustness evaluation under distance sensor noise $\sigma \in [0.01, 0.20]\text{ m}$.
6. **Actuator-Lag Sensitivity Case:**
   - Actuator lag variation $\tau \in [0.00, 0.30]\text{ s}$ across physical communication channels.

---

## 14. Python-versus-dSPACE Comparison Metrics

Quantitative parity between Python and dSPACE will be audited across the 11 key telemetry indicators:

| Comparison Metric | Permissible Parity Tolerance | Evaluation Criterion |
| :--- | :--- | :--- |
| **`collision_status`** | Exact match (`False` == `False`) | Zero collisions permitted across all runs |
| **`min_distance`** | $|\Delta d_{\text{min}}| \le 0.02\text{ m}$ | Must track Python clearance within sensor noise resolution |
| **`final_ego_speed`** | $|\Delta v_{\text{final}}| \le 0.05\text{ m/s}$ | Verifies identical terminal stopping kinematics |
| **`final_relative_velocity`** | $|\Delta v_{\text{rel,final}}| \le 0.05\text{ m/s}$ | Verifies speed-matching convergence in CCRm |
| **`ttc_at_activation`** | $|\Delta TTC_{\text{act}}| \le 0.05\text{ s}$ | Verifies identical kinematic boundary triggering |
| **`aeb_activation_time`** | $|\Delta t_{\text{act}}| \le 0.05\text{ s}$ ($1\text{ step}$) | Verifies activation synchronicity |
| **`fsm_state_transitions`** | $|\Delta N_{\text{trans}}| \le 1$ transition | Verifies FSM stability and identical escalation path |
| **`triplet_reversal_count`** | Exact match ($\Delta = 0$) | Verifies complete absence of chattering |
| **`max_commanded_decel`** | Exact match ($\pm 0.00\text{ m/s}^2$) | Discrete demand levels must match $-2.0, -4.0, -6.0$ |
| **`max_actual_decel`** | $|\Delta a_{\text{max}}| \le 0.10\text{ m/s}^2$ | Verifies actuator model response parity |
| **`stopping_feasible`** | Exact match (`True` == `True`) | Scenario-aware feasibility must agree 100% |

---

## 15. Synchronization and Calibration Plan

1. **Clock Synchronization:**
   - The Python host acts as the primary timekeeper by embedding monotonic millisecond timestamps in each transmission frame.
   - dSPACE echoes the timestamp back in its response packet, enabling continuous round-trip time (RTT) calculation and clock drift compensation.
2. **Zero-Order-Hold (ZOH) Latching:**
   - If a dSPACE response packet is delayed beyond $dt = 0.05\text{ s}$, the Python plant latches the previous brake command for exactly one step, logging a communication overrun event.
3. **Transport Latency Calibration:**
   - A dedicated pre-test ping/echo routine will measure baseline channel latency ($T_{\text{comm}}$). If $T_{\text{comm}} > 5\text{ ms}$, latency compensation will be activated.

---

## 16. Risks and Limitations

1. **Lack of Native C Compiler on Host:** If the dSPACE Windows machine lacks an installed ANSI C compiler for the real-time target, standalone C deployment (Architecture A) cannot be compiled directly without external build tools.
2. **Operating System Jitter:** Standard Windows networking (non-real-time socket stack) can introduce non-deterministic packet delays if using UDP. A dedicated point-to-point network adapter or CAN interface will mitigate this.
3. **Model Parity Discrepancies:** Floating-point rounding differences between Python (`float64`) and real-time target hardware (`float32`) could theoretically alter threshold boundary crossings by $\pm 1$ timestep under heavy sensor noise.

---

## 17. Step 11 Acceptance Criteria

Step 11 is formally complete and accepted when:
- [x] The architecture document (`docs/dspace_hil_architecture_plan.md`) is fully drafted without claiming premature HIL completion.
- [x] No MATLAB or Simulink dependencies are required or referenced for implementation.
- [x] All 9 primary signals and directions are specified with data types and timing constraints.
- [x] The 11 comparison metrics and tolerance bounds are formally established.
- [x] The contradiction in the Step 10 report is resolved, clearly separating the original baseline experiment from the follow-up correction.
- [x] The production Python codebase remains intact with 123/123 tests passing.

---

## 18. Exact Next Action for the Project

### **Single Action on the Connected dSPACE Computer:**
> Open a Windows command terminal (`cmd.exe` or PowerShell) on the connected dSPACE computer and run:
> ```cmd
> dir "%ProgramFiles%\dSPACE*" /s /b
> ```
> *(or inspect the installed programs / environment variables) to confirm the exact installed dSPACE software package and version without altering any files or configurations.*
