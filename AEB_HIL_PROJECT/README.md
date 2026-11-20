# Digital Twin Assisted HIL Validation Framework for Autonomous Vehicle Functional Verification

## Phase 1: Autonomous Emergency Braking (AEB) Local Python Validation

### Overview
Phase 1 focuses on developing and functionally verifying the Autonomous Emergency Braking (AEB) algorithm entirely in Python. The framework is designed with a strict modular architecture to allow seamless transition to dSPACE Hardware-in-the-Loop (HIL) testing in subsequent phases.

### Directory Structure
- `config/`: System parameters, safety bounds, vehicle limits, and simulation settings.
- `src/aeb_controller.py`: Pure AEB decision logic (FCW, Partial Brake, Full Brake).
- `src/vehicle_model.py`: Longitudinal vehicle dynamics model.
- `src/simulation.py`: Closed-loop simulation engine.
- `src/metrics.py`: Functional verification KPIs (Impact speed, TTC min, stopping clearance).
- `scenarios/`: Euro NCAP test scenario definitions (CCRs, CCRm, CCRb).
- `tests/`: Pytest unit tests for algorithm verification.
- `results/`: Output logs and telemetry data.
- `plots/`: Trajectory, speed, and braking performance graphs.

### HIL Readiness Architecture
All AEB control logic and vehicle plant models are completely decoupled from hardware-specific APIs. In later phases, dSPACE integration will be introduced as an external interface layer without modifying core AEB algorithms.
