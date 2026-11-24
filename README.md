# Digital Twin-Based Development and HIL Validation of an Autonomous Emergency Braking (AEB) System for the VIRYA Drive-by-Wire Platform

## Team Members

| Sl. No. | Name | USN |
|---|---|---|
| 1 | Pranjwal Hukkeri | 01FE23BEC217 |
| 2 | Sejal Jartarghar | 01FE23BEC325 |

---

## 1. Problem Statement

Autonomous vehicle functions such as Automatic Emergency Braking must operate reliably under different driving and obstacle conditions. Testing these functions exclusively through physical vehicle experiments can be time-consuming, expensive, and difficult to reproduce consistently.

A structured simulation-assisted HIL validation approach can help evaluate control-system behaviour in a repeatable environment before deployment.

This project addresses the need for a Digital Twin Assisted HIL validation framework for the AEB functionality of the VIRYA DBW platform.

## 2. Objectives

- Study the VIRYA Drive-by-Wire platform and its Automatic Emergency Braking functionality.
- Understand the AEB model structure and its interaction with the vehicle system.
- Explore the dSPACE environment required for model integration and HIL validation.
- Establish communication with the dSPACE SCALEXIO hardware platform.
- Investigate the integration of Python with the ASAM XIL API for automated testing.
- Develop repeatable test scenarios for evaluating AEB functionality.
- Analyse system responses and identify discrepancies between expected and measured behaviour.
- Establish a foundation for Digital Twin Assisted functional verification.

## 3. Technologies and Tools

| Technology / Tool | Purpose |
|---|---|
| VIRYA DBW Platform | Target vehicle platform for AEB functional verification |
| Automatic Emergency Braking (AEB) Model | Control functionality under investigation |
| dSPACE SCALEXIO | Target HIL hardware platform |
| dSPACE ConfigurationDesk | Model and hardware configuration and application integration |
| dSPACE ControlDesk | Experiment environment and interaction with the dSPACE platform |
| ASAM XIL API | Intended interface for automated test execution and measurement access |
| Python | Test automation and XIL communication scripts |
| Windows Command Prompt | Network diagnostics and dSPACE service investigation |
| GitHub | Project documentation and version-controlled project repository |

## 4. Project Methodology

The planned project workflow consists of the following stages:

1. **System Study:** Understand the VIRYA DBW platform and AEB functionality.
2. **Model Exploration:** Inspect the available AEB model and understand its structure.
3. **dSPACE Configuration:** Explore model integration and hardware configuration using ConfigurationDesk.
4. **HIL Platform Integration:** Establish communication with the SCALEXIO hardware.
5. **Test Automation:** Connect Python test scripts to the dSPACE environment through the ASAM XIL API.
6. **Test Scenario Development:** Define repeatable scenarios for evaluating AEB behaviour.
7. **Functional Validation:** Execute test cases and observe system responses.
8. **Result Analysis:** Compare measured behaviour against expected AEB functionality and document findings.

## 5. Work Completed So Far

The following activities have been carried out during the project:

### 5.1 Understanding the VIRYA AEB System

- Studied the project objective and the role of Automatic Emergency Braking in the VIRYA DBW platform.
- Explored the available VIRYA AEB project files and model structure.
- Investigated the relationship between the AEB model and the intended HIL validation workflow.

### 5.2 dSPACE Environment Exploration

- Explored the available dSPACE software environment.
- Investigated ConfigurationDesk for model-function mapping, topology configuration, and application build procedures.
- Explored ControlDesk projects and experiment configurations relevant to the intended validation setup.
- Investigated the available application and platform configuration.

### 5.3 SCALEXIO Hardware Investigation

- Identified SCALEXIO as the intended target hardware platform.
- Identified the configured platform IP address as `192.168.140.10`.
- Performed network connectivity testing using the Windows `ping` command.
- Received successful ping replies with no packet loss, confirming basic network reachability.
- Investigated the dSPACE Platform Management and Device Access services.
- Verified that the relevant dSPACE services were running on the computer.

**Note:** Successful ping responses confirm network reachability only. They do not independently confirm that the SCALEXIO platform is fully registered, online, or ready for HIL execution.

### 5.4 Python and ASAM XIL API Integration

- Set up the Python environment for investigating automated test execution.
- Created and executed a Python connection script named `test_connection.py`.
- Attempted to create an XIL test bench and establish a MAPort connection.
- Investigated the XIL communication setup and its integration with the dSPACE environment.

## 6.Current Project Status

**Overall Status: In Progress – HIL Integration and Validation**

| Project Activity | Current Status |
|---|---|
| Project scope and AEB application study | Completed |
| VIRYA AEB project and model exploration | Completed at the initial investigation level |
| dSPACE software environment exploration | Completed at the initial investigation level |
| ConfigurationDesk configuration investigation | In Progress |
| ControlDesk experiment and platform investigation | In Progress |
| SCALEXIO network reachability check | Successful |
| SCALEXIO online platform integration | Pending confirmation |
| Python test script preparation | Initial setup completed |
| ASAM XIL MAPort communication | Under investigation |
| Automated AEB test execution | Not yet completed |
| HIL test execution | Pending |
| Test result analysis and validation | Pending |



## 7. Planned Future Work

The next stages of the project are:

1. Complete SCALEXIO platform registration and verify its online status.
2. Confirm the application configuration and successful deployment to the target hardware.
3. Establish a successful Python-to-dSPACE MAPort connection.
4. Develop and organize AEB test cases using a structured test-case file such as `cases.csv`.
5. Execute test scenarios involving different vehicle and obstacle conditions.
6. Capture relevant AEB signals and system responses.
7. Analyse braking behaviour and evaluate the functional response of the AEB system.
8. Document test results, observations, and conclusions.

## 8. Expected Outcomes

The intended outcomes of this project are:

- A structured Digital Twin Assisted HIL validation workflow for the VIRYA AEB application.
- Integration of the VIRYA AEB functionality with the dSPACE SCALEXIO HIL environment.
- Python-based automated test execution through the ASAM XIL API.
- Repeatable test scenarios for evaluating AEB functionality.
- Measurement and analysis of relevant system responses.
- A documented functional verification approach that can support further development and testing of autonomous vehicle systems.

## 9. Conclusion

The project has progressed through the initial study of the VIRYA AEB application, exploration of the dSPACE software environment, investigation of SCALEXIO hardware connectivity, and initial development of Python-based XIL communication.

Basic network reachability to the configured SCALEXIO IP address has been verified, and the relevant dSPACE services have been investigated. However, complete hardware integration and automated test execution remain pending.

The immediate priorities are confirming SCALEXIO online integration and establishing successful communication between Python and the dSPACE platform. Once these prerequisites are addressed, the project can proceed towards AEB test execution, functional validation, and result analysis.

---

**Project Domain:** Automotive Systems | Autonomous Vehicles | Digital Twin | Hardware-in-the-Loop Testing | Functional Verification

**Repository:** This repository contains project documentation and will be updated with relevant implementation files, test scripts, configurations, and validation results as the project progresses. ye dali thi dekh
