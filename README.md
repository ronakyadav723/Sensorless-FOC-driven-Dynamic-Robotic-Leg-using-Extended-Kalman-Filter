# Sensorless FOC-Driven Dynamic Robotic Leg using Extended Kalman Filter

> A mathematically rigorous Sensorless Field-Oriented Control (FOC) architecture for a 2-DOF robotic leg, driven entirely by Extended Kalman Filter (EKF) rotor angle/speed estimation — no physical position encoders required.

[![MATLAB](https://img.shields.io/badge/MATLAB-Simulink%20%2F%20Simscape-orange)](https://www.mathworks.com/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-lightgrey)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-MIT-green)](#license)

---

## Overview

This project replaces idealized physics-engine actuators with a **high-fidelity electrical digital twin** of a Permanent Magnet Synchronous Motor (PMSM), controlled without a physical rotor position sensor. Rotor angle (θ̂ₑ) and angular velocity (ω̂ₑ) are estimated in real time by an Extended Kalman Filter operating purely on measured phase currents and applied voltages, using the motor's back-EMF as the observable.

The estimated states drive a complete FOC cascade (Clarke/Park transforms, cascaded current-loop PI control, SVPWM) inside a Simscape Electrical power-stage model. This electrical simulation is co-simulated in real time with a ROS 2 / Gazebo mechanical simulation of a 2-DOF robotic leg, closing the loop between electrical torque generation and mechanical joint dynamics — including gait trajectory tracking, payload loading, and disturbance events such as simulated foot-strike impact.

**Core goal:** validate that a sensorless EKF-based FOC drive can maintain angle lock and avoid stalling under dynamic, real-world mechanical loading — entirely without an encoder.

---

## Key Features

- **Continuous-time PMSM state-space model** in the stationary (α-β) reference frame
- **Extended Kalman Filter observer** estimating rotor electrical angle and speed from back-EMF, with innovation gating for outlier/measurement rejection
- **Cascaded FOC current control** (Id/Iq PI loops) driven exclusively by EKF-estimated angle — no encoder feedback anywhere in the loop
- **Simscape Electrical digital twin** of the 3-phase inverter + PMSM power stage
- **Bidirectional ROS 2 co-simulation bridge** between MATLAB/Simulink (electrical domain) and Gazebo (mechanical domain), using the ROS Toolbox for native ROS 2 pub/sub inside Simulink
- **2-DOF robotic leg** (hip + knee) modeled in URDF with a Gazebo `ros2_control` interface, driven by torque commands derived from inverse-dynamics/PD trajectory tracking
- **Disturbance and stability testing** — foot-strike and payload-loading scenarios used to evaluate EKF angle-lock robustness under sudden mechanical shock

---

## System Architecture

```
┌─────────────────────────────┐         ROS 2 (Float64MultiArray)        ┌──────────────────────────────┐
│   Ubuntu Laptop              │ ────────────────────────────────────▶   │   MATLAB/Simulink Laptop      │
│   ROS 2 Jazzy + Gazebo       │        /leg/torque_cmd (τ_ref)          │   ROS 2 Toolbox                │
│   Harmonic                   │ ◀────────────────────────────────────   │                                │
│                               │       /leg/motor_feedback               │  ┌──────────────────────────┐  │
│  ┌─────────────────────────┐ │       (θ, ω, α, θ̂ₑ)                     │  │  EKF (Back-EMF observer)  │  │
│  │ 2-DOF Leg (URDF)         │ │                                          │  │  θ̂ₑ, ω̂ₑ estimation        │  │
│  │  hip_joint, knee_joint   │ │                                          │  └──────────────┬───────────┘  │
│  └─────────────────────────┘ │                                          │                 ▼               │
│  ┌─────────────────────────┐ │                                          │  ┌──────────────────────────┐  │
│  │ torque_bridge_node.py    │ │                                          │  │ FOC Cascade                │  │
│  │  - Gait trajectory gen.  │ │                                          │  │  Clarke/Park, Id/Iq PI,   │  │
│  │  - PD torque estimate    │ │                                          │  │  SVPWM                     │  │
│  │  - τ_ref / τ_load        │ │                                          │  └──────────────┬───────────┘  │
│  └─────────────────────────┘ │                                          │                 ▼               │
│                               │                                          │  ┌──────────────────────────┐  │
│                               │                                          │  │ Simscape 3-Phase Inverter  │  │
│                               │                                          │  │  + PMSM Power Stage        │  │
│                               │                                          │  └──────────────────────────┘  │
└─────────────────────────────┘                                          └──────────────────────────────┘
```

**Signal roles:**
- **τ_ref** — commanded torque (from gait trajectory + inverse dynamics), converted to `Iq_ref` and driving the motor's current loop.
- **τ_load** — external mechanical disturbance (gravity, payload, ground reaction), injected onto the Simscape rotor via an Ideal Torque Source, independent of τ_ref.
- **θ̂ₑ, ω̂ₑ** — EKF-estimated electrical angle/speed used internally by the FOC cascade (never fed back to Gazebo as ground truth).

---

## Repository Structure

```
.
├── Complete_Sensorless_FOC_EKF_simulink_model.slx   # Full FOC + EKF + Simscape power stage model
├── complete_recovery.slx                             # Model checkpoint / recovery version
├── motor_params.m                                    # PMSM electrical & mechanical parameters
├── robotic_ROS2/                                     # ROS 2 workspace: leg URDF, controllers, bridge nodes
└── README.md
```

---

## Motor & Control Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Stator resistance | Rₛ | 2.875 Ω |
| Flux linkage | λ_m | 0.175 Wb |
| d-axis inductance | L_d | 6.0 mH |
| q-axis inductance | L_q | 10.5 mH |
| DC bus voltage | V_dc | 48 V |
| PWM switching frequency | f_sw | 20 kHz |
| Current loop bandwidth | — | ≈ 1 kHz |
| Iq current loop gains | Kp, Ki | 65.97, 18063 |
| Id current loop gains | Kp, Ki | 37.7, 18063 |
| EKF process noise covariance | Q | diag(1e-6, 1e-3) |
| EKF measurement noise covariance | R | diag(1e-1, 1e-1) |
| EKF integration step | dt | 1e-5 s |

Full parameter definitions are in [`motor_params.m`](./motor_params.m).

---

## Getting Started

### Prerequisites

| Component | Requirement |
|---|---|
| MATLAB | R2023b or later, with **Simulink**, **Simscape Electrical**, and **ROS Toolbox** |
| ROS 2 | Jazzy Jalisco |
| Simulation | Gazebo Harmonic |
| OS | Ubuntu 24.04 (ROS 2/Gazebo side); Windows or Ubuntu (MATLAB side) |

Both machines must be reachable over the same network and share a common `ROS_DOMAIN_ID`.

### 1. Clone the repository

```bash
git clone https://github.com/ronakyadav723/Sensorless-FOC-driven-Dynamic-Robotic-Leg-using-Extended-Kalman-Filter.git
cd Sensorless-FOC-driven-Dynamic-Robotic-Leg-using-Extended-Kalman-Filter
```

### 2. Build the ROS 2 workspace (Ubuntu / Gazebo side)

```bash
cd robotic_ROS2/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Set a shared ROS 2 domain on both machines

```bash
export ROS_DOMAIN_ID=42        # must match on both the Ubuntu and MATLAB machines
```

### 4. Launch the Gazebo leg simulation

```bash
ros2 launch leg_gazebo leg_full_v3.launch.py
```

### 5. Open and run the Simulink model

In MATLAB, on the second machine:

```matlab
setenv("ROS_DOMAIN_ID","42")
open("Complete_Sensorless_FOC_EKF_simulink_model.slx")
```

Run the model. The Simulink ROS 2 Subscribe/Publish blocks connect automatically to `/leg/torque_cmd` and `/leg/motor_feedback`.

### 6. Verify the live bridge

```bash
ros2 topic echo /leg/torque_cmd
ros2 topic echo /leg/motor_feedback
```

---

## Evaluation Focus

This project is evaluated (per the original problem statement) against:

- **Estimation Accuracy** — EKF angle estimate (θ̂ₑ) tracking fidelity against the true simulated angle under dynamic loads
- **System Robustness** — FOC loop survival (no stall, no desync) under sudden mechanical disturbance
- **Mathematical Rigor** — correctness of the state-space model, EKF Jacobians, and covariance tuning justification
- **Integration Quality** — latency, synchronization, and stability of the electrical–mechanical co-simulation bridge

---

## Roadmap / Bonus Objectives

- [ ] High-Frequency Injection (HFI) for zero-speed rotor angle observability
- [ ] Replace PI current loops with Model Predictive Control (MPC) for improved transient torque response during mechanical shocks

---

## License

This project is released under the [MIT License](LICENSE).

---

## Acknowledgements

Developed as part of **IITISoC 2026** — Science & Technology Council, IIT Indore.

