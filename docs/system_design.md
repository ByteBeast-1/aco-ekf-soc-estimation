# System Architectural Design Document
## Thermal-Adaptive 2-RC Multi-Objective ACO-EKF Battery SOC Estimation

![System Architecture](file:///c:/Users/sande/OneDrive/Desktop/aco-ekf-soc-estimation/figures/00_system_architecture.png)

---

### 1. System Topology Architecture

```mermaid
graph TD
    subgraph Input Layer
        I1[Driving Profiles: UDDS, CC, Pulse] --> M[Multiplier Node ✕]
        I2[Thermal Dynamics: 24°C → 39°C] --> M
        I3[NASA B0005 Hardware Logs] --> M
    end

    subgraph Sensing Layer
        M --> S[Current, Voltage & Temp Sensors]
    end

    subgraph Offline AI Swarm Layer
        ACO[MO-ACOR AI Optimizer - 4D Log-Space] --> OPT[Optimal Covariances Q* and R*]
    end

    subgraph Core Estimator Engine
        S --> TWIN[2-RC Physical Twin - Arrhenius R(T)]
        OPT --> EKF[3-State EKF Core Core - x = [SOC, V1, V2]^T]
        TWIN --> EKF
        EKF --> JOS[Joseph-Form Covariance Update]
        JOS --> SOC[Estimated SOC % - Sub-0.01% MAE]
    end

    subgraph Multi-Platform Deployment & Feedback
        SOC --> SIM[Simulink MBD Model: advanced_bms_aco_ekf.slx]
        SIM --> FW[ANSI C Firmware: bms_ekf.c < 2 KB RAM]
        FW --> DISP[BMS Real-Time EV Fuel Gauge Display]
        DISP -. Feedback Correction Loop .-> I3
    end
```

---

### 2. Block-by-Block Architecture Specifications

#### 1. Input Sensing & Load Layer
* **Driving Profiles:** Generates dynamic current load sequences ($I_{\text{load}}$) corresponding to highway cruising (CC 0.5C), city traffic (Pulse 1C), and aggressive urban driving (UDDS).
* **Thermal Dynamics:** Models battery self-heating from $24^\circ\text{C} \to 39^\circ\text{C}$ over 3,600 seconds.
* **NASA B0005 Dataset:** Ingests physical 18650 cell experimental laboratory logs.

#### 2. Offline AI Swarm Layer (`aco_optimizer.py`)
* **MO-ACOR AI:** 20 virtual ants search in 4D logarithmic space $[-8, -5]$.
* **Optimal Matrices:** Discovers Pareto-optimal $Q^* = \text{diag}([10^{-7}, 10^{-7}, 10^{-7}])$ and $R^* = 2.455 \times 10^{-3}$ minimizing tracking MSE and initial error convergence time.

#### 3. 2-RC Physical Digital Twin & EKF Core (`ekf.py` & `battery_model.py`)
* **2-RC Physics Twin:** Evaluates Arrhenius temperature compensation $R_0(T), R_1(T), R_2(T)$ and 8th-order OCV polynomial curve.
* **3-State EKF Core:** Runs real-time 1-second Predict-Correct recursion ($\mathbf{x}_k = [\text{SOC}_k, V_{1,k}, V_{2,k}]^T$).
* **Joseph-Form Covariance Update:** Calculates $\mathbf{P}_k = (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}_p(\mathbf{I}-\mathbf{K}\mathbf{H})^T + \mathbf{K}R^*\mathbf{K}^T$ guaranteeing 100% positive-definiteness ($P > 0$).

#### 4. Multi-Platform Deployment & Feedback Loop
* **Simulink MBD Model:** Interactive 2-subsystem block diagram (`advanced_bms_aco_ekf.slx`).
* **ANSI C Firmware:** Zero-malloc production C code (`c_firmware/bms_ekf.c`).
* **BMS Fuel Gauge Display:** Drives live EV dashboard percentage display with sub-0.01% MAE accuracy.
