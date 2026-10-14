# 🏛️ BMS Architecture Overview

The **Thermal-Adaptive 2-RC MO-ACO-EKF System** is architected for model-based design, automotive-style workflows, providing mathematical equivalence across Python simulation, Simulink SIL simulation, and ANSI C microcontroller firmware.

---

## 🧩 High-Level System Architecture


```mermaid
graph TD
    subgraph Inputs ["Input Drive Profiles"]
        DP["Drive Cycle Current I(t)<br/>(UDDS, Pulse, CC, Fast-Charge)"]
        TR["Ambient Temperature T_amb"]
    end

    subgraph Plant ["Battery Physics Plant (Subsystem 1)"]
        BPP["2-RC Electrochemical Core<br/>R0(T), R1(T), C1, R2(T), C2<br/>Arrhenius Parameter Scaling"]
        OCV["8th-Order OCV(SOC) Surface"]
        BPP -->|True State| PT["BMS_PlantTruth Bus<br/>[True SOC, True V, True T]"]
    end

    subgraph Sensors ["Sensor Measurement & Faults (Subsystem 2)"]
        SM["Sensor Noise Injection<br/>Voltage: σ_V = 49.5 mV<br/>Current: σ_I = 50 mA"]
        FI["Fault Injection Engine<br/>1: Current Offset (+0.25A)<br/>2: Voltage Glitches (±0.5V)<br/>3: Thermistor Disconnect"]
        SM & FI -->|Noisy Data| MB["BMS_Measurement Bus<br/>[Measured V, Measured I, Measured T]"]
    end

    subgraph Estimator ["ACO-Tuned Controller (Subsystem 3)"]
        ACOR["Offline MO-ACOR Tuning<br/>Q* = diag(1e-7, 1e-7, 1e-7)<br/>R* = 2.455e-3 V²"]
        EKF["4-State Joseph-Form EKF<br/>x = [SOC, V1, V2, Cap_Ratio]^T<br/>Sage-Husa Dynamic R / RLS R0"]
        ACOR -.->|Covariances| EKF
        MB --> EKF
        EKF --> EB["BMS_Estimate Bus<br/>[SOC_pct, V_pred, NIS, PTrace, FaultCode]"]
    end

    subgraph Scopes ["Monitoring & Telemetry"]
        EB & PT --> COMP["SOC_Comparison_Scope<br/>(True vs EKF vs Coulomb)"]
        EB --> DIAG["Diagnostic_Fault_Code_Scope<br/>(Safety Bitmask)"]
        EB --> NIS["Innovation_NIS_Scope<br/>(Filter Whiteness ~1.0)"]
    end

    DP --> BPP
    TR --> BPP
    DP --> SM
```

---

## 📐 Signal Bus Definitions (`bms_data.sldd`)

All inter-subsystem communications flow over strongly-typed Simulink Bus Objects:

| Bus Name | Elements | Physical Units | Description |
| :--- | :--- | :---: | :--- |
| **`BMS_PlantTruth`** | `SOC_pct`, `Voltage_V`, `Temperature_K` | `[%]`, `[V]`, `[K]` | Ground-truth physical cell internal states |
| **`BMS_Measurement`** | `Voltage_V`, `Current_A`, `Temperature_K` | `[V]`, `[A]`, `[K]` | Hardware sensor readings corrupted by noise & faults |
| **`BMS_Estimate`** | `SOC_pct`, `Voltage_V`, `Innovation_V`, `PTrace`, `NIS`, `FaultCode`, `R0_Ohm`, `SOC_Coulomb_pct`, `R_cov_V2`, `CapacityRatio` | Various | Complete telemetry and diagnostic output from estimator |
| **`BMS_Reference`** | `SOC_pct`, `Source_Tag` | `[%]`, `[-]` | Reference cell SOC and source discriminator ($0.0 = \text{SIM}, 1.0 = \text{REPLAY}$) |

---

## 📦 Referenced Estimator Architecture (`bms_estimator_ref.slx`)

To eliminate code duplication, enforce modularity, and support multi-instance execution, the core estimation logic is encapsulated within a single reusable referenced model: [`bms_estimator_ref.slx`](../simulink_matlab/bms_estimator_ref.slx).

- **Execution Characteristics:** Discrete fixed-step ($T_s = 1.0\text{ s}$), multi-instance enabled (`TotalNumberRefModelInstancesAllowed = 'Multi'`).
- **Interfaces:** Strictly bus-typed. Accepts [`BMS_Measurement`](#bms_measurement) as input and emits [`BMS_Estimate`](#bms_estimate) as output.
- **Coder-Constant Parameterization:** The model block accepts an operating mode parameter (`bms_est_mode`), configuring the internal estimation algorithm without code branching or separate model files:
  - `Mode = 0`: Baseline Joseph-form Extended Kalman Filter with fixed tuned covariances ($Q^*, R^*$).
  - `Mode = 2`: SOTA Adaptive EKF with Sage-Husa dynamic measurement noise covariance adaptation and recursive least squares ($R_0$) estimation.
  - `Mode = 10`: SOH Capacity EKF with dual-state estimation tracking fading cell capacity over degradation cycles.
- **Model Sharing:** Reused across all top-level models—[`advanced_bms_aco_ekf.slx`](../simulink_matlab/advanced_bms_aco_ekf.slx) (1 instance), [`sota_bms_aco_ekf.slx`](../simulink_matlab/sota_bms_aco_ekf.slx) (3 parallel instances), and [`bms_unified.slx`](../simulink_matlab/bms_unified.slx) (3 parallel instances).

---

## 🔄 Unified System Model (`bms_unified.slx`)

[`bms_unified.slx`](../simulink_matlab/bms_unified.slx) replaces separate synthetic and replay models by integrating synthetic simulation and real measured NASA data replay into a single, cohesive Simulink architecture.

```mermaid
graph TD
    subgraph Inputs ["Simulation Scenarios & Data Sources"]
        DP["Drive Cycle Profile<br/>(Synthetic I, V, T)"]
        NASA["NASA Battery Dataset<br/>(Measured I, V, T)"]
        FC["Fault Injection Engine<br/>(Current offset, V glitch, Open thermistor)"]
    end

    subgraph Variant ["Variant Subsystem: Input_Source_Variant (BMS_Source)"]
        direction TB
        CS["Choice_SIM (BMS_Source == 1)<br/>2-RC Plant Truth + Sensor Noise + Faults"]
        CR["Choice_REPLAY (BMS_Source == 2)<br/>NASA Replay Data + Conditional Faults"]
    end

    subgraph Estimators ["Estimator Bank (3x bms_estimator_ref)"]
        direction TB
        E1["Baseline EKF (Mode 0)"]
        E2["Adaptive SOTA EKF (Mode 2)"]
        E3["Capacity SOH EKF (Mode 10)"]
    end

    subgraph Telemetry ["Unified Logging & Output"]
        LOG["bms_log<br/>(14 Signal Channels)"]
        CSV["scenario_<name>.csv<br/>(15 Columns: Time_s + 14 signals)"]
    end

    DP --> CS
    FC --> CS
    NASA --> CR
    FC --> CR

    CS -->|BMS_Measurement| E1 & E2 & E3
    CR -->|BMS_Measurement| E1 & E2 & E3
    CS -->|BMS_Reference: Tag=0| LOG
    CR -->|BMS_Reference: Tag=1| LOG

    E1 -->|BMS_Estimate| LOG
    E2 -->|BMS_Estimate| LOG
    E3 -->|BMS_Estimate| LOG
    LOG --> CSV
```

### 1. Variant Source Selection (`BMS_Source`)
The model uses a Simulink Variant Subsystem (`Input_Source_Variant`) configured with parameter-based variant control:
- **`BMS_Source == 1` (`Choice_SIM`)**: Executes the 2-RC equivalent circuit physical plant (`bms_plant_step`) corrupted by Gaussian sensor noise and fault injection. Emits `BMS_PlantTruth` and `BMS_Measurement`.
- **`BMS_Source == 2` (`Choice_REPLAY`)**: Feeds measured voltage, current, and temperature directly from NASA lithium-ion experimental datasets (cells B0005, B0006, B0007, B0018). Emits `BMS_Measurement` and optionally applies sensor faults if `BMS_InjectFaultsOnReplay == 1`.

### 2. Reference & Source Discrimination (`BMS_Reference`)
To enable automated regression testing and disambiguation, the source subsystem outputs a `BMS_Reference` bus containing:
- `SOC_pct`: Reference State of Charge ($0 \dots 100\,\%$). In SIM mode, this is true electrochemical plant SOC; in REPLAY mode, this is the coulomb-counter reference derived from NASA experimental discharge data.
- `Source_Tag`: Numeric tag ($0.0$ for SIM, $1.0$ for REPLAY). Regression Suite 8 tests verify that this tag is strictly non-vacuous.

### 3. Unified Telemetry Logging (`bms_log`)
All estimation and reference signals are bundled into a 14-signal `bms_log` bus:
1. `Reference SOC (%)` (`BS_Reference/SOC_pct`)
2. `EKF Baseline SOC (%)` (`BS_Baseline/SOC_pct`)
3. `Adaptive EKF SOC (%)` (`BS_Adaptive/SOC_pct`)
4. `Coulomb Counting SOC (%)` (`BS_Baseline/SOC_Coulomb_pct`)
5. `Measured Voltage (V)` (`BS_Measured/Voltage_V`)
6. `Adaptive Predicted Voltage (V)` (`BS_Adaptive/Voltage_V`)
7. `Innovation (V)` (`BS_Adaptive/Innovation_V`)
8. `NIS (Normalized Innovation Squared)` (`BS_Adaptive/NIS`)
9. `Diagnostic Fault Code Bitmask` (`BS_Adaptive/FaultCode`)
10. `Identified Resistance R0 (Ohm)` (`BS_Adaptive/R0_Ohm`)
11. `Adapted Measurement Variance R (V^2)` (`BS_Adaptive/R_cov_V2`)
12. `Capacity EKF SOC (%)` (`BS_Capacity/SOC_pct`)
13. `Estimated Capacity Ratio (SOH)` (`BS_Capacity/CapacityRatio`)
14. `Source Tag` (`BS_Reference/Source_Tag`)

When processed and exported by [`run_all.m`](../simulink_matlab/run_all.m), `Time_s` is prepended as column 1 to create standard 15-column scenario CSV archives in `results/scenario_<name>.csv`.

---

## 🔒 3-Subsystem Layout Rationale

1. **`Battery_Physics_Plant`**: Isolates physical chemical and thermal equations from control logic.
2. **`Sensor_Measurement`**: Emulates automotive wiring harnesses, ADC quantization, and failure modes without modifying plant physics.
3. **`ACO_EKF_Controller`**: Blind to internal ground-truth; receives only sensor measurements, ensuring strict Software-in-the-Loop (SIL) fidelity.
