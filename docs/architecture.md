# 🏛️ BMS Architecture Overview

The **Thermal-Adaptive 2-RC MO-ACO-EKF System** is architected for automotive Model-Based Design (MBD), providing mathematical equivalence across Python simulation, Simulink SIL simulation, and ANSI C microcontroller firmware.

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

---

## 🔒 3-Subsystem Layout Rationale

1. **`Battery_Physics_Plant`**: Isolates physical chemical and thermal equations from control logic.
2. **`Sensor_Measurement`**: Emulates automotive wiring harnesses, ADC quantization, and failure modes without modifying plant physics.
3. **`ACO_EKF_Controller`**: Blind to internal ground-truth; receives only sensor measurements, ensuring strict Software-in-the-Loop (SIL) fidelity.
