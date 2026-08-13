# System Design — ACO-EKF Battery SOC Estimation

---

## 1. High-Level System Architecture

```mermaid
graph TB
    subgraph SENSORS["🔌 Sensor Layer"]
        CS[Current Sensor]
        VS[Voltage Sensor]
        TS[Temperature Sensor]
    end

    subgraph DATA["📦 Data Layer"]
        DG[data_generator.py\nSynthetic Profiles Generator]
        DS1[Profile 1: CC Discharge]
        DS2[Profile 2: Pulse Discharge]
        DS3[Profile 3: Dynamic Drive Cycle]
        DS4[Profile 4: CCCV Charge]
        DG --> DS1 & DS2 & DS3 & DS4
    end

    subgraph MODEL["🔋 Battery Model Layer"]
        BM[battery_model.py\nFirst-Order RC Thevenin Model]
        OCV[OCV-SOC Polynomial]
        ST[State Transition f·x·u]
        MH[Measurement Function h·x·u]
        FJ[State Jacobian F]
        HJ[Measurement Jacobian H]
        BM --> OCV & ST & MH & FJ & HJ
    end

    subgraph OPT["🐜 Optimization Layer"]
        ACO[aco_optimizer.py\nACOR Algorithm]
        ARC[Solution Archive\n20 elite candidates]
        PHE[Pheromone Weights\nRank-based]
        GKN[Gaussian Kernels\nSampling mechanism]
        OBJ[Objective Function\nMSE of SOC error]
        ACO --> ARC --> PHE --> GKN --> OBJ
    end

    subgraph EKF_LAYER["📡 Estimation Layer"]
        EKF[ekf.py\nExtended Kalman Filter]
        PRD[Predict Step\nx and P]
        COR[Correct Step\nK and x]
        KAL[Kalman Gain\nK computation]
        EKF --> PRD & COR & KAL
    end

    subgraph EVAL["📊 Evaluation Layer"]
        MAIN[main.py\nOrchestration Pipeline]
        MET[Metrics Computation\nMSE MAE MAPE R2]
        COMP[ACO-EKF vs Conv-EKF\nComparison]
        MAIN --> MET & COMP
    end

    subgraph VIZ["📈 Visualization Layer"]
        VIS[visualization.py]
        F1[01 Current Voltage]
        F2[02 SOC Comparison]
        F3[03 SOC Error]
        F4[04 ACO Convergence]
        F5[05 Metrics Table]
        VIS --> F1 & F2 & F3 & F4 & F5
    end

    SENSORS -->|Real hardware| EKF_LAYER
    DATA -->|Synthetic data| OPT
    DATA -->|Test data| EVAL
    MODEL -->|f h F H| EKF_LAYER
    MODEL -->|Simulate profiles| DATA
    OPT -->|Q* and R*| EKF_LAYER
    EKF_LAYER -->|SOC estimates| EVAL
    EVAL -->|Results dict| VIZ
```

---

## 2. Component Architecture

```mermaid
classDiagram

    class BatteryModel {
        +float R0
        +float R1
        +float C1
        +float Qn
        +float eta
        +float dt
        +float _a
        +float _b
        +ocv_from_soc(soc) float
        +docv_dsoc(soc) float
        +state_transition(x, I) ndarray
        +terminal_voltage(x, I) float
        +state_jacobian() ndarray
        +measurement_jacobian(soc) ndarray
        +simulate(current_profile, soc0, vp0) dict
    }

    class ExtendedKalmanFilter {
        +BatteryModel model
        +ndarray Q
        +float R
        +ndarray P0
        -ndarray _x_hat
        -ndarray _P
        +reset(soc0, vp0)
        +step(I, Vt_measured) float
        +run(currents, voltages, soc0, vp0) ndarray
        +state() ndarray
        +covariance() ndarray
    }

    class ACOR {
        +callable objective
        +list bounds
        +int k_archive
        +int n_ants
        +int max_iter
        +float q_acor
        +float xi
        +ndarray best_solution
        +float best_objective
        +list convergence
        +run() dict
        -_weights() ndarray
        -_sample_gaussian(archive, sigma, weights) ndarray
        -_compute_sigma(archive) ndarray
        -_evaluate(params) float
    }

    class BatteryDataset {
        +str name
        +ndarray time
        +ndarray current_true
        +ndarray current_meas
        +ndarray voltage_true
        +ndarray voltage_meas
        +ndarray soc_ref
        +float soc0
    }

    class Pipeline {
        +dict CONFIG
        +main()
        -_make_Q(q_soc, q_vp) ndarray
        -_make_P0(cfg) ndarray
        -_run_ekf_on_dataset(ds, model, Q, R, P0, soc0) ndarray
        -_objective_factory(model, datasets, P0, cfg) callable
    }

    BatteryModel <|-- ExtendedKalmanFilter : uses
    BatteryModel <|-- BatteryDataset : simulates into
    ExtendedKalmanFilter <|-- ACOR : evaluates via objective
    ExtendedKalmanFilter <|-- Pipeline : runs
    ACOR <|-- Pipeline : runs
    BatteryDataset <|-- Pipeline : loads
```

---

## 3. Data Flow Diagram

```mermaid
flowchart LR
    subgraph INPUT
        I_raw[Raw Current I_k]
        V_raw[Raw Voltage Vt_k]
    end

    subgraph NOISE
        I_n[+ Gaussian Noise\nstd = 0.005 × I_C]
        V_n[+ Gaussian Noise\nstd = 5 mV]
    end

    subgraph BATTERY_EQ
        SOC_eq["SOC(k+1) = SOC(k) − η·I·Δt / Qn"]
        VP_eq["Vp(k+1)  = a·Vp(k) + b·I"]
        VT_eq["Vt(k) = OCV(SOC) − Vp − R₀·I"]
    end

    subgraph EKF_STEPS
        direction TB
        E1["① Predict state\n x̂ = f(x̂_prev, I)"]
        E2["② Predict covariance\n P = F·P·Fᵀ + Q"]
        E3["③ Predict voltage\n Vt_pred = h(x̂, I)"]
        E4["④ Innovation\n r = Vt_meas − Vt_pred"]
        E5["⑤ Kalman gain\n K = P·Hᵀ·(H·P·Hᵀ+R)⁻¹"]
        E6["⑥ Correct state\n x̂ = x̂ + K·r"]
        E7["⑦ Update covariance\n P = (I−KH)P(I−KH)ᵀ+KRKᵀ"]
        E1-->E2-->E3-->E4-->E5-->E6-->E7
    end

    subgraph ACO_LOOP
        direction TB
        A1[Init 20 random Q/R]
        A2[Rank by SOC error]
        A3[Sample 8 new candidates\nnear best solutions]
        A4[Evaluate each on training data]
        A5[Keep top 20]
        A6{Converged?}
        A7[Output Q* and R*]
        A1-->A2-->A3-->A4-->A5-->A6
        A6-->|No| A3
        A6-->|Yes| A7
    end

    subgraph OUTPUT
        SOC_est[SOC Estimate %]
        FIGS[5 Comparison Figures]
        METRICS[MSE / MAE / MAPE / R²]
    end

    I_raw --> I_n --> BATTERY_EQ
    V_raw --> V_n --> BATTERY_EQ
    BATTERY_EQ --> EKF_STEPS
    ACO_LOOP -->|Q*, R*| EKF_STEPS
    EKF_STEPS --> SOC_est --> METRICS --> FIGS
```

---

## 4. EKF Internal State Machine

```mermaid
stateDiagram-v2
    [*] --> Uninitialized

    Uninitialized --> Ready : reset(soc0, vp0)\nsets x̂ = [soc0, 0]\nsets P = P₀

    Ready --> Predicting : step() called\nwith current I

    Predicting --> Correcting : State predicted\nx̂_pred = f(x̂, I)\nP_pred = FPFᵀ + Q

    Correcting --> Ready : Correction done\nr = Vt_meas − h(x̂_pred)\nK = P·Hᵀ·S⁻¹\nx̂ = x̂_pred + K·r\nP = Joseph form update\nReturn SOC estimate

    Ready --> Done : All samples\nprocessed

    Done --> [*]
```

---

## 5. ACO Optimization State Machine

```mermaid
stateDiagram-v2
    [*] --> Init

    Init --> ArchiveEval : Generate 20 random\n[log q_SOC, log q_Vp, log r_V]\nin search bounds

    ArchiveEval --> Sorted : Evaluate all 20 candidates\nRun EKF on training data\nCompute MSE for each

    Sorted --> AntSampling : Sort archive by MSE\nBest solution recorded

    AntSampling --> CandidateEval : Compute rank weights\nCompute Gaussian sigmas\nSample 8 new candidates\nnear archive solutions

    CandidateEval --> Merge : Evaluate each candidate\nRun EKF + compute MSE

    Merge --> CheckStop : Merge archive + new candidates\nKeep top 20 by MSE

    CheckStop --> AntSampling : iteration < max_iter

    CheckStop --> Done : iteration == max_iter

    Done --> [*] : Return Q*, R*,\nbest_MSE,\nconvergence list
```

---

## 6. Module Interface Contracts

### `battery_model.py`

| Method | Input | Output | Purpose |
|--------|-------|--------|---------|
| `ocv_from_soc(soc)` | `float [0,1]` | `float [V]` | OCV polynomial lookup |
| `docv_dsoc(soc)` | `float [0,1]` | `float [V]` | Derivative for Jacobian H |
| `state_transition(x, I)` | `x: (2,), I: float` | `(2,)` | Compute x(k+1) |
| `terminal_voltage(x, I)` | `x: (2,), I: float` | `float [V]` | Compute Vt |
| `state_jacobian()` | — | `(2,2)` | Constant F matrix |
| `measurement_jacobian(soc)` | `float` | `(1,2)` | H = [dOCV/dSOC, -1] |
| `simulate(I_profile, soc0)` | `(N,), float` | `dict` | Batch simulation |

### `ekf.py`

| Method | Input | Output | Purpose |
|--------|-------|--------|---------|
| `reset(soc0, vp0)` | `float, float` | — | Initialize filter state |
| `step(I, Vt_meas)` | `float, float` | `float [SOC]` | One EKF recursion |
| `run(currents, voltages, soc0)` | `(N,), (N,), float` | `(N,) [SOC]` | Batch processing |

### `aco_optimizer.py`

| Method | Input | Output | Purpose |
|--------|-------|--------|---------|
| `run()` | — | `dict` | Full ACOR optimization |
| `_weights()` | — | `(k,)` | Rank-based pheromone weights |
| `_compute_sigma(archive)` | `(k,3)` | `(k,3)` | Gaussian kernel widths |
| `_sample_gaussian(archive, sigma, w)` | `(k,3),(k,3),(k,)` | `(3,)` | Draw one candidate |
| `_evaluate(params)` | `[q_SOC, q_Vp, r_V]` | `float` | Run objective function |

---

## 7. Key Design Decisions

```mermaid
mindmap
  root((ACO-EKF\nSystem Design))
    Battery Model
      First-Order RC
        Simple enough for microcontroller
        Captures polarization dynamics
      8th-Order OCV Polynomial
        Nonlinear SOC-voltage mapping
        Enables EKF correction
      Jacobians analytical
        No numerical differentiation needed
        Faster and more stable

    EKF Design
      Joseph Form Covariance Update
        Numerically stable
        Preserves positive-definite P
      State Clipping
        SOC clamped to 0-1
        Prevents divergence
      Log-scale Q and R search
        Covers 5 decades of values
        Unbiased search space

    ACO Design
      ACOR variant
        Continuous domain search
        No discretization needed
      Log-space search
        1e-7 to 1e-1 range efficiently
        Avoids large-value bias
      Archive size 20
        Diversity maintained
        Not too slow to evaluate
      Objective is MSE
        Simple and differentiable concept
        Directly measures SOC error

    Data Design
      Train-Test Split
        ACO tunes on profiles 1 and 2
        Tests on profiles 3 and 4
        Prevents overfitting
      4 Operating Profiles
        Different dynamics
        Different SOC ranges
        Different current patterns
      Sensor Noise Added
        Realistic simulation
        Tests filter robustness
```

---

## 8. Deployment View

```mermaid
graph TB
    subgraph OFFLINE["🖥️ Offline Phase — PC / Workstation"]
        direction TB
        A[Collect training data\nfrom real battery or simulation]
        B[Run ACO optimizer\n30–60 iterations × 8–10 ants]
        C[Find optimal Q* and R*]
        D[Validate on test profiles]
        A --> B --> C --> D
    end

    subgraph ONLINE["⚙️ Online Phase — Microcontroller / BMS"]
        direction TB
        E[Read current sensor\nevery Δt = 1 second]
        F[Read voltage sensor]
        G[Run EKF step with Q* and R*]
        H[Update SOC estimate]
        I[Display or transmit SOC]
        E & F --> G --> H --> I --> E
    end

    C -->|Q*, R* flashed\nto firmware| ONLINE
```

### Resource Requirements

| Phase | Platform | Memory | Computation |
|-------|----------|--------|-------------|
| ACO Tuning | PC / workstation | ~50 MB RAM | 30–600 sec (depends on iterations) |
| EKF Real-Time | Microcontroller | < 2 KB RAM | < 1 ms per sample at 1 Hz |

---

## 9. File Dependency Map

```mermaid
graph LR
    BM[battery_model.py]
    EKF[ekf.py]
    ACO[aco_optimizer.py]
    DG[data_generator.py]
    VIS[visualization.py]
    MAIN[main.py]

    BM -->|BatteryModel class| EKF
    BM -->|simulate method| DG
    EKF -->|ExtendedKalmanFilter| ACO
    EKF -->|ExtendedKalmanFilter| MAIN
    ACO -->|ACOR class| MAIN
    DG -->|BatteryDataset list| MAIN
    VIS -->|plot functions| MAIN

    style BM fill:#1565C0,color:#fff
    style EKF fill:#6A1B9A,color:#fff
    style ACO fill:#2E7D32,color:#fff
    style DG fill:#E65100,color:#fff
    style VIS fill:#AD1457,color:#fff
    style MAIN fill:#37474F,color:#fff
```

---

## 10. Error & Noise Model

```mermaid
graph LR
    subgraph TRUE["True System"]
        TI[True Current]
        TV[True Voltage]
        TS[True SOC]
    end

    subgraph NOISE_MODEL["Noise Model"]
        CN["Current noise\nw_I ~ N(0, 0.005·I_C)"]
        VN["Voltage noise\nv ~ N(0, 5 mV)"]
        MN["Model noise\nw_x ~ N(0, Q)"]
    end

    subgraph MEASURED["Measured / Estimated"]
        MI[Measured Current]
        MV[Measured Voltage]
        ES[Estimated SOC]
    end

    TI -->|+ noise| CN --> MI
    TV -->|+ noise| VN --> MV
    TS -->|model mismatch| MN

    MI -->|drives coulomb counting| ES
    MV -->|correction term| ES
    MN -->|Q matrix represents| ES
```

---

## Summary Table

| Layer | File | Technology | Key Algorithm |
|-------|------|-----------|---------------|
| **Battery Model** | `battery_model.py` | NumPy | First-order RC, OCV polynomial, Jacobians |
| **Data** | `data_generator.py` | NumPy | Profile simulation + Gaussian noise |
| **Estimation** | `ekf.py` | NumPy | EKF with Joseph-form P update |
| **Optimization** | `aco_optimizer.py` | NumPy | ACOR: rank-weighted Gaussian sampling |
| **Pipeline** | `main.py` | Python | Train/test split, CONFIG dict |
| **Visualization** | `visualization.py` | Matplotlib | 5 publication figures |
| **Docs** | `README.md` | Markdown | Usage guide |
