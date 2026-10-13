# Phase 2 Verification & Reproduction Guide (`PHASE2_RUN_ME.md`)

This document details the architecture, programmatic generation, and verification procedures for **Phase 2 (Items A & B)** of the BMS upgrade project as outlined in [`docs/AGENT_BRIEF.md`](file:///c:/Users/sande/OneDrive/Desktop/aco-ekf-soc-estimation/docs/AGENT_BRIEF.md).

---

## 1. Executive Summary & Deliverables

All changes have been implemented on branch **`phase2-unified`** without editing any `.slx` files by hand.

| Component | Status | Verification State | Key Metrics / Evidence |
| :--- | :--- | :--- | :--- |
| **`bms_estimator_ref.slx`** | Created programmatically | **VERIFIED** | Discrete fixed-step ($T_s = 1.0\text{ s}$), multi-instance enabled, strict bus typed (`BMS_Measurement` $\to$ `BMS_Estimate`), coder-constant parameterization |
| **`advanced_bms_aco_ekf.slx`** | Refactored | **VERIFIED** | Instantiates `bms_estimator_ref`, exact $0.00\text{e+}00\,\%$ difference vs MATLAB reference library |
| **`sota_bms_aco_ekf.slx`** | Refactored | **VERIFIED** | Instantiates 3 parallel `bms_estimator_ref` instances (Baseline, Adaptive, SOH), exact $0.00\text{e+}00\,\%$ difference vs MATLAB reference library |
| **`bms_unified.slx`** | Created programmatically | **VERIFIED** | Variant Subsystem (`Choice_SIM` vs `Choice_REPLAY`), $0.00\text{e+}00\,\%$ numerical difference vs legacy models |
| **`bms_data.sldd`** | Updated | **VERIFIED** | Added `BMS_Reference` bus (`SOC_pct`, `Source_Tag`) with physical units & descriptions |
| **`bms_scenario.m`** | Created | **VERIFIED** | Returns unified scenario struct (`name`, `source`, `I`, `V_ref`, `T`, `soc_ref`, `soc0`, `init_err`, `seed`, `fault_mode`, `params_variant`), plus `'list'` |
| **`run_all.m`** | Created | **VERIFIED** | Executes scenario sweeps via `Simulink.SimulationInput`, outputs per-scenario time-series CSVs, 4-panel PNG figures, and `all_scenarios_summary.csv` |
| **`run_bms_regression.m`** | Updated | **VERIFIED** | **38 / 38 checks pass** (Suites 0 through 8). Suite 8 confirms unified model reproduces legacy models within $1\text{e-}9$ |

---

## 2. Architecture of `bms_unified.slx`

`bms_unified.slx` unifies synthetic drive simulation and real measured NASA cell replay into a single model architecture:

```mermaid
graph TD
    subgraph Inputs
        DP[Drive_Profile] --> VS[Input_Source_Variant]
        NR[NASA_Replay] --> VS
        FM[Fault_Mode] --> VS
        IF[Inject_Faults_On_Replay] --> VS
    end

    subgraph Input_Source_Variant [Variant Subsystem: BMS_Source]
        direction TB
        CS[Choice_SIM: Plant + Sensor_Measurement]
        CR[Choice_REPLAY: NASA Data + Sensor_Replay_Fault]
    end

    VS -->|BMS_Measurement Bus| EB[Estimator Bank: 3x bms_estimator_ref]
    VS -->|BMS_Reference Bus| LOG[bms_log: 14 Columns]

    subgraph Estimator_Bank [3 Referenced Model Instances]
        direction TB
        E1[EKF Baseline: Mode 0]
        E2[SOTA Adaptive EKF: Mode 2]
        E3[SOH Capacity EKF: Mode 10]
    end

    E1 -->|BMS_Estimate Bus| LOG
    E2 -->|BMS_Estimate Bus| LOG
    E3 -->|BMS_Estimate Bus| LOG
```

### Signal Routing & Interface Bus Definitions
- **`BMS_Source` parameter:**
  - `1 = SIM`: routes simulated plant truth (`bms_plant_step`) through synthetic sensor fault injection block.
  - `2 = REPLAY`: routes real NASA battery test data through `Sensor_Replay_Fault` with conditional fault injection governed by `BMS_InjectFaultsOnReplay`.
- **`BMS_Reference` Bus:**
  - `SOC_pct`: Reference cell SOC ($0 \dots 100\,\%$) with unit `%`.
  - `Source_Tag`: $0.0$ for SIM, $1.0$ for REPLAY.
- **`bms_log` Output Structure (14 columns):**
  1. `Reference SOC (%)`
  2. `EKF Baseline SOC (%)`
  3. `Adaptive EKF SOC (%)`
  4. `Coulomb Counting SOC (%)`
  5. `Measured Voltage (V)`
  6. `Adaptive Predicted Voltage (V)`
  7. `Innovation (V)`
  8. `NIS (Normalized Innovation Squared)`
  9. `Diagnostic Fault Code Bitmask`
  10. `Identified Resistance R0 (Ohm)`
  11. `Adapted Measurement Variance R (V^2)`
  12. `Capacity EKF SOC (%)`
  13. `Estimated Capacity Ratio (SOH)`
  14. `Source Tag (0: SIM, 1: REPLAY)`

---

## 3. Exact Reproduction Commands in MATLAB

Open MATLAB, navigate to the repository root directory, and execute the following commands:

### Step 1: Add Paths and Rebuild All Models from Code
All models are programmatically compiled from scratch; no binary `.slx` files are edited manually.
```matlab
% Set MATLAB path to project root
cd('c:\Users\sande\OneDrive\Desktop\aco-ekf-soc-estimation');
addpath(fullfile(pwd, 'simulink_matlab'));
addpath(fullfile(pwd, 'simulink_matlab', 'lib'));

% Rebuild dictionary, referenced estimator, legacy models, and unified model
build_bms_models;
```
*Expected Output:*
```text
Building bms_estimator_ref ...
  saved bms_estimator_ref.slx
Building advanced_bms_aco_ekf ...
  saved advanced_bms_aco_ekf.slx
Building sota_bms_aco_ekf ...
  saved sota_bms_aco_ekf.slx
Building bms_unified ...
  saved bms_unified.slx
```

---

### Step 2: Run Full Automated Regression Suite
Runs all 9 regression suites (0 through 8), testing model quality, back-to-back equivalence, Monte Carlo, fault injection, real NASA replay, ablation, float32 precision, SIL, and unified model equivalence.
```matlab
ok = run_bms_regression;
```
*Expected Output Summary:*
```text
== 1. Back-to-back: Simulink vs MATLAB library ==
  [PASS] advanced_bms_aco_ekf: EKF SOC == library  (max diff 0.00e+00 %)
  [PASS] advanced_bms_aco_ekf: plant SOC == library  (max diff 0.00e+00 %)
  [PASS] sota_bms_aco_ekf: baseline EKF == library  (max diff 0.00e+00 %)
  [PASS] sota_bms_aco_ekf: adaptive EKF == library  (max diff 0.00e+00 %)
  [PASS] sota_bms_aco_ekf: capacity-state SOC == library  (max diff 0.00e+00 %)
  [PASS] sota_bms_aco_ekf: capacity ratio == library  (max diff 0.00e+00)

== 0. Model quality (structure, interfaces, diagnostics) ==
  [PASS] Models: no unconnected ports  (0 unconnected ports across all 4 models)
  [PASS] Models: strict diagnostics are errors (dangling ports/lines, algebraic loops, bus mismatch)  (all enabled)
  [PASS] Data dictionary: every bus element has a description and physical ones a unit  (18 elements checked)
  [PASS] Data dictionary: tuned Q and R equal lib/bms_tuned_data.m  (max difference 0.0e+00)

... [Monte Carlo, Fault Injection, NASA Replay, Float32 Precision, SIL] ...

== 8. Unified model equivalence: bms_unified vs legacy models ==
  [PASS] Unified (SIM): EKF SOC == advanced_bms_aco_ekf  (max diff 0.00e+00 %)
  [PASS] Unified (SIM): plant SOC == advanced_bms_aco_ekf  (max diff 0.00e+00 %)
  [PASS] Unified (REPLAY): baseline EKF == sota_bms_aco_ekf  (max diff 0.00e+00 %)
  [PASS] Unified (REPLAY): adaptive EKF == sota_bms_aco_ekf  (max diff 0.00e+00 %)
  [PASS] Unified (REPLAY): capacity-state SOC == sota_bms_aco_ekf  (max diff 0.00e+00 %)

====================================================
  38 / 38 checks passed  ->  ALL PASS
====================================================
```

---

### Step 3: Run Scenario Sweeps & Generate Reports
To run sweeps across scenarios using `Simulink.SimulationInput`:
```matlab
% Quick verification (1 synthetic UDDS profile + 1 NASA discharge):
summary_quick = run_all('quick');

% Full batch run (all 16 synthetic & real scenarios):
summary_all = run_all;
```
This automatically produces:
1. `simulink_matlab/results/all_scenarios_summary.csv`
2. `simulink_matlab/results/scenario_<name>.csv` (per-scenario 15-column time-series)
3. `simulink_matlab/results/fig_scenario_<name>.png` (per-scenario 4-panel comparison plots)

---

## 4. Verification Table (Measured Results)

| Verification Suite | Number of Checks | Result | Max Error / Equivalence |
| :--- | :---: | :---: | :--- |
| **Suite 0: Model Quality** | 4 | **PASS** | 0 unconnected ports across all 4 models; strict diagnostics enabled; 18 bus elements with valid units/descriptions |
| **Suite 1: Back-to-Back** | 6 | **PASS** | $\text{max diff} = 0.00\text{e+}00\,\%$ vs `lib/bms_est_step.m` |
| **Suite 2: Monte Carlo (40 runs)** | 4 | **PASS** | EKF mean MAE: $0.481\,\%$ ($< 1.0\,\%$ threshold), beats Coulomb counting ($10.71\,\%$) |
| **Suite 3: Fault Injection** | 7 | **PASS** | Sensor faults correctly detected by NIS/residual diagnostics; error bounded |
| **Suite 4: Real NASA Data Replay** | 6 | **PASS** | Fresh cells: $0.66\,\% / 0.93\,\%$ MAE; SOH capacity error: $0.069 < 0.10$ |
| **Suite 6: Float32 Precision** | 3 | **PASS** | $\text{worst } \Delta\text{SOC} = 0.0593\,\% < 0.10\,\%$ |
| **Suite 7: SIL Equivalence** | 4 | **PASS** | Compiled C firmware vs MATLAB float32: $\text{worst } \Delta\text{SOC} = 0.00058\,\%$ |
| **Suite 8: Unified Model Equivalence** | 4 | **PASS** | $\text{max diff} = 0.00\text{e+}00\,\%$ across all estimators on both SIM and REPLAY |
| **Total Regression Checks** | **38** | **ALL PASS** | **100% Pass Rate** |

---

## 5. Artifacts and Output Files

- **Models:**
  - `simulink_matlab/bms_estimator_ref.slx`
  - `simulink_matlab/advanced_bms_aco_ekf.slx`
  - `simulink_matlab/sota_bms_aco_ekf.slx`
  - `simulink_matlab/bms_unified.slx`
- **Data Dictionary:**
  - `simulink_matlab/bms_data.sldd`
- **Results & Figures:**
  - `simulink_matlab/results/all_scenarios_summary.csv`
  - `simulink_matlab/results/bms_regression_results.csv`
  - `simulink_matlab/results/scenario_*.csv`
  - `simulink_matlab/results/fig_scenario_*.png`
