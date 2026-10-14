# Phase 2 Verification & Reproduction Guide (`PHASE2_RUN_ME.md`)

This document details the architecture, programmatic generation, and verification procedures for **Phase 2 (Items A & B)** of the BMS upgrade project as outlined in [`docs/AGENT_BRIEF.md`](../docs/AGENT_BRIEF.md).

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
| **`run_all.m`** | Created | **VERIFIED** | Executes scenario sweeps via `Simulink.SimulationInput`, outputs per-scenario time-series CSVs (15 columns), 4-panel PNG figures, and `all_scenarios_summary.csv` |
| **`run_bms_regression.m`** | Updated | **VERIFIED** | **39 / 39 checks pass** (Suites 0 through 8). Suite 8 confirms unified model reproduces legacy models within $1\text{e-}9$ and source switch is non-vacuous |

> [!NOTE]
> **Model Reference Architecture & Equivalence by Construction:**
> The legacy models (`advanced_bms_aco_ekf.slx` and `sota_bms_aco_ekf.slx`) were programmatically refactored to instantiate `bms_estimator_ref.slx`, sharing the exact same referenced block as `bms_unified.slx`. Consequently, numerical equivalence between `bms_unified.slx` and the legacy models in Suite 8 ($0.00\text{e+}00\,\%$) is partly by construction.
> **Suite 1** (which runs back-to-back Simulink simulations against the independent pure-MATLAB reference implementation [`lib/bms_est_step.m`](lib/bms_est_step.m)) provides the independent, non-circular mathematical verification.

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
- **`bms_log` Output Structure (14 machine-readable signals):**
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
  14. `Source Tag` (`BS_Reference/Source_Tag`: 0 = SIM, 1 = REPLAY)

*(Note on column counts: `bms_log` contains 14 signal columns; when exported by `run_all.m` to `results/scenario_<name>.csv`, a `Time_s` column is prepended, yielding 15 columns total in the CSV file).*

---

## 3. Exact Reproduction Commands in MATLAB

Open MATLAB, navigate to the repository root directory, and execute the following commands:

### Step 1: Add Paths and Rebuild All Models from Code
All models are programmatically compiled from scratch; no binary `.slx` files are edited manually.
```matlab
% Set working directory to repository root
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
  [PASS] Unified: source switch is non-vacuous (Source Tag 0 vs 1, distinct reference SOC)  (tags [0, 1], ref diff 15.8 %)

====================================================
  39 / 39 checks passed  ->  ALL PASS
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
1. `results/all_scenarios_summary.csv`
2. `results/scenario_<name>.csv` (per-scenario 15-column time-series: `Time_s` + 14 signals)
3. `results/fig_scenario_<name>.png` (per-scenario 4-panel comparison plots)

---

## 4. Verification Table (Measured Results Generated from Real Output)

| Verification Suite | Checks | Result | Measured Thresholds / Equivalence Summary |
| :--- | :---: | :---: | :--- |
| **Suite 0: Model Quality** | 4 | **PASS** | 0 unconnected ports across all 4 models; strict diagnostics enabled; 18 bus elements with valid units & descriptions; tuned Q/R equal dictionary ($0.0\text{e+}00$) |
| **Suite 1: Back-to-Back vs Library** | 6 | **PASS** | $\text{max diff} = 0.00\text{e+}00\,\%$ vs [`lib/bms_est_step.m`](lib/bms_est_step.m) (independent verification of EKF, plant, adaptive, and capacity states) |
| **Suite 2: Monte Carlo (40 runs)** | 4 | **PASS** | EKF mean MAE: $0.481\,\%$ ($< 1.0\,\%$ threshold), worst MAE: $1.153\,\%$ ($< 2.0\,\%$), max instantaneous: $1.852\,\%$ ($< 6.0\,\%$), beats Coulomb counting ($10.71\,\%$) |
| **Suite 3: Fault Injection** | 6 | **PASS** | Fault-free clean ($0$ flags); Fault 1 (current offset): bounded max $0.75\,\%$; Fault 2 (voltage glitch): bounded max $0.40\,\%$, detected; Fault 3 (open thermistor): bounded max $0.40\,\%$, detected |
| **Suite 4: Real NASA Data Replay** | 6 | **PASS** | Fresh unseen cells (B0005/B0007): $0.66\,\% / 0.93\,\%$ MAE ($< 2.0\,\%$); all cycles: worst $4.89\,\%$ ($< 6.5\,\%$); beats CC; charge cycles: $3.55\,\% / 3.73\,\%$ MAE; SOH capacity ratio mean error: $0.069 < 0.10$ |
| **Suite 5: Adaptation Feature Ablation** | 0 | **INFO** | Comparative ablation analysis across 8 mode configurations logged to console and `results/bms_regression_results.csv` |
| **Suite 6: Float32 Precision** | 3 | **PASS** | Worst $\Delta\text{SOC} = 0.0593\,\% < 0.10\,\%$; worst $\Delta\text{V} = 0.265\text{ mV} < 1.0\text{ mV}$; diagnostic flags identical across all 15 scenarios |
| **Suite 7: SIL Equivalence** | 4 | **PASS** | Compiled C firmware vs MATLAB float32: worst $\Delta\text{SOC} = 0.00058\,\% < 0.002\,\%$; worst $\Delta\text{V} = 0.293\text{ mV} < 1.0\text{ mV}$; relative trace difference $1.2\text{e-}04 < 1\text{e-}03$; diagnostic flags identical |
| **Suite 8: Unified Model Equivalence** | 6 | **PASS** | $\text{max diff} = 0.00\text{e+}00\,\%$ across all estimators on both SIM and REPLAY; non-vacuous switch verified (tags $[0, 1]$, ref series diff $> 1.0\,\%$) |
| **Total Regression Checks** | **39** | **ALL PASS** | **100% Pass Rate** |

---

## 5. Technical Note: SIL Determinism and "B0005 charge #1"

A minor variation in the SIL error metric for **`B0005 charge #1`** was observed between historical commits ($0.000557\,\% / 0.286\text{ mV}$) and Windows runs ($0.000580\,\% / 0.293\text{ mV}$). The root cause was investigated:

1. **Determinism within Environment:**
   Running `bms_sil_run` repeatedly within the same environment produces **100% bit-for-bit identical results** (`dsoc = 0.00057987 %, dv = 0.29325584 mV`).
2. **Absence of Stochasticity:**
   Replay scenarios use recorded NASA cell measurements directly. No noise injection or random seeds are invoked during replay execution.
3. **Cross-Platform Math Library Evaluation:**
   The variation is caused by floating-point transcendentals (`expf` in the Arrhenius temperature adjustment) across C standard libraries:
   - Historical commit `e224867` was compiled with Linux GCC linking against GNU C Library (`glibc` `libm`).
   - The current environment compiles using MSYS2 MinGW GCC linking against Windows Universal C Runtime (`UCRT` `libm`).
   Minor 1-ULP (unit in the last place) rounding differences in `expf` compound over 10,517 discrete time steps (the longest scenario in the suite, spanning ~3 hours), resulting in a negligible $0.000023\,\%$ ($0.23\text{ ppm}$) maximum instantaneous difference. Both results remain well within the $0.002\,\%$ SIL acceptance threshold.

---

## 6. Artifacts and Output Files

- **Models:**
  - `bms_estimator_ref.slx`
  - `advanced_bms_aco_ekf.slx`
  - `sota_bms_aco_ekf.slx`
  - `bms_unified.slx`
- **Data Dictionary:**
  - `bms_data.sldd`
- **Results & Figures:**
  - `results/all_scenarios_summary.csv`
  - `results/bms_regression_results.csv`
  - `results/scenario_*.csv`
  - `results/fig_scenario_*.png`
