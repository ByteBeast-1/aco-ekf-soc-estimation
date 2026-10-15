# 🔋 2-RC Thermal MO-ACO-EKF SOC Estimation System

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](pyproject.toml)
[![MATLAB R2024b](https://img.shields.io/badge/MATLAB-R2024b-orange.svg)](simulink_matlab/)
[![Simulink Models](https://img.shields.io/badge/Simulink-MBD-red.svg)](simulink_matlab/)
[![Embedded C](https://img.shields.io/badge/C99-gcc%20--Wall%20--Wextra%20--Wconversion%20clean-brightgreen.svg)](c_firmware/)
[![Regression Suite](https://img.shields.io/badge/Regression-54%2F54%20PASS-brightgreen.svg)](simulink_matlab/results/bms_regression_results.csv)

A model-based design, automotive-style, multi-platform engineering implementation of the **Thermal-Adaptive 2-RC Multi-Objective Ant Colony Optimized Extended Kalman Filter (MO-ACO-EKF)** for Lithium-Ion Battery State-of-Charge (SOC) and State-of-Health (SOH) estimation.

Extends the foundational research of:
> Kannan M, Sundareswaran K, et al., *"Ant Colony Optimized Extended Kalman Filter for State of Charge Estimation of Lithium-Ion Battery"*, **IEEE Transactions on Instrumentation and Measurement**, Vol. 74, 2025.

---

## 🌟 4 Key Engineering Novelties

1. **Arrhenius Thermal-Adaptive 2-RC Physics ($T$-2RC):** Dynamic parameter tracking $R_0(T), R_1(T), R_2(T)$ solving the IEEE basepaper's fixed-temperature limitation. *(Note: The Python research pipeline uses a 3-state vector $\mathbf{x} = [\text{SOC}, V_1, V_2]^T$; the MATLAB/Simulink MBD suite uses an extended 4-state formulation $\mathbf{x} = [\text{SOC}, V_1, V_2, \text{CapacityRatio}]^T$ with an optional online capacity state).*
2. **Multi-Objective Ant Colony Optimization (MO-ACOR):** Dual-objective continuous AI swarm optimization in log-space minimizing both steady-state tracking error (MSE) and initial offset convergence time.
3. **Joseph-Form 32-Bit ANSI C Firmware:** Zero-malloc, statically bounded C99 code (`<250 bytes` RAM state) verified clean under `gcc -Wall -Wextra -Wconversion -pedantic` and matching MATLAB via Software-in-the-Loop (SIL) within $0.00058\%$ SOC discrepancy.
4. **Model-Based Design (MBD) Architecture:** Complete 3-subsystem visual block diagram (`Battery_Physics_Plant`, `Sensor_Measurement`, and `ACO_EKF_Controller`) coupled via typed bus objects in `bms_data.sldd`.

---

## 📁 Repository Structure

```
aco-ekf-soc-estimation/
├── README.md                        # Project landing page with verified empirical metrics
├── LICENSE                          # MIT open-source license
├── CHANGELOG.md                     # Semantic versioning changelog
├── CONTRIBUTING.md                  # Development, branch, and verification guidelines
├── pyproject.toml, environment.yml  # Modern Python packaging and conda environment
├── requirements.txt                 # Pinned dependencies
│
├── python/                          # Python research pipeline (run from repo root)
│   ├── main.py                      #   Master pipeline: data -> MO-ACO -> EKF -> figures
│   ├── battery_model.py             #   Thermal-adaptive 2-RC ECM + 8th-order OCV
│   ├── ekf.py                       #   Joseph-form Extended Kalman Filter
│   ├── ukf.py                       #   Unscented Kalman Filter benchmark
│   ├── aco_optimizer.py             #   Multi-objective continuous ACO (MO-ACOR)
│   ├── adaptive_hybrid_ekf.py       #   Adaptive hybrid EKF (Sage-Husa R + RLS + NN residual)
│   ├── online_rls.py, residual_nn.py#   Online RLS parameter ID, residual MLP corrector
│   ├── data_generator.py            #   4 dynamic operating profiles with thermal gradients
│   ├── benchmark_suite.py           #   4-way comparative benchmark suite
│   ├── nasa_benchmark_runner.py     #   Benchmark on data/SYNTHETIC_B0005.csv
│   ├── main_ultimate.py             #   Adaptive pipeline (RLS + Sage-Husa + NN)
│   └── visualization.py
│
├── simulink_matlab/                 # MATLAB & Simulink MBD models (see docs/Simulink_Upgrade.md)
│   ├── advanced_bms_aco_ekf.slx     #   MAIN MODEL: simulated plant + faultable sensors + ACO-tuned EKF
│   ├── sota_bms_aco_ekf.slx         #   REAL NASA REPLAY: EKF vs Adaptive EKF vs SOH capacity state
│   ├── animate_bms_live.m           #   Live high-contrast animation dashboard
│   ├── lib/                         #   Shared algorithm library (plant, sensors, EKF, ACO tuner, NASA loader)
│   ├── build_bms_models.m           #   Programmatically regenerates both models from lib/
│   ├── run_simulink.m, run_sota_simulink.m
│   ├── run_bms_regression.m         #   54-check automated verification suite
│   └── results/                     #   Regression CSV, baseline logs, block-diagram snapshots
│
├── c_firmware/                      # Embedded ANSI C firmware (<250 B RAM)
│   ├── bms_ekf.h, bms_ekf.c         #   EKF + diagnostics, mirrors lib/bms_est_step.m
│   ├── main_test.c, sil/            #   Unit smoke test, software-in-the-loop driver
│   └── README.md
│
├── data/                            # Benchmark CSVs (see data/README.md)
│   ├── SYNTHETIC_B0005.csv          #   Synthetic 2 A discharge test bench for Python
│   └── NASA_B0005_OFFICIAL_Cycle1.csv#  Real measured cycle #1 extracted from B0005.mat
├── data_raw/                        # Official NASA Ames B0005/B0006/B0007 .mat laboratory data
├── results/                         # Regression outputs & baseline verification logs
├── figures/                         # Publication-ready figures (00-16)
├── tools/                           # Tooling (make_results_table.py, architecture generators)
└── docs/                            # In-depth engineering documentation
    ├── architecture.md              #   System architecture & signal bus layout
    ├── Complete_Project_Guide.md    #   Mathematical derivations and block breakdown
    ├── Master_Study_Plan.md         #   Viva defense and study roadmap
    ├── SIMULINK_MASTER_NOTEBOOK.md  #   Advanced BMS model specification
    └── SOTA_BMS_MASTER_NOTEBOOK.md  #   Real-data replay specification
```

---

## 📊 Performance Benchmarks

<!-- RESULTS_TABLE_START -->

### Table 1: Model-Matched Synthetic Benchmarks (40 Randomized Monte Carlo Runs)

> [!NOTE]
> **Note on Synthetic Results:** These synthetic metrics evaluate an ideal 2-RC Arrhenius plant with random parameter scale shifts, initial guess offsets ($\pm 20\%$), and sensor noise. Because the estimator physics model structurally matches the simulated plant, synthetic numbers are optimistic compared to real electrochemistry.

| Estimator Algorithm | Mean MAE (%) | Worst-Case MAE (%) | 95th Percentile MAE (%) | Worst Max Error (%) |
| :--- | :---: | :---: | :---: | :---: |
| **Coulomb Counting** ($\pm 20\%$ init error) | 10.71% | 20.80% | 18.22% | 21.50% |
| **ACO-Tuned EKF** (Mode 0) | **0.481%** | **1.153%** | **0.982%** | **1.852%** |
| **Adaptive EKF** (Sage-Husa $R$, Mode 2) | 0.481% | 1.184% | 1.006% | 2.124% |

---

### Table 2: Real Measured Cell Data Validation (NASA Ames Laboratory Cells, Out-of-Sample)

> [!IMPORTANT]
> **Experimental Data Integrity:** Model parameters and ACO covariances were calibrated strictly on Cell **B0006** (cycles #1 and #60). Cells **B0005** and **B0007** (and aged cycles) are completely **unseen, out-of-sample real measured cell data tests** from the NASA Ames Prognostics Center of Excellence.

| Cell & Cycle Condition | Delivered Capacity | Coulomb Counting MAE (%) | ACO-Tuned EKF MAE (%) | Adaptive EKF (Sage-Husa) MAE (%) | EKF Max Error (%) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **B0005 #1** | 1.86 Ah | 10.74% | **0.66%** | **0.72%** | 2.21% |
| **B0005 #60** | 1.67 Ah | 6.60% | **3.65%** | **3.53%** | 6.82% |
| **B0005 #120** | 1.40 Ah | 7.26% | **4.89%** | **4.71%** | 9.44% |
| **B0007 #1** | 1.89 Ah | 11.94% | **0.93%** | **1.11%** | 2.47% |
| **B0007 #60** | 1.71 Ah | 8.01% | **2.74%** | **2.64%** | 4.97% |
| **B0007 #120** | 1.51 Ah | 6.04% | **3.78%** | **3.65%** | 7.26% |
| **B0006 #120** | 1.36 Ah | 7.64% | **2.28%** | **2.43%** | 5.84% |

---

### Table 3: Fault Injection Diagnostic Coverage (UDDS Drive Cycle)

| Injected Fault Mode ($t=900\text{--}1200\text{ s}$) | Filter MAE (%) | Max Error (%) | Diagnostic Flag Status | Safety Behavior |
| :--- | :---: | :---: | :---: | :--- |
| **Mode 0: Normal (No Fault)** | 0.146% | 0.40% | Clear (0) | Baseline nominal tracking |
| **Mode 1: Current Sensor Bias (+0.25 A)** | 0.371% | 0.75% | **Detected (Bit 32: Current Offset)** | Flags persistent bias within 204 s; error reaches 0.75% |
| **Mode 2: Voltage Sensor Glitches ($\pm 0.5\text{ V}$)** | 0.144% | 0.40% | **Detected (Bit 8: Residual Rejected)** | Outliers rejected by NIS gating; zero error impact |
| **Mode 3: Thermistor Disconnect (Open Circuit)** | 0.147% | 0.40% | **Detected (Bit 2: Temp Out of Range)** | Holds last good temperature (`T = st.T_last`) |

> [!NOTE]
> **Convergence Window Exclusion & Offset Sensitivity:** All error metrics (MAE and max error) in the tables above exclude the initial 120 s convergence window (`skip = 120 s` in `simulink_matlab/run_bms_regression.m`), evaluating steady-state performance once initial offsets have settled. The measured minimum current offset that the bit-32 detector flags is $\pm 0.25\text{ A}$ (reliably flagged within $204\text{ s}$ on UDDS), whereas sub-threshold offsets $\le \pm 0.10\text{ A}$ produce smoothed innovation bias below the $5.5\text{ mV}$ threshold and remain unflagged.

---

### Table 4: Cross-Feeding Estimator Matrix (Simulated Plant vs. Real NASA Replay with Faults)

> [!NOTE]
> **Cross-Feeding Methodology & Metric Definitions:**
> Evaluates all three estimator variants (Baseline EKF, Adaptive EKF (Sage-Husa), and Online Capacity SOH EKF) across simulated plant conditions (known truth, sensor faults, 40-run Monte Carlo ensemble) and real NASA experimental cell replay with and without sensor faults injected (`BMS_InjectFaultsOnReplay = 1`). Metrics exclude the initial 120 s convergence window:
> - **Time to $\le 1\%$ (stay 60s) (Baseline EKF):** Time in seconds for Baseline EKF to first enter $|err| \le 1.0\%$ and remain $\le 1.0\%$ for at least 60 consecutive seconds. Reported as `N/A (>1%)` if never maintained (no looser fallback threshold).
> - **Post-120s Err $>1\%$ (Baseline EKF):** Fraction/percentage of samples strictly after the 120 s convergence window ($t > 120\text{ s}$) where Baseline EKF absolute error exceeds $1.0\%$.
> - **Fault Diagnostics:** Evaluates per-spike detection for periodic glitches (spikes occur every 25 samples between $t = 899\text{ s}$ and $1199\text{ s}$, 13 spikes total) reporting fraction of spikes flagged and first-flag delay after the first spike.

> [!IMPORTANT]
> **Summary:** Baseline EKF is best on healthy fresh cells and requires no tuning. Across all six aged NASA cycles (cycles #60 and #120 of B0005, B0006, and B0007), online Capacity SOH EKF achieves a mean MAE of 3.096% vs 3.136% for baseline EKF (-0.039 pp shift), showing that capacity adaptation provides only modest cell-dependent gains and does not consistently improve over baseline across all cells (e.g., slightly higher error on aged B0006). Adaptive EKF (Sage-Husa) shows no clear gain on real data.

| Scenario Description | Data Source | Fault Mode | Baseline EKF MAE | Adaptive EKF MAE | Capacity SOH EKF MAE | Worst Max Err | Time to $\le 1\%$ (stay 60s) (Baseline EKF) | Post-120s Err $>1\%$ (Baseline EKF) | Fault Diagnostics |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **sim_clean_udds** | `synthetic` | Mode 0 | 0.163% | 0.166% | 0.163% | 0.71% | 2 s | 0.0% | None |
| **sim_pulse** | `synthetic` | Mode 0 | 0.112% | 0.114% | 0.112% | 0.70% | 2 s | 0.0% | None |
| **sim_fault1_I_offset** | `synthetic` | Mode 1 | 0.387% | 0.364% | 0.387% | 0.74% | 2 s | 0.0% | **Detected (Bit 32)** |
| **sim_fault2_V_spike** | `synthetic` | Mode 2 | 0.162% | 0.166% | 0.162% | 0.71% | 2 s | 0.0% | **100% spikes (delay 0s)** |
| **sim_fault3_T_open** | `synthetic` | Mode 3 | 0.160% | 0.163% | 0.160% | 0.71% | 2 s | 0.0% | **Instant (delay 0s)** |
| **sim_monte_carlo** | `synthetic` | Mode 0 | 0.169% | 0.180% | 0.169% | 1.16% | 24 s | 0.0% | None |
| **nasa_B0005_clean** | `nasa` | Mode 0 | 0.664% | 0.721% | 0.664% | 2.49% | 689 s | 17.7% | None |
| **nasa_B0005_fault1** | `nasa` | Mode 1 | 0.644% | 0.771% | 0.644% | 2.49% | 689 s | 20.8% | **Detected (Bit 32)** |
| **nasa_B0005_fault2** | `nasa` | Mode 2 | 0.664% | 0.721% | 0.664% | 2.49% | 689 s | 17.7% | **100% spikes (delay 0s)** |
| **nasa_B0005_fault3** | `nasa` | Mode 3 | 0.669% | 0.722% | 0.669% | 2.49% | 689 s | 17.7% | **Instant (delay 0s)** |
| **nasa_B0007_clean** | `nasa` | Mode 0 | 0.929% | 1.110% | 0.929% | 2.57% | 460 s | 26.1% | None |
| **nasa_B0005_aged_120** | `nasa` | Mode 0 | 4.893% | 4.714% | 4.712% | 9.44% | 282 s | 85.2% | None |


---

### Table 5: Hyperparameter Tuner Benchmark Comparison (Equal Evaluation Budget & Search Bounds)

> [!NOTE]
> **Fair Benchmark Methodology & Search Bounds:**
> Compares 5 hyperparameter discovery methods on the identical 4 log-parameters ($Q_{\mathrm{SOC}}, Q_{V_1}, Q_{V_2}, R$), identical search bounds ($\log_{10} Q \in [-9, -4]$, $\log_{10} R \in [-4, -1]$), and identical fitness evaluation function. Evaluates generalization on held-out test scenarios (unseen drive cycle seeds for synthetic plant; unseen cells B0005 & B0007 for NASA replay) strictly excluding the first 120 s convergence window.
> - **Stochastic Methods (ACO_R, Random Search):** Evaluated across 5 random seeds (reported as mean $\pm$ std, [min, max]).
> - **Deterministic Methods (Hand-Tuned, Grid Search, Nelder-Mead):** Run once with single deterministic result.
> - **Skipped Methods:** Genetic Algorithm (`ga`) and Particle Swarm (`particleswarm`) were skipped because MATLAB Global Optimization Toolbox is not installed in the execution environment.

> [!IMPORTANT]
> **Tuner Benchmark Summary:** Synthetic plant winner: **ACO R** (mean held-out MAE: 0.159%); Real NASA replay winner: **Grid Search** (mean held-out MAE: 1.998%).

| Optimization Algorithm | Type | Evaluations | Synthetic Held-Out MAE (%) | NASA Held-Out MAE (%) | Mean Runtime (s) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Hand-Tuned Baseline** | Deterministic | 1 | 0.272% | 5.260% | 0.2 s |
| **Random Search** | Stochastic (5 seeds) | 200 | 0.163% ± 0.003% [0.159%, 0.166%] | 2.980% ± 0.161% [2.759%, 3.187%] | 35.7 s |
| **Grid Search** | Deterministic | 192 | 0.161% | **1.998%** | 29.8 s |
| **Nelder-Mead (fminsearch)** | Deterministic | 200 | 0.160% | 3.172% | 27.0 s |
| **Continuous ACO (ACO_R)** | Stochastic (5 seeds) | 200 | **0.159% ± 0.000% [0.159%, 0.160%]** | 2.714% ± 0.190% [2.382%, 2.834%] | 78.0 s |


---

### Table 6: Widened Search Bounds Benchmark (10 Seeds, Widened by 1 Decade on Each Side)

> [!NOTE]
> **Widened Search Space Specification & Evaluation Setup:**
> - Search space widened by 1 decade on each side: $\log_{10} Q_i \in [-10, -3]$ (previously $[-9, -4]$) and $\log_{10} R \in [-5, 0]$ (previously $[-4, -1]$).
> - Evaluated across 10 independent random seeds for ACO_R and Random_Search, and uniform Grid_Search.
> - **Per-cell Breakdown:** Evaluated on unseen NASA cells B0005 and B0007 across cycles #1, #60, and #120, reporting fresh (#1) vs aged (#120) performance.
> - **Overfitting Diagnostics:** Reports training fitness next to held-out MAE to quantify optimization vs generalization gaps.

| Optimization Method | Source | Seeds | Mean Training Cost | Held-Out MAE (%) | Fresh Cells (#1) MAE | Aged Cells (#120) MAE | Bounds Saturation Rate |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Ant Colony Optimization (ACO_R)** | Synthetic | 10 | 1.42e-04 | 0.155% ± 0.000% [0.155%, 0.155%] | N/A (model-matched) | N/A (model-matched) | 50.0% (20/40 params) |
| **Uniform Grid Search** | Synthetic | 1 | 1.42e-04 | 0.156% | N/A (model-matched) | N/A (model-matched) | 75.0% (3/4 params) |
| **Random Uniform Search** | Synthetic | 10 | 1.42e-04 | 0.158% ± 0.006% [0.153%, 0.173%] | N/A (model-matched) | N/A (model-matched) | 5.0% (2/40 params) |
| **Ant Colony Optimization (ACO_R)** | Real NASA | 10 | 6.51e-04 | 5.219% ± 4.242% [2.349%, 15.210%] | 4.21% | 4.93% | 30.0% (12/40 params) |
| **Uniform Grid Search** | Real NASA | 1 | 7.90e-04 | 2.295% | 0.50% | 3.24% | 50.0% (2/4 params) |
| **Random Uniform Search** | Real NASA | 10 | 8.03e-04 | 4.677% ± 4.181% [1.812%, 15.463%] | 3.87% | 4.50% | 10.0% (4/40 params) |


---

### Table 7: Leave-One-Cell-Out Cross-Validation Benchmark (NASA Cells B0005, B0006, B0007)

> [!NOTE]
> **Leave-One-Cell-Out Cross-Validation Setup (phase3-cv):**
> - **Folds:** Rotate across all 3 NASA cells: tune on 2 cells' cycles #1, #60, and #120 (6 cycles); test on the 3rd cell's cycles #1, #60, and #120.
> - **Methods:** Hand-Tuned baseline, Uniform Grid Search, Random Search (10 seeds), ACO_R (10 seeds, original bounds), and ACO_R with validation-based pick (`ACO_R_Val_Pick`, picking best seed on unseen cycle #30 outside train/test).
> - **Budget:** Same evaluation budget across all methods.
> - **Held-Out Generalization vs. In-Sample Notice:** The in-sample re-evaluation of `cv_best` on B0005/B0007 yields 1.869% (and 4.893% -> 2.224% on B0005 #120); these are in-sample figures. The true held-out cross-cell generalization metric is the **3.657%** cross-fold mean.

| Method | Test Cell | Mean MAE (%) | Median MAE (%) | Std MAE (%) | Worst Seed (MAE) | Fresh Cells (#1) [Med / Worst] | Aged Cells (#120) [Med / Worst] |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Hand-Tuned Baseline** | B0005 | 4.892% | 4.892% | 0.000% | 4.892% (N/A) | 3.552% (N/A) | 8.400% (N/A) |
| **Hand-Tuned Baseline** | B0006 | 11.435% | 11.435% | 0.000% | 11.435% (N/A) | 4.827% (N/A) | 19.706% (N/A) |
| **Hand-Tuned Baseline** | B0007 | 4.473% | 4.473% | 0.000% | 4.473% (N/A) | 3.654% (N/A) | 6.578% (N/A) |
| *--> Hand-Tuned Baseline (Cross-Fold Mean)* | **ALL 3 CELLS** | **6.933%** | — | — | — | **4.011%** | **11.562%** |
| **Grid Search** | B0005 | 3.061% | 3.061% | 0.000% | 3.061% (N/A) | 0.828% (N/A) | 4.894% (N/A) |
| **Grid Search** | B0006 | 6.757% | 6.757% | 0.000% | 6.757% (N/A) | 3.879% (N/A) | 11.999% (N/A) |
| **Grid Search** | B0007 | 2.482% | 2.482% | 0.000% | 2.482% (N/A) | 1.207% (N/A) | 3.704% (N/A) |
| *--> Grid Search (Cross-Fold Mean)* | **ALL 3 CELLS** | **4.100%** | — | — | — | **1.971%** | **6.866%** |
| **Random Search (10 seeds)** | B0005 | 2.763% | 2.839% | 0.324% | 3.199% (#79) | 1.290% / 1.821% | 4.181% / 5.433% |
| **Random Search (10 seeds)** | B0006 | 6.727% | 6.713% | 0.895% | 8.062% (#9999) | 4.234% / 4.403% | 11.135% / 14.545% |
| **Random Search (10 seeds)** | B0007 | 2.340% | 2.442% | 0.189% | 2.508% (#53) | 2.444% / 2.577% | 3.049% / 3.320% |
| *--> Random Search (10 seeds) (Cross-Fold Mean)* | **ALL 3 CELLS** | **3.943%** | — | — | — | **2.611%** | **6.198%** |
| **ACO_R (10 seeds)** | B0005 | 2.602% | 2.522% | 0.250% | 3.124% (#2024) | 0.975% / 1.673% | 3.981% / 4.938% |
| **ACO_R (10 seeds)** | B0006 | 7.362% | 7.561% | 0.502% | 7.850% (#79) | 3.848% / 4.444% | 13.999% / 14.646% |
| **ACO_R (10 seeds)** | B0007 | 2.160% | 2.078% | 0.202% | 2.603% (#101) | 2.221% / 2.684% | 2.766% / 3.451% |
| *--> ACO_R (10 seeds) (Cross-Fold Mean)* | **ALL 3 CELLS** | **4.041%** | — | — | — | **2.304%** | **6.756%** |
| **ACO_R Validation-Pick** | B0005 | 2.611% | 2.611% | 0.000% | 2.611% (#101) | 0.627% / 0.627% | 4.043% / 4.043% |
| **ACO_R Validation-Pick** | B0006 | 6.372% | 6.372% | 0.000% | 6.372% (#137) | 3.934% / 3.934% | 11.020% / 11.020% |
| **ACO_R Validation-Pick** | B0007 | 1.987% | 1.987% | 0.000% | 1.987% (#13) | 2.364% / 2.364% | 2.273% / 2.273% |
| *--> ACO_R Validation-Pick (Cross-Fold Mean)* | **ALL 3 CELLS** | **3.657%** | — | — | — | **2.308%** | **5.779%** |


<!-- RESULTS_TABLE_END -->

*(Note: The above tables are automatically populated from `simulink_matlab/results/bms_regression_results.csv` via `python tools/make_results_table.py --update README.md`.)*

---

## 🚀 Quickstart Guide

### 1. Run Python AI Pipeline
```bash
python python/main.py
python python/benchmark_suite.py
python python/nasa_benchmark_runner.py
```

### 2. Run MATLAB & Simulink Simulation
In MATLAB Command Window:
```matlab
cd simulink_matlab
run_simulink('udds', 0)         % Simulated plant + EKF model (profile, fault mode, init error, seed)
run_sota_simulink('B0005', 1)   % Replay real NASA discharge through Simulink model
animate_bms_live                % Launch interactive real-time dashboard GUI
run_bms_regression              % Run full 54-check automated test harness
build_bms_models                % Programmatically regenerate models from lib/
```

### 3. Compile Embedded ANSI C Firmware
```bash
gcc -std=c99 -O2 -Wall -Wextra -Wconversion -pedantic c_firmware/bms_ekf.c c_firmware/main_test.c -o c_firmware/bms_ekf_test.exe -lm
./c_firmware/bms_ekf_test.exe
```
