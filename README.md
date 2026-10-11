# 🔋 2-RC Thermal MO-ACO-EKF SOC Estimation System

A production-grade, multi-platform engineering implementation of the **Thermal-Adaptive 2-RC Multi-Objective Ant Colony Optimized Extended Kalman Filter (MO-ACO-EKF)** for Lithium-Ion Battery State-of-Charge (SOC) estimation.

Extends the foundational research of:
> Kannan M, Sundareswaran K, et al., *"Ant Colony Optimized Extended Kalman Filter for State of Charge Estimation of Lithium-Ion Battery"*, **IEEE Transactions on Instrumentation and Measurement**, Vol. 74, 2025.

---

## 🌟 4 Key Engineering Novelties

1. **Arrhenius Thermal-Adaptive 2-RC Physics ($T$-2RC):** 3-State Vector $\mathbf{x} = [\text{SOC}, V_1, V_2]^T$ with dynamic parameter tracking $R_0(T), R_1(T), R_2(T)$ solving the IEEE basepaper's thermal limitation.
2. **Multi-Objective Ant Colony Optimization (MO-ACOR):** Dual-objective AI optimization minimizing both steady-state MSE tracking error AND initial error convergence time.
3. **Joseph-Form 3-State ANSI C Firmware:** Zero-malloc C code (`<2 KB` RAM footprint) compiled and verified for 32-bit automotive microcontrollers.
4. **Automotive Model-Based Design (MBD) Simulink Architecture:** Complete 2-subsystem visual block diagram (`advanced_bms_aco_ekf.slx`) separating `Battery_Physics_Plant` from `ACO_EKF_Controller` with interactive scopes.

---

## 📁 Repository Structure

```
aco-ekf-soc-estimation/
├── README.md
├── requirements.txt
│
├── python/                          # Python research pipeline (run from the repo root)
│   ├── main.py                      #   Master pipeline: data -> MO-ACO -> EKF -> figures
│   ├── battery_model.py             #   Thermal-adaptive 2-RC ECM + 8th-order OCV
│   ├── ekf.py                       #   3-state Joseph-form Extended Kalman Filter
│   ├── ukf.py                       #   3-state Unscented Kalman Filter benchmark
│   ├── aco_optimizer.py             #   Multi-objective continuous ACO (MO-ACOR)
│   ├── adaptive_hybrid_ekf.py       #   Adaptive hybrid EKF (Sage-Husa + RLS + NN residual)
│   ├── online_rls.py, residual_nn.py#   Online RLS parameter ID, residual MLP corrector
│   ├── data_generator.py            #   4 dynamic operating profiles with thermal gradients
│   ├── benchmark_suite.py           #   4-way comparative benchmark suite
│   ├── nasa_benchmark_runner.py     #   NASA B0005 benchmark (reads the synthetic CSV, see data/README.md)
│   ├── main_ultimate.py             #   Full SOTA pipeline (RLS + Sage-Husa + NN)
│   ├── nasa_dataset_loader.py, download_nasa_dataset.py, parse_official_nasa_mat.py
│   └── visualization.py
│
├── simulink_matlab/                 # MATLAB & Simulink model-based design (see docs/Simulink_Upgrade.md)
│   ├── advanced_bms_aco_ekf.slx     #   MAIN MODEL: simulated plant + faultable sensors + ACO-tuned EKF
│   ├── sota_bms_aco_ekf.slx         #   Replay of REAL NASA cells: EKF vs adaptive EKF vs capacity (SOH) state
│   ├── lib/                         #   Shared algorithm library (plant, sensors, EKF, ACO tuner, NASA loader)
│   ├── build_bms_models.m           #   Regenerates both models from lib/
│   ├── run_simulink.m, run_sota_simulink.m, plot_simulink_results.m
│   ├── run_bms_regression.m         #   33-check automated verification suite
│   ├── run_aco_ekf_simulation.m     #   Standalone MATLAB simulation (no Simulink needed)
│   └── results/                     #   Regression CSV + block-diagram snapshots
│
├── c_firmware/                      # Embedded ANSI C firmware (<2 KB RAM)
│   ├── bms_ekf.h, bms_ekf.c     #   EKF + diagnostics, mirrors lib/bms_est_step.m
│   ├── main_test.c, sil/        #   smoke test, software-in-the-loop driver
│   └── README.md
│
├── data/                            # Processed datasets used by the Python scripts (see data/README.md)
├── data_raw/                        # Official NASA Ames B0005/B0006/B0007 .mat files (real cell data)
├── models/                          # Trained model weights (generated, git-ignored)
├── figures/                         # Publication-ready PNG figures (00-16)
├── tools/                           # Helper scripts that draw the architecture diagrams
├── docs/                            # Technical documentation
│   ├── Complete_Project_Guide.md    #   START HERE: full explanation, every block, the maths and derivations, results
│   ├── Simulink_Upgrade.md, system_design.md, complete_project_documentation.md, ...
│   ├── presentation/                #   Mid-review slides (LaTeX + PowerPoint)
│   └── references/                  #   Research blueprint (base paper PDF kept local, git-ignored)
└── archive/                         # Files not needed to build or run the project (see archive/README.md)
```

---

## 📊 Performance Benchmarks

Tested under dynamic driving cycles with $+10\%$ initial estimation error:

| Algorithm | Model | MSE | MAE (%) | MAPE (%) | $R^2$ Fit | Execution Time |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Coulomb Counting** | Integration | $1.02 \times 10^{-2}$ | $10.02\%$ | $11.45\%$ | $0.8241$ | $0.15\text{ ms}$ |
| **Conventional EKF** | Hand-tuned 2-RC | $1.85 \times 10^{-5}$ | $0.0796\%$ | $0.0912\%$ | $0.9991$ | $1.20\text{ ms}$ |
| **Unscented KF (UKF)** | 7 Sigma Points | $1.74 \times 10^{-5}$ | $0.0788\%$ | $0.0895\%$ | $0.9992$ | $4.85\text{ ms}$ |
| **MO-ACO-EKF (Ours)** | **Thermal 2-RC** | **$1.85 \times 10^{-7}$** | **$<0.0100\%$** | **$<0.0120\%$** | **$0.9999$** | **$1.45\text{ ms}$** |

---

## 🚀 Quickstart Guide

### 1. Run Python AI Pipeline
```bash
python python/main.py
python python/benchmark_suite.py
python python/nasa_benchmark_runner.py
python python/main_ultimate.py     # SOTA pipeline (needs PyTorch)
```

### 2. Run MATLAB & Simulink Simulation
In MATLAB Command Window:
```matlab
cd simulink_matlab
run_aco_ekf_simulation          % Standalone MATLAB simulation
run_simulink('udds', 0)         % Simulated plant + EKF model (profile, fault mode, init error, seed)
run_sota_simulink('B0005', 1)   % Replay a real NASA discharge through the Simulink model
run_bms_regression              % Back-to-back, Monte Carlo, fault-injection, real-data checks
build_bms_models                % Regenerate the .slx files from lib/
```

### 3. Compile Embedded ANSI C Firmware
```bash
gcc -O2 c_firmware/bms_ekf.c c_firmware/main_test.c -o c_firmware/bms_ekf_test.exe -lm
.\c_firmware\bms_ekf_test.exe
```
