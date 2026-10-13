# Changelog

All notable changes to the **Thermal-Adaptive 2-RC MO-ACO-EKF BMS Project** are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased] - Phase 1 Documentation Hardening

### Added
- `tools/make_results_table.py`: Automated tool to generate reproducible Markdown benchmark tables directly from `simulink_matlab/results/bms_regression_results.csv`.
- `results/baseline_regression_20261013.txt`: Baseline verification log documenting 33/33 passing checks before Phase 1.
- `LICENSE`: MIT open-source license.
- `CONTRIBUTING.md`: Engineering workflow, branch standards, coding guidelines.
- `pyproject.toml` & `environment.yml`: Modern Python packaging and conda environment specifications.
- `docs/architecture.md`: One-page architecture diagram and data-flow document.

### Changed
- `README.md`:
  - Replaced idealized synthetic metrics with verifiable dual tables: **Table 1: Model-Matched Synthetic Benchmarks** and **Table 2: Real Hardware Validation (NASA Ames Laboratory)**.
  - Corrected state-space description to **4-state Joseph-form formulation** with optional capacity state.
  - Corrected Simulink plant description from "2-subsystem" to **3-subsystem architecture** (`Battery_Physics_Plant`, `Sensor_Measurement`, `ACO_EKF_Controller`).
  - Standardized terminology from "SOTA" to **Adaptive EKF (Sage-Husa R)**.
- Renamed `data/NASA_B0005.csv` to `data/SYNTHETIC_B0005.csv` to make it clear that it contains synthetic test-bench data, distinguishing it from physical raw `.mat` files in `data_raw/`.
- `requirements.txt`: Pinned exact package versions (`numpy==1.26.4`, `scipy==1.15.3`, `matplotlib==3.10.6`, etc.).
- Relocated duplicate master notebooks from repo root strictly into `docs/` and git-ignored on-demand `.docx` binaries.

---

## [0.1.0] - 2026-10-12: Core Engineering Release

### Added
- 4-State Joseph-form EKF in MATLAB (`simulink_matlab/lib/bms_est_step.m`) and ANSI C (`c_firmware/bms_ekf.c`).
- Continuous Multi-Objective Ant Colony Optimization ($\text{ACO}_{\mathbb{R}}$) for $(Q, R)$ tuning.
- Arrhenius dynamic thermal parameter scaling on $R_0(T), R_1(T), R_2(T)$.
- Simulink Model-Based Design models: `advanced_bms_aco_ekf.slx` and `sota_bms_aco_ekf.slx`.
- Live interactive animation GUI dashboard (`animate_bms_live.m`).
- Automated 33-check regression test suite (`run_bms_regression.m`).
