# Contributing to 2-RC Thermal MO-ACO-EKF BMS

Thank you for contributing to the 2-RC Thermal MO-ACO-EKF Battery Management System project.  
To maintain model-based design, automotive-style code quality and reproducibility, all contributions must follow these guidelines.

---

## 🚦 Branching Strategy & Development Workflow

1. **Never commit or push directly to `main`**:
   - All work happens on dedicated feature/phase branches (e.g., `phase-1-honest-docs`, `phase-2-unified-model`).
   - Branch names must follow standard conventions: `feat/<name>`, `fix/<name>`, or `phase-<num>-<name>`.
2. **Review & Merge Protocol**:
   - Pull requests must be submitted for review.
   - Merges to `main` are performed exclusively by the repository maintainer.
   - Never force-push (`git push -f`) to shared branches.

---

## 🛠️ Verification & Test Rules

1. **Single Source of Truth**:
   - Algorithm implementations reside strictly in `simulink_matlab/lib/*.m` and `c_firmware/bms_ekf.c`.
   - Never paste custom MATLAB scripts into Simulink `.slx` blocks; models are generated programmatically via `build_bms_models.m`.
2. **Mandatory Regression Pass**:
   - Before submitting any change, run `simulink_matlab/run_bms_regression.m` in MATLAB.
   - All **33 automated checks** must pass (including model quality, back-to-back equivalence, and SIL verification).
3. **No Invented Numbers**:
   - Every metric reported in `README.md` or technical reports must be generated directly by an executable script (`tools/make_results_table.py`).
   - If a specific toolbox, target hardware, or dataset is unavailable, the feature must be labeled `NOT VERIFIED`.

---

## 💻 Code Style Guidelines

- **Python**: Follow PEP 8 guidelines. Format with `ruff` or `black`. Type hints are strongly encouraged.
- **MATLAB**: Use descriptive variable names, vectorized operations, and pre-allocated matrices.
- **Embedded C**: ANSI C99 (`-std=c99`), zero dynamic memory allocation (`malloc`/`free`), MISRA C clean, strictly bounded stack usage.
