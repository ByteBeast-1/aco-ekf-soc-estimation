# ACO-EKF SOC Estimation for Lithium-Ion Batteries

A complete Python implementation of the **Ant Colony Optimized Extended Kalman Filter (ACO-EKF)** for lithium-ion battery State-of-Charge (SOC) estimation.

Based on the methodology of:
> Kannan M, Sundareswaran K, et al., *"Ant Colony Optimized Extended Kalman Filter for State of Charge Estimation of Lithium-Ion Battery"*, IEEE, 2025.

---

## Project Structure

```
aco-ekf-soc-estimation/
├── battery_model.py     # First-order RC equivalent-circuit model + OCV(SOC)
├── ekf.py               # Extended Kalman Filter recursion
├── aco_optimizer.py     # ACOR (ACO for continuous domains) — tunes Q and R
├── data_generator.py    # Synthetic battery profiles (4 operating points)
├── visualization.py     # All comparison plots and metrics charts
├── main.py              # Main pipeline — runs end-to-end
├── requirements.txt
└── figures/             # Output plots (created on first run)
```

---

## Algorithm Overview

```
Battery current & voltage measurements
        ↓
First-order RC battery model (Thevenin equivalent circuit)
        ↓
Extended Kalman Filter (EKF)
    • State prediction:   x̂_k|k-1 = f(x̂_k-1|k-1, Ik)
    • Cov prediction:     P_k|k-1  = F·P·Fᵀ + Q
    • Innovation:         rk       = Vt - h(x̂_k|k-1)
    • Kalman gain:        K        = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
    • State correction:   x̂_k|k   = x̂_k|k-1 + K·rk
        ↓
ACO (ACOR) outer optimisation loop
    • Tunes Q (process-noise covariance)
    • Tunes R (measurement-noise covariance)
    • Objective: minimise MSE(SOC_estimated, SOC_reference)
        ↓
Validated on independent test profiles
```

---

## Battery Model

Thevenin first-order RC equivalent circuit:

| Parameter | Symbol | Default |
|-----------|--------|---------|
| Series resistance | R₀ | 15 mΩ |
| Polarisation resistance | R₁ | 10 mΩ |
| Polarisation capacitance | C₁ | 3000 F |
| Nominal capacity | Qₙ | 2 Ah (7200 As) |
| Coulombic efficiency | η | 1.0 |
| Sampling interval | dt | 1 s |

OCV–SOC relationship modelled as an 8th-order polynomial.

---

## Operating Profiles

| # | Profile | Description |
|---|---------|-------------|
| 1 | CC Discharge 0.5C | Constant 0.5C discharge from 95% SOC |
| 2 | Pulse Discharge 1C | 30 s on / 30 s rest pulses |
| 3 | Dynamic Drive Cycle | UDDS-like stochastic current |
| 4 | CCCV Charge 0.5C | CC phase → CV taper charge from 20% SOC |

Profiles 1–2 are used for ACO training. Profiles 3–4 are held out for independent testing.

---

## Installation & Running

```bash
# Install dependencies
pip install -r requirements.txt

# Run the full pipeline
python main.py
```

Output figures will be saved to the `figures/` directory.

---

## Output Figures

| File | Content |
|------|---------|
| `01_current_voltage.png` | Measured current & voltage for all profiles |
| `02_soc_comparison.png` | SOC: Reference vs Conv. EKF vs ACO-EKF |
| `03_soc_error.png` | Absolute SOC error over time |
| `04_aco_convergence.png` | ACO best-objective vs iteration |
| `05_metrics_table.png` | MSE / MAE / MAPE / R² bar charts |

---

## Performance Metrics (Paper Reference)

| Metric | Reported (paper) |
|--------|-----------------|
| MSE    | 0.0165% |
| MAE    | 1.2% |
| MAPE   | 2% |
| R²     | 99.34% |

---

## Configuration

All hyperparameters are collected in the `CONFIG` dictionary at the top of `main.py`:

```python
CONFIG = {
    # ACO settings
    "aco_max_iter":  60,       # number of ACO iterations
    "aco_k_archive": 20,       # archive (elite) size
    "aco_n_ants":    10,       # ants per iteration

    # EKF default covariances (conventional EKF baseline)
    "q_soc_default": 1e-5,
    "q_vp_default":  1e-5,
    "r_v_default":   1e-3,

    # SOC initialisation error (to test convergence robustness)
    "soc_init_error": 0.10,    # 10% initial SOC error
    ...
}
```

---

## Key Concepts

- **EKF**: Estimates SOC by combining model prediction with voltage measurement correction.
- **Q matrix**: Process-noise covariance — how much we trust the battery model.
- **R scalar**: Measurement-noise variance — how much we trust the voltage sensor.
- **ACO**: Searches for Q and R values that minimise SOC estimation error.
- **ACOR**: ACO adapted for continuous search spaces using Gaussian kernels.
