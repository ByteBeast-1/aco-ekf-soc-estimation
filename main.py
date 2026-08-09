"""
main.py
=======
Entry point for the ACO-EKF lithium-ion battery SOC estimation project.

Workflow
--------
1.  Generate four synthetic battery operating profiles.
2.  Split into training (ACO tuning) and testing (independent evaluation).
3.  Run ACOR to find optimal EKF covariance matrices Q* and R*.
4.  Evaluate ACO-EKF on the test profiles.
5.  Evaluate conventional EKF (with default Q/R) on the same profiles.
6.  Print a metrics summary table.
7.  Generate and save all comparison figures.

Usage
-----
    python main.py

Optional arguments can be set via the CONFIG dict below.
"""

import os
import time
import numpy as np

from battery_model  import BatteryModel
from ekf            import ExtendedKalmanFilter
from aco_optimizer  import ACOR
from data_generator import generate_all_profiles, BatteryDataset
from visualization  import (
    save_all_figures,
    compute_metrics,
)


# ════════════════════════════════════════════════════════════════════════════
# Configuration
# ════════════════════════════════════════════════════════════════════════════
CONFIG = {
    # Battery model parameters (first-order RC)
    "R0":  0.015,      # ohmic resistance [Ω]
    "R1":  0.010,      # polarisation resistance [Ω]
    "C1":  3000.0,     # polarisation capacitance [F]
    "Qn":  7200.0,     # nominal capacity [A·s] (2 Ah)
    "eta": 1.0,        # Coulombic efficiency
    "dt":  1.0,        # sampling interval [s]

    # ACO hyperparameters
    "aco_k_archive":  20,     # archive size
    "aco_n_ants":     8,      # ants per iteration
    "aco_max_iter":   30,     # max iterations
    "aco_q":          0.1,    # diversity factor
    "aco_xi":         0.85,   # Gaussian spread factor
    "aco_seed":       42,

    # ACO search bounds  (log10 scale)
    #   [log10(q_SOC_min/max), log10(q_Vp_min/max), log10(r_V_min/max)]
    "aco_bounds": [(-7, -2), (-7, -2), (-5, -1)],

    # Conventional EKF default covariances
    "q_soc_default":  1e-5,
    "q_vp_default":   1e-5,
    "r_v_default":    1e-3,

    # EKF initial uncertainty
    "P0_soc": 0.10 ** 2,   # SOC initial variance
    "P0_vp":  0.05 ** 2,   # Vp initial variance

    # Which profiles to use for ACO training (0-indexed)
    # remaining profiles are used for testing
    "train_profile_idx": [0, 1],   # CC-discharge and Pulse for training
    "test_profile_idx":  [2, 3],   # Dynamic and CCCV for testing

    # Intentional initial SOC error for testing robustness
    "soc_init_error":  0.10,   # start EKF 10% away from true SOC

    # Output
    "output_dir": "figures",
    "data_seed":  0,
}


# ════════════════════════════════════════════════════════════════════════════
# Helper functions
# ════════════════════════════════════════════════════════════════════════════

def _make_Q(q_soc: float, q_vp: float) -> np.ndarray:
    """Build 2×2 diagonal process-noise covariance."""
    return np.diag([q_soc, q_vp])


def _make_P0(cfg: dict) -> np.ndarray:
    """Build initial state covariance."""
    return np.diag([cfg["P0_soc"], cfg["P0_vp"]])


def _run_ekf_on_dataset(
    ds:    BatteryDataset,
    model: BatteryModel,
    Q:     np.ndarray,
    R:     float,
    P0:    np.ndarray,
    soc0:  float | None = None,
) -> np.ndarray:
    """
    Run the EKF on one dataset and return SOC estimates.

    Parameters
    ----------
    soc0 : float | None
        Starting SOC estimate.  If None, uses the true ds.soc0.
    """
    if soc0 is None:
        soc0 = ds.soc0

    ekf = ExtendedKalmanFilter(model, Q=Q, R=R, P0=P0)
    return ekf.run(ds.current_meas, ds.voltage_meas, soc0=soc0)


def _objective_factory(
    model:    BatteryModel,
    datasets: list[BatteryDataset],
    P0:       np.ndarray,
    cfg:      dict,
):
    """
    Return an ACO objective function that runs the EKF on training datasets.

    The objective is the mean MSE across all training profiles.
    """
    def objective(q_soc: float, q_vp: float, r_v: float) -> float:
        Q     = _make_Q(q_soc, q_vp)
        mse_list = []
        for ds in datasets:
            # Apply intentional initial SOC error to training too
            soc_init = float(np.clip(
                ds.soc0 + cfg["soc_init_error"] * np.sign(0.5 - ds.soc0),
                0.02, 0.98,
            ))
            soc_est = _run_ekf_on_dataset(ds, model, Q, r_v, P0, soc0=soc_init)
            mse     = float(np.mean((soc_est - ds.soc_ref) ** 2))
            mse_list.append(mse)
        return float(np.mean(mse_list))

    return objective


# ════════════════════════════════════════════════════════════════════════════
# Main pipeline
# ════════════════════════════════════════════════════════════════════════════

def main():
    cfg = CONFIG

    print("=" * 65)
    print("  ACO-EKF Battery SOC Estimation")
    print("=" * 65)

    # ── Step 1: Battery model ──────────────────────────────────────────
    print("\n[1/6] Creating battery model …")
    model = BatteryModel(
        R0  = cfg["R0"],
        R1  = cfg["R1"],
        C1  = cfg["C1"],
        Qn  = cfg["Qn"],
        eta = cfg["eta"],
        dt  = cfg["dt"],
    )
    P0 = _make_P0(cfg)
    print(f"      R0={cfg['R0']} Ohm  R1={cfg['R1']} Ohm  "
          f"C1={cfg['C1']} F  Qn={cfg['Qn']/3600:.1f} Ah")

    # ── Step 2: Generate data ──────────────────────────────────────────
    print("\n[2/6] Generating battery operating profiles …")
    all_datasets = generate_all_profiles(model=model, seed=cfg["data_seed"])

    train_ds = [all_datasets[i] for i in cfg["train_profile_idx"]]
    test_ds  = [all_datasets[i] for i in cfg["test_profile_idx"]]

    print(f"      Training profiles : {[d.name for d in train_ds]}")
    print(f"      Testing  profiles : {[d.name for d in test_ds]}")

    # ── Step 3: ACO optimisation ───────────────────────────────────────
    print("\n[3/6] Running ACO to tune EKF covariances (Q, R) …")
    objective = _objective_factory(model, train_ds, P0, cfg)

    t0  = time.time()
    aco = ACOR(
        objective  = objective,
        bounds     = cfg["aco_bounds"],
        k_archive  = cfg["aco_k_archive"],
        n_ants     = cfg["aco_n_ants"],
        max_iter   = cfg["aco_max_iter"],
        q_acor     = cfg["aco_q"],
        xi         = cfg["aco_xi"],
        seed       = cfg["aco_seed"],
        verbose    = True,
    )
    aco_result = aco.run()
    elapsed    = time.time() - t0

    q_soc_opt, q_vp_opt, r_v_opt = aco_result["best_params"]
    Q_opt = _make_Q(q_soc_opt, q_vp_opt)

    print(f"\n      ACO finished in {elapsed:.1f} s")
    print(f"      Optimal Q = diag([{q_soc_opt:.3e}, {q_vp_opt:.3e}])")
    print(f"      Optimal R = {r_v_opt:.3e}")
    print(f"      Best training MSE = {aco_result['best_objective']:.6f}")

    # ── Step 4: Default (conventional) EKF covariances ────────────────
    Q_conv = _make_Q(cfg["q_soc_default"], cfg["q_vp_default"])
    R_conv = cfg["r_v_default"]

    print(f"\n      Conventional EKF Q = diag([{cfg['q_soc_default']:.1e},"
          f" {cfg['q_vp_default']:.1e}])  R = {cfg['r_v_default']:.1e}")

    # ── Step 5: Evaluate on ALL four profiles ──────────────────────────
    print("\n[4/6] Evaluating ACO-EKF and Conventional EKF on test profiles …")

    # Collect results for all profiles (train + test) for visualization
    results = []
    for ds in all_datasets:
        soc_init = float(np.clip(
            ds.soc0 + cfg["soc_init_error"] * np.sign(0.5 - ds.soc0),
            0.02, 0.98,
        ))

        soc_aco  = _run_ekf_on_dataset(ds, model, Q_opt,  r_v_opt, P0, soc_init)
        soc_conv = _run_ekf_on_dataset(ds, model, Q_conv, R_conv,  P0, soc_init)

        results.append({
            "name":         ds.name,
            "time":         ds.time,
            "soc_ref":      ds.soc_ref,
            "soc_aco":      soc_aco,
            "soc_conv":     soc_conv,
            "current_true": ds.current_true,
            "current_meas": ds.current_meas,
            "voltage_true": ds.voltage_true,
            "voltage_meas": ds.voltage_meas,
        })

    # ── Step 6: Print metrics ──────────────────────────────────────────
    print("\n[5/6] Performance metrics")
    print("-" * 75)
    header = (
        f"{'Profile':<28} {'Method':<16} "
        f"{'MSE':>10} {'MAE (%)':>10} {'MAPE (%)':>10} {'R²':>8}"
    )
    print(header)
    print("-" * 75)

    for res in results:
        for label, key in [("Conv. EKF", "soc_conv"), ("ACO-EKF", "soc_aco")]:
            m = compute_metrics(res[key], res["soc_ref"])
            print(
                f"  {res['name']:<26} {label:<16} "
                f"{m['MSE']:>10.6f} {m['MAE']:>10.4f} "
                f"{m['MAPE']:>10.4f} {m['R2']:>8.4f}"
            )
        print()

    print("-" * 75)

    # ── Step 7: Save figures ───────────────────────────────────────────
    print(f"\n[6/6] Generating figures → '{cfg['output_dir']}/' …")
    save_all_figures(
        results     = results,
        convergence = aco_result["convergence"],
        output_dir  = cfg["output_dir"],
    )

    print("\n" + "=" * 65)
    print("  Done!  Open the 'figures/' folder to view all plots.")
    print("=" * 65)


if __name__ == "__main__":
    main()
