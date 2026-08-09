"""
benchmark_suite.py
===================
Comprehensive 4-Way Algorithm Comparison Suite for Lithium-Ion Battery SOC Estimation:

Algorithms Evaluated:
  1. Coulomb Counting (Drifting integration baseline)
  2. Conventional EKF (Hand-tuned default Q and R)
  3. Unscented Kalman Filter - UKF (5 sigma points, Cholesky UT)
  4. ACO-EKF (Our proposed system - Ant Colony Optimized Q* and R*)

Evaluates tracking accuracy under initial SOC estimation error (+10% offset),
computes execution timings, and renders publication-ready comparative figures.
"""

import os
import time
import numpy as np
import matplotlib.pyplot as plt

from battery_model  import BatteryModel
from ekf            import ExtendedKalmanFilter
from ukf            import UnscentedKalmanFilter
from aco_optimizer  import ACOR
from data_generator import generate_all_profiles, BatteryDataset
from visualization  import compute_metrics, C_BG, C_REF, C_EKF, C_ACO


# ---------------------------------------------------------------------------
# Coulomb Counting Estimator Class (Simple Current Integration)
# ---------------------------------------------------------------------------
class CoulombCounting:
    """Coulomb Counting estimator (suffers from sensor bias drift)."""
    def __init__(self, model: BatteryModel):
        self.model = model

    def run(self, currents: np.ndarray, soc0: float) -> np.ndarray:
        N = len(currents)
        soc_est = np.zeros(N)
        soc = soc0
        dt  = self.model.dt
        Qn  = self.model.Qn
        eta = self.model.eta

        for k in range(N):
            soc = soc - (eta * currents[k] * dt) / Qn
            soc = float(np.clip(soc, 0.0, 1.0))
            soc_est[k] = soc
        return soc_est


# ---------------------------------------------------------------------------
# Main Benchmark Suite Runner
# ---------------------------------------------------------------------------
def run_benchmark_suite():
    print("=" * 75)
    print("  4-WAY ALGORITHM BENCHMARK SUITE FOR BATTERY SOC ESTIMATION")
    print("  Comparing: Coulomb Counting vs Conv. EKF vs UKF vs ACO-EKF")
    print("=" * 75)

    # 1. Initialize Battery Model
    model = BatteryModel()
    P0    = np.diag([0.10**2, 0.05**2])

    # 2. Generate Operating Profiles
    datasets = generate_all_profiles(model=model, seed=42)
    train_ds = datasets[:2]   # CC discharge & Pulse
    test_ds  = datasets[2:]   # Dynamic & CCCV

    # 3. Run ACO Optimization for ACO-EKF
    print("\n[ACO] Optimizing EKF Q and R covariances...")
    def objective(q_soc, q_vp, r_v):
        Q = np.diag([q_soc, q_vp])
        mses = []
        for ds in train_ds:
            soc_init = float(np.clip(ds.soc0 + 0.10 * np.sign(0.5 - ds.soc0), 0.02, 0.98))
            ekf = ExtendedKalmanFilter(model, Q=Q, R=r_v, P0=P0)
            soc_est = ekf.run(ds.current_meas, ds.voltage_meas, soc0=soc_init)
            mses.append(np.mean((soc_est - ds.soc_ref)**2))
        return float(np.mean(mses))

    aco = ACOR(objective=objective, max_iter=30, n_ants=8, verbose=False)
    aco_res = aco.run()
    q_soc_opt, q_vp_opt, r_v_opt = aco_res["best_params"]
    Q_opt = np.diag([q_soc_opt, q_vp_opt])
    R_opt = r_v_opt

    print(f"      ACO Optimal Q = diag([{q_soc_opt:.2e}, {q_vp_opt:.2e}]), R = {r_v_opt:.3e}")

    # Standard EKF default parameters
    Q_conv = np.diag([1e-5, 1e-5])
    R_conv = 1e-3

    # Standard UKF noise parameters
    Q_ukf  = np.diag([1e-5, 1e-5])
    R_ukf  = 1e-3

    # 4. Evaluate All Estimators Across Test Datasets
    print("\n[Benchmark] Running 4 Estimators across Test Datasets...\n")

    results_all = []

    for ds in datasets:
        soc_init = float(np.clip(ds.soc0 + 0.10 * np.sign(0.5 - ds.soc0), 0.02, 0.98))

        # 1. Coulomb Counting
        t0 = time.time()
        cc = CoulombCounting(model)
        soc_cc = cc.run(ds.current_meas, soc0=soc_init)
        t_cc = (time.time() - t0) * 1000

        # 2. Conventional EKF
        t0 = time.time()
        ekf_conv = ExtendedKalmanFilter(model, Q=Q_conv, R=R_conv, P0=P0)
        soc_conv = ekf_conv.run(ds.current_meas, ds.voltage_meas, soc0=soc_init)
        t_conv = (time.time() - t0) * 1000

        # 3. Unscented Kalman Filter (UKF)
        t0 = time.time()
        ukf = UnscentedKalmanFilter(model, Q=Q_ukf, R=R_ukf, P0=P0)
        soc_ukf = ukf.run(ds.current_meas, ds.voltage_meas, soc0=soc_init)
        t_ukf = (time.time() - t0) * 1000

        # 4. ACO-EKF (Proposed)
        t0 = time.time()
        ekf_aco = ExtendedKalmanFilter(model, Q=Q_opt, R=R_opt, P0=P0)
        soc_aco = ekf_aco.run(ds.current_meas, ds.voltage_meas, soc0=soc_init)
        t_aco = (time.time() - t0) * 1000

        results_all.append({
            "name":     ds.name,
            "time":     ds.time,
            "soc_ref":  ds.soc_ref,
            "soc_cc":   soc_cc,   "t_cc":   t_cc,
            "soc_conv": soc_conv, "t_conv": t_conv,
            "soc_ukf":  soc_ukf,  "t_ukf":  t_ukf,
            "soc_aco":  soc_aco,  "t_aco":  t_aco,
        })

    # 5. Print Comparison Table
    print("-" * 85)
    print(f"{'Profile':<24} {'Algorithm':<18} {'MSE':>10} {'MAE (%)':>10} {'MAPE (%)':>10} {'R²':>8} {'Time (ms)':>10}")
    print("-" * 85)

    for res in results_all:
        algos = [
            ("Coulomb Counting", res["soc_cc"], res["t_cc"]),
            ("Conventional EKF", res["soc_conv"], res["t_conv"]),
            ("UKF (Sigma Points)", res["soc_ukf"], res["t_ukf"]),
            ("ACO-EKF (Proposed)", res["soc_aco"], res["t_aco"]),
        ]
        for name, soc_est, exec_t in algos:
            m = compute_metrics(soc_est, res["soc_ref"])
            print(f"  {res['name']:<22} {name:<18} {m['MSE']:>10.6f} {m['MAE']:>10.4f} {m['MAPE']:>10.4f} {m['R2']:>8.4f} {exec_t:>10.2f}")
        print()

    print("-" * 85)

    # 6. Render Benchmark Figure
    render_benchmark_figure(results_all)


# ---------------------------------------------------------------------------
# Benchmark Visualization Function
# ---------------------------------------------------------------------------
def render_benchmark_figure(results_all: list[dict]):
    """Render 4-way comparison plots."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.patch.set_facecolor(C_BG)

    C_CC  = "#FF9800"  # Orange
    C_UKF = "#9C27B0"  # Purple

    for idx, res in enumerate(results_all):
        ax = axes[idx // 2, idx % 2]
        ax.set_facecolor(C_BG)
        t = res["time"] / 60.0

        ax.plot(t, res["soc_ref"] * 100, color=C_REF, lw=2.0, label="Reference SOC", zorder=4)
        ax.plot(t, res["soc_cc"]  * 100, color=C_CC,  lw=1.2, linestyle=":", label="Coulomb Counting", zorder=1)
        ax.plot(t, res["soc_conv"]* 100, color=C_EKF, lw=1.2, linestyle="--", label="Conv. EKF", zorder=2)
        ax.plot(t, res["soc_ukf"] * 100, color=C_UKF, lw=1.2, linestyle="-.", label="UKF", zorder=3)
        ax.plot(t, res["soc_aco"] * 100, color=C_ACO, lw=1.8, label="ACO-EKF (Proposed)", zorder=5)

        ax.set_ylabel("SOC (%)")
        ax.set_xlabel("Time (min)")
        ax.set_title(res["name"], fontweight="bold")
        ax.legend(fontsize=8, loc="best")
        ax.grid(True, linestyle="--", alpha=0.4)

    fig.suptitle("4-Way Benchmark Suite: Coulomb Counting vs Conv. EKF vs UKF vs ACO-EKF", fontsize=14, fontweight="bold")
    fig.tight_layout()

    os.makedirs("figures", exist_ok=True)
    fig_path = os.path.join("figures", "06_benchmark_suite.png")
    fig.savefig(fig_path, bbox_inches="tight")
    print(f"\n  [Plot] Benchmark Figure Saved → '{fig_path}'")


if __name__ == "__main__":
    run_benchmark_suite()
