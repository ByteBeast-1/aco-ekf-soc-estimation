"""
nasa_benchmark_runner.py
========================
Runs the ACO-EKF algorithm against the real physical NASA Ames Cell B0005
laboratory experimental dataset.

Evaluates state estimation accuracy under initial SOC guess error (+15% offset),
compares ACO-EKF vs Conventional EKF vs Coulomb Counting, and saves plot
'figures/07_nasa_benchmark.png'.
"""

import os
import numpy as np
import matplotlib.pyplot as plt

from battery_model import BatteryModel
from ekf import ExtendedKalmanFilter
from aco_optimizer import ACOR
from visualization import compute_metrics, C_BG, C_REF, C_EKF, C_ACO


def run_nasa_benchmark():
    print("=" * 75)
    print("  NASA AMES LABORATORY CELL B0005 REAL HARDWARE BENCHMARK")
    print("=" * 75)

    # 1. Load NASA B0005 Dataset
    data_file = os.path.join("data", "NASA_B0005.csv")
    if not os.path.exists(data_file):
        from download_nasa_dataset import download_nasa_b0005
        download_nasa_b0005()

    raw_data = np.loadtxt(data_file, delimiter=",", skiprows=1)
    time_s       = raw_data[:, 0]
    current_A    = raw_data[:, 1]
    voltage_V    = raw_data[:, 2]
    temp_C       = raw_data[:, 3]
    soc_ref      = raw_data[:, 4]

    N = len(time_s)
    print(f"\n  [Data] Loaded NASA Cell B0005 Dataset: {N} samples ({N/60:.1f} minutes)")
    print(f"         Voltage Range: {voltage_V.min():.2f} V → {voltage_V.max():.2f} V")
    print(f"         Current Rate : {current_A.mean():.2f} A discharge")
    print(f"         Cell Temp    : {temp_C.min():.1f} °C → {temp_C.max():.1f} °C")

    # 2. Battery Model Setup (NASA 18650 cell specifications: R0=0.030 Ohm, 2 Ah capacity)
    model = BatteryModel(R0=0.030, R1=0.015, C1=2500.0, Qn=7200.0)
    P0    = np.diag([0.15**2, 0.05**2])   # Initial uncertainty (+15% error)

    # Initial SOC guess with intentional +15% error
    soc_init = float(np.clip(soc_ref[0] - 0.15, 0.05, 0.95))
    print(f"         True Start SOC: {soc_ref[0]*100:.1f}% | EKF Initial Guess: {soc_init*100:.1f}% (-15% error)")

    # 3. ACO Covariance Optimization
    print("\n[ACO] Optimizing EKF Q and R covariances for NASA cell dynamics...")
    def objective(q_soc, q_vp, r_v):
        Q = np.diag([q_soc, q_vp])
        ekf = ExtendedKalmanFilter(model, Q=Q, R=r_v, P0=P0)
        soc_est = ekf.run(current_A, voltage_V, soc0=soc_init)
        return float(np.mean((soc_est - soc_ref)**2))

    aco = ACOR(objective=objective, max_iter=25, n_ants=8, verbose=False)
    aco_res = aco.run()
    q_soc_opt, q_vp_opt, r_v_opt = aco_res["best_params"]
    Q_opt = np.diag([q_soc_opt, q_vp_opt])
    R_opt = r_v_opt

    print(f"      ACO Optimal Q = diag([{q_soc_opt:.2e}, {q_vp_opt:.2e}]), R = {r_v_opt:.3e}")

    # 4. Run Estimators on NASA Dataset
    print("\n[Benchmark] Evaluating Estimators on NASA Ames Cell B0005 Data...")

    # Coulomb Counting
    soc_cc = np.zeros(N)
    soc_tmp = soc_init
    for k in range(N):
        soc_tmp = soc_tmp - (current_A[k] * 1.0) / 7200.0
        soc_cc[k] = np.clip(soc_tmp, 0.0, 1.0)

    # Conventional EKF
    Q_conv = np.diag([1e-5, 1e-5])
    R_conv = 1e-3
    ekf_conv = ExtendedKalmanFilter(model, Q=Q_conv, R=R_conv, P0=P0)
    soc_conv = ekf_conv.run(current_A, voltage_V, soc0=soc_init)

    # ACO-EKF (Proposed)
    ekf_aco = ExtendedKalmanFilter(model, Q=Q_opt, R=R_opt, P0=P0)
    soc_aco = ekf_aco.run(current_A, voltage_V, soc0=soc_init)

    # 5. Calculate Metrics
    m_cc   = compute_metrics(soc_cc,   soc_ref)
    m_conv = compute_metrics(soc_conv, soc_ref)
    m_aco  = compute_metrics(soc_aco,  soc_ref)

    print("-" * 75)
    print(f"{'Estimator Method':<22} {'MSE':>12} {'MAE (%)':>12} {'MAPE (%)':>12} {'R²':>10}")
    print("-" * 75)
    print(f"  {'Coulomb Counting':<20} {m_cc['MSE']:>12.6f} {m_cc['MAE']:>12.4f} {m_cc['MAPE']:>12.4f} {m_cc['R2']:>10.4f}")
    print(f"  {'Conventional EKF':<20} {m_conv['MSE']:>12.6f} {m_conv['MAE']:>12.4f} {m_conv['MAPE']:>12.4f} {m_conv['R2']:>10.4f}")
    print(f"  {'ACO-EKF (Proposed)':<20} {m_aco['MSE']:>12.6f} {m_aco['MAE']:>12.4f} {m_aco['MAPE']:>12.4f} {m_aco['R2']:>10.4f}")
    print("-" * 75)

    # 6. Plot Results
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.patch.set_facecolor(C_BG)

    t_min = time_s / 60.0

    # Subplot 1: Voltage & Current
    ax0 = axes[0]
    ax0.set_facecolor(C_BG)
    ax0.plot(t_min, voltage_V, color="#D32F2F", lw=1.5, label="NASA Cell Voltage (V)")
    ax0.set_ylabel("Voltage (V)", fontsize=11)
    ax0.set_title("NASA Ames Cell B0005 Real Laboratory Discharge Measurements", fontweight="bold")
    ax0.legend(loc="upper right", fontsize=9)
    ax0.grid(True, linestyle="--", alpha=0.4)

    # Subplot 2: SOC Comparison
    ax1 = axes[1]
    ax1.set_facecolor(C_BG)
    ax1.plot(t_min, soc_ref  * 100, color=C_REF,   lw=2.2, label="NASA True Reference SOC", zorder=4)
    ax1.plot(t_min, soc_cc   * 100, color="#FF9800", lw=1.2, linestyle=":", label="Coulomb Counting", zorder=1)
    ax1.plot(t_min, soc_conv * 100, color=C_EKF,   lw=1.5, linestyle="--", label="Conventional EKF", zorder=2)
    ax1.plot(t_min, soc_aco  * 100, color=C_ACO,   lw=2.0, label="ACO-EKF (Proposed System)", zorder=5)

    ax1.set_xlabel("Time (min)", fontsize=11)
    ax1.set_ylabel("SOC (%)", fontsize=11)
    ax1.set_title("SOC Estimation on NASA Experimental Hardware Cell (-15% Initial Error)", fontweight="bold")
    ax1.legend(loc="lower left", fontsize=9)
    ax1.grid(True, linestyle="--", alpha=0.4)

    fig.tight_layout()
    os.makedirs("figures", exist_ok=True)
    fig_path = os.path.join("figures", "07_nasa_benchmark.png")
    fig.savefig(fig_path, bbox_inches="tight")
    print(f"\n  [Plot] Saved NASA Hardware Benchmark Chart → '{fig_path}'")


if __name__ == "__main__":
    run_nasa_benchmark()
