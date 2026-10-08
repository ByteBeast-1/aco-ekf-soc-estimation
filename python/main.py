"""
main.py
=======
Master Entry Point for 2-RC Thermal MO-ACO-EKF Lithium-Ion Battery SOC Estimation.

Workflow
--------
1. Generate four synthetic battery operating profiles with thermal gradients.
2. Run MO-ACOR to find optimal 3-state covariance matrices Q* (3x3) and R* (1x1).
3. Evaluate 2-RC MO-ACO-EKF on all 4 test profiles.
4. Compare against conventional EKF.
5. Print performance metrics table (MSE, MAE, MAPE, R^2).
6. Save publication-grade figures 01–05.
"""

import os
import time
import numpy as np

from battery_model  import BatteryModel
from ekf            import ExtendedKalmanFilter
from aco_optimizer  import ACOR
from data_generator import generate_all_profiles, BatteryDataset
from visualization  import save_all_figures, compute_metrics


def run_main():
    print("=" * 75)
    print("  2-RC THERMAL MO-ACO-EKF LITHIUM-ION BATTERY SOC ESTIMATION PIPELINE")
    print("=" * 75)

    model = BatteryModel()

    print("\n[Step 1/4] Generating 4 Battery Operating Profiles with Thermal Transients...")
    datasets = generate_all_profiles(model)

    print("\n[Step 2/4] Running Multi-Objective Continuous ACO (MO-ACOR) Optimization...")
    training_dataset = datasets[2]  # Dynamic UDDS drive cycle

    def objective_func(q_soc, q_v1, q_v2, r_v):
        Q_cand = np.diag([q_soc, q_v1, q_v2])
        x0_bad = [training_dataset.soc0 + 0.10, 0.0, 0.0]  # +10% initial error
        ekf_test = ExtendedKalmanFilter(model, Q=Q_cand, R=r_v, x0=x0_bad)
        N = len(training_dataset.time)
        soc_est = np.zeros(N)
        for k in range(N):
            x_upd, _ = ekf_test.step(training_dataset.current_meas[k], training_dataset.voltage_meas[k], training_dataset.temp_k[k])
            soc_est[k] = x_upd[0]
        mse = np.mean((training_dataset.soc_ref - soc_est)**2)
        conv_idx = np.argmax(np.abs(training_dataset.soc_ref - soc_est) < 0.01)
        conv_time = conv_idx * model.dt
        return mse + 0.001 * conv_time

    aco = ACOR(
        objective=objective_func,
        bounds=[(-8.0, -5.0), (-8.0, -5.0), (-8.0, -5.0), (-4.0, -2.0)],
        k_archive=20,
        n_ants=20,
        max_iter=15,
        seed=42
    )

    t0_aco = time.perf_counter()
    Q_opt, R_opt, aco_history = aco.optimize()
    t_aco_elapsed = time.perf_counter() - t0_aco
    print(f"  [MO-ACOR] Completed in {t_aco_elapsed:.2f} seconds!")

    # Step 3: Run Evaluation
    print("\n[Step 3/4] Evaluating 2-RC MO-ACO-EKF vs. Conventional EKF across Profiles...")
    Q_def = np.diag([1e-5, 1e-5, 1e-5])
    R_def = 1e-3

    results = []
    for ds in datasets:
        x0_bad = [ds.soc0 + 0.10, 0.0, 0.0]

        # Conv EKF
        ekf_conv = ExtendedKalmanFilter(model, Q=Q_def, R=R_def, x0=x0_bad)
        soc_conv = np.zeros(len(ds.time))
        for k in range(len(ds.time)):
            x_upd, _ = ekf_conv.step(ds.current_meas[k], ds.voltage_meas[k], ds.temp_k[k])
            soc_conv[k] = x_upd[0]

        # MO-ACO-EKF
        ekf_aco = ExtendedKalmanFilter(model, Q=Q_opt, R=R_opt, x0=x0_bad)
        soc_aco = np.zeros(len(ds.time))
        v_est_aco = np.zeros(len(ds.time))
        for k in range(len(ds.time)):
            x_upd, _ = ekf_aco.step(ds.current_meas[k], ds.voltage_meas[k], ds.temp_k[k])
            soc_aco[k] = x_upd[0]
            v_est_aco[k] = model.terminal_voltage(x_upd, ds.current_meas[k], ds.temp_k[k])

        m_conv = compute_metrics(ds.soc_ref, soc_conv)
        m_aco  = compute_metrics(ds.soc_ref, soc_aco)

        results.append({
            "dataset": ds,
            "soc_conv": soc_conv,
            "soc_aco": soc_aco,
            "v_est_aco": v_est_aco,
            "metrics_conv": m_conv,
            "metrics_aco": m_aco,
        })

    # Step 4: Summary Table
    print("\n[Step 4/4] Performance Summary Table Across Profiles:")
    print("=" * 85)
    print(f"{'Profile Name':<22} | {'Algorithm':<16} | {'MSE':<10} | {'MAE (%)':<9} | {'MAPE (%)':<9} | {'R^2':<8}")
    print("-" * 85)
    for res in results:
        name = res["dataset"].name
        mc   = res["metrics_conv"]
        ma   = res["metrics_aco"]
        print(f"{name:<22} | {'Conv. EKF':<16} | {mc['MSE']:<10.3e} | {mc['MAE']*100:<9.4f} | {mc['MAPE']:<9.4f} | {mc['R2']:<8.4f}")
        print(f"{'':<22} | {'2-RC MO-ACO-EKF':<16} | {ma['MSE']:<10.3e} | {ma['MAE']*100:<9.4f} | {ma['MAPE']:<9.4f} | {ma['R2']:<8.4f}")
        print("-" * 85)

    print("\n[Saving] Rendering and saving publication figures 01–05...")
    save_all_figures(results, aco_history)
    print("\n=================================================================")
    print("  Pipeline Execution Completed Successfully!")
    print("=================================================================")


if __name__ == "__main__":
    run_main()
