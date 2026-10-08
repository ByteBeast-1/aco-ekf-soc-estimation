"""
visualization.py
================
Publication-quality plots for the 2-RC Thermal MO-ACO-EKF SOC estimation project.
"""

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

matplotlib.rcParams.update({
    "font.family":        "DejaVu Sans",
    "font.size":          11,
    "axes.titlesize":     13,
    "axes.labelsize":     11,
    "axes.spines.top":    False,
    "axes.spines.right":  False,
    "axes.grid":          True,
    "grid.linestyle":     "--",
    "grid.alpha":         0.4,
    "legend.framealpha":  0.9,
    "figure.dpi":         120,
})

C_REF   = "#2196F3"   # blue – reference SOC
C_EKF   = "#F44336"   # red – conventional EKF
C_ACO   = "#4CAF50"   # green – MO-ACO-EKF
C_BG    = "#F8F9FA"


def compute_metrics(soc_ref: np.ndarray, soc_est: np.ndarray) -> dict:
    """Compute MSE, MAE, MAPE, R² between reference and estimated SOC (both in [0, 1])."""
    err   = soc_est - soc_ref
    mse   = float(np.mean(err ** 2))
    mae   = float(np.mean(np.abs(err)))
    mask  = soc_ref > 0.01
    mape  = float(np.mean(np.abs(err[mask] / soc_ref[mask]))) * 100.0
    ss_res = np.sum(err ** 2)
    ss_tot = np.sum((soc_ref - soc_ref.mean()) ** 2)
    r2    = float(1.0 - (ss_res / (ss_tot + 1e-12)))
    return {"MSE": mse, "MAE": mae, "MAPE": mape, "R2": r2}


def plot_soc_comparison(results: list[dict], save_path: str | None = None) -> plt.Figure:
    n = len(results)
    fig, axes = plt.subplots(n, 1, figsize=(12, 3.2 * n), sharex=False)
    fig.patch.set_facecolor(C_BG)
    if n == 1:
        axes = [axes]

    for ax, res in zip(axes, results):
        ds = res["dataset"]
        t = ds.time / 60.0
        ax.set_facecolor(C_BG)
        ax.plot(t, ds.soc_ref * 100, color=C_REF, lw=2.0, label="Reference SOC")
        ax.plot(t, res["soc_conv"] * 100, color=C_EKF, lw=1.2, ls="--", label="Conv. EKF")
        ax.plot(t, res["soc_aco"] * 100, color=C_ACO, lw=2.0, label="2-RC MO-ACO-EKF (Ours)")
        ax.set_ylabel("SOC (%)")
        ax.set_title(f"SOC Tracking — {ds.name}", fontweight="bold")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (min)")
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
    return fig


def plot_soc_error(results: list[dict], save_path: str | None = None) -> plt.Figure:
    n = len(results)
    fig, axes = plt.subplots(n, 1, figsize=(12, 2.8 * n), sharex=False)
    fig.patch.set_facecolor(C_BG)
    if n == 1:
        axes = [axes]

    for ax, res in zip(axes, results):
        ds = res["dataset"]
        t = ds.time / 60.0
        err_conv = np.abs(ds.soc_ref - res["soc_conv"]) * 100
        err_aco  = np.abs(ds.soc_ref - res["soc_aco"]) * 100
        ax.set_facecolor(C_BG)
        ax.plot(t, err_conv, color=C_EKF, lw=1.0, alpha=0.7, label="Conv. EKF Error")
        ax.plot(t, err_aco, color=C_ACO, lw=1.8, label="2-RC MO-ACO-EKF Error")
        ax.set_ylabel("Absolute Error (%)")
        ax.set_title(f"SOC Estimation Error — {ds.name}", fontweight="bold")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (min)")
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
    return fig


def plot_aco_convergence(history: list[float], save_path: str | None = None) -> plt.Figure:
    fig, ax = plt.subplots(figsize=(8, 4.5))
    fig.patch.set_facecolor(C_BG)
    ax.set_facecolor(C_BG)
    ax.plot(range(1, len(history) + 1), history, color=C_ACO, marker="o", lw=2.0)
    ax.set_title("Multi-Objective ACOR Optimization Convergence", fontweight="bold")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("MSE Fitness Score")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
    return fig


def plot_metrics_table(results: list[dict], save_path: str | None = None) -> plt.Figure:
    names = [r["dataset"].name for r in results]
    mae_conv = [r["metrics_conv"]["MAE"] * 100 for r in results]
    mae_aco  = [r["metrics_aco"]["MAE"] * 100 for r in results]

    x = np.arange(len(names))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.patch.set_facecolor(C_BG)
    ax.set_facecolor(C_BG)

    rects1 = ax.bar(x - width/2, mae_conv, width, label="Conv. EKF", color=C_EKF, edgecolor="black")
    rects2 = ax.bar(x + width/2, mae_aco, width, label="2-RC MO-ACO-EKF", color=C_ACO, edgecolor="black")

    ax.set_ylabel("Mean Absolute Error (MAE %)")
    ax.set_title("MAE Error Comparison Across 4 Profiles", fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15)
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
    return fig


def plot_current_voltage(results: list[dict], save_path: str | None = None) -> plt.Figure:
    n = len(results)
    fig, axes = plt.subplots(n, 2, figsize=(14, 3.0 * n))
    fig.patch.set_facecolor(C_BG)

    for row, res in enumerate(results):
        ds = res["dataset"]
        t = ds.time / 60.0

        # Current
        ax_I = axes[row, 0]
        ax_I.set_facecolor(C_BG)
        ax_I.plot(t, ds.current_meas, color="#5C6BC0", lw=0.8, alpha=0.7, label="Measured current")
        ax_I.plot(t, ds.current_true, color="#1A237E", lw=1.2, label="True current")
        ax_I.set_ylabel("Current (A)")
        ax_I.set_title(f"{ds.name} - Current", fontweight="bold")
        ax_I.legend(fontsize=8)

        # Voltage
        ax_V = axes[row, 1]
        ax_V.set_facecolor(C_BG)
        ax_V.plot(t, ds.voltage_meas, color="#EF9A9A", lw=0.8, alpha=0.7, label="Measured voltage")
        ax_V.plot(t, ds.voltage_true, color="#B71C1C", lw=1.2, label="True voltage")
        ax_V.set_ylabel("Voltage (V)")
        ax_V.set_title(f"{ds.name} - Voltage", fontweight="bold")
        ax_V.legend(fontsize=8)

    for ax in axes[-1, :]:
        ax.set_xlabel("Time (min)")

    fig.suptitle("Battery Current & Voltage Profiles", fontsize=14, fontweight="bold")
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
        print(f"  [Plot] Saved -> {save_path}")

    return fig


def save_all_figures(results: list[dict], convergence: list[float], out_dir: str = "figures"):
    os.makedirs(out_dir, exist_ok=True)
    plot_current_voltage(results, save_path=os.path.join(out_dir, "01_current_voltage.png"))
    plot_soc_comparison(results, save_path=os.path.join(out_dir, "02_soc_comparison.png"))
    plot_soc_error(results, save_path=os.path.join(out_dir, "03_soc_error.png"))
    plot_aco_convergence(convergence, save_path=os.path.join(out_dir, "04_aco_convergence.png"))
    plot_metrics_table(results, save_path=os.path.join(out_dir, "05_metrics_table.png"))
    print(f"  [Plot] All figures 01-05 saved successfully to '{out_dir}/'!")
