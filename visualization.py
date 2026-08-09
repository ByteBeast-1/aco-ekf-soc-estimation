"""
visualization.py
================
Publication-quality plots for the ACO-EKF SOC estimation project.

Functions
---------
plot_soc_comparison     – SOC estimated vs reference (all profiles)
plot_soc_error          – Absolute SOC error over time
plot_aco_convergence    – ACO objective vs iteration
plot_metrics_table      – Metrics bar chart (MSE, MAE, MAPE, R²)
plot_current_voltage    – Battery current & voltage profiles
save_all_figures        – Render and save all figures to a folder
"""

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D

# ── Global style ──────────────────────────────────────────────────────────
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

# Colour palette
C_REF   = "#2196F3"   # blue  – reference SOC
C_EKF   = "#F44336"   # red   – conventional EKF
C_ACO   = "#4CAF50"   # green – ACO-EKF
C_CONV  = "#FF9800"   # orange – conventional EKF error
C_ACO_E = "#9C27B0"   # purple – ACO-EKF error
C_CONV_ = "#E91E63"
C_BG    = "#F8F9FA"


# ---------------------------------------------------------------------------
# Helper: compute performance metrics
# ---------------------------------------------------------------------------
def compute_metrics(soc_est: np.ndarray, soc_ref: np.ndarray) -> dict:
    """Compute MSE, MAE, MAPE, R² between estimated and reference SOC."""
    err   = soc_est - soc_ref
    mse   = float(np.mean(err ** 2))
    mae   = float(np.mean(np.abs(err)))
    # MAPE: avoid division by zero for near-zero SOC
    mask  = soc_ref > 0.01
    mape  = float(np.mean(np.abs(err[mask] / soc_ref[mask]))) * 100.0
    ss_res = np.sum(err ** 2)
    ss_tot = np.sum((soc_ref - soc_ref.mean()) ** 2)
    r2    = 1.0 - ss_res / (ss_tot + 1e-12)
    return {"MSE": mse, "MAE": mae * 100, "MAPE": mape, "R2": r2}


# ---------------------------------------------------------------------------
# 1. SOC comparison plot
# ---------------------------------------------------------------------------
def plot_soc_comparison(
    results:    list[dict],
    save_path:  str | None = None,
) -> plt.Figure:
    """
    Multi-panel SOC comparison: reference, conventional EKF, ACO-EKF.

    Parameters
    ----------
    results : list of dict, one per profile, each containing:
        'name'       : str
        'time'       : ndarray
        'soc_ref'    : ndarray
        'soc_conv'   : ndarray  (conventional EKF)
        'soc_aco'    : ndarray  (ACO-EKF)
    save_path : str, optional – file path to save the figure
    """
    n = len(results)
    fig, axes = plt.subplots(n, 1, figsize=(12, 3.5 * n), sharex=False)
    fig.patch.set_facecolor(C_BG)

    if n == 1:
        axes = [axes]

    for ax, res in zip(axes, results):
        t = res["time"] / 60.0   # seconds → minutes
        ax.set_facecolor(C_BG)

        ax.plot(t, res["soc_ref"]  * 100, color=C_REF, lw=2.0,
                label="Reference SOC", zorder=3)
        ax.plot(t, res["soc_conv"] * 100, color=C_EKF, lw=1.5,
                linestyle="--", label="Conventional EKF", zorder=2)
        ax.plot(t, res["soc_aco"]  * 100, color=C_ACO, lw=1.5,
                linestyle="-.", label="ACO-EKF", zorder=4)

        ax.set_ylabel("SOC (%)", fontsize=11)
        ax.set_title(res["name"], fontweight="bold")
        ax.legend(loc="best", fontsize=9)
        ax.set_ylim(
            max(0, (min(res["soc_ref"].min(), res["soc_aco"].min()) * 100) - 5),
            min(100, (max(res["soc_ref"].max(), res["soc_aco"].max()) * 100) + 5),
        )

    axes[-1].set_xlabel("Time (min)", fontsize=11)

    fig.suptitle(
        "SOC Estimation — ACO-EKF vs Conventional EKF",
        fontsize=15, fontweight="bold", y=1.01,
    )
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
        print(f"  [Plot] Saved → {save_path}")

    return fig


# ---------------------------------------------------------------------------
# 2. SOC error plot
# ---------------------------------------------------------------------------
def plot_soc_error(
    results:   list[dict],
    save_path: str | None = None,
) -> plt.Figure:
    """Absolute SOC error (%) over time for all profiles."""
    n = len(results)
    fig, axes = plt.subplots(n, 1, figsize=(12, 3.0 * n), sharex=False)
    fig.patch.set_facecolor(C_BG)

    if n == 1:
        axes = [axes]

    for ax, res in zip(axes, results):
        t        = res["time"] / 60.0
        err_conv = np.abs(res["soc_conv"] - res["soc_ref"]) * 100
        err_aco  = np.abs(res["soc_aco"]  - res["soc_ref"]) * 100

        ax.set_facecolor(C_BG)
        ax.fill_between(t, err_conv, alpha=0.25, color=C_EKF)
        ax.fill_between(t, err_aco,  alpha=0.30, color=C_ACO)
        ax.plot(t, err_conv, color=C_EKF, lw=1.2, label="Conventional EKF error")
        ax.plot(t, err_aco,  color=C_ACO, lw=1.2, label="ACO-EKF error")

        # Annotate mean errors
        ax.axhline(err_conv.mean(), color=C_EKF, lw=1, linestyle=":", alpha=0.7)
        ax.axhline(err_aco.mean(),  color=C_ACO, lw=1, linestyle=":", alpha=0.7)

        ax.set_ylabel("|Error| (%)", fontsize=11)
        ax.set_title(res["name"], fontweight="bold")
        ax.legend(loc="upper right", fontsize=9)

    axes[-1].set_xlabel("Time (min)", fontsize=11)
    fig.suptitle(
        "Absolute SOC Estimation Error",
        fontsize=15, fontweight="bold", y=1.01,
    )
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
        print(f"  [Plot] Saved → {save_path}")

    return fig


# ---------------------------------------------------------------------------
# 3. ACO convergence curve
# ---------------------------------------------------------------------------
def plot_aco_convergence(
    convergence:  list[float],
    save_path:    str | None = None,
) -> plt.Figure:
    """Plot ACO best-objective vs iteration."""
    fig, ax = plt.subplots(figsize=(10, 4.5))
    fig.patch.set_facecolor(C_BG)
    ax.set_facecolor(C_BG)

    iters = np.arange(len(convergence))
    ax.semilogy(iters, convergence, color=C_ACO, lw=2.0,
                marker="o", markersize=3, markevery=5, label="Best MSE")
    ax.fill_between(iters, convergence, alpha=0.15, color=C_ACO)

    ax.set_xlabel("ACO Iteration", fontsize=12)
    ax.set_ylabel("Best Objective (MSE)", fontsize=12)
    ax.set_title("ACO Convergence Curve", fontsize=14, fontweight="bold")
    ax.legend(fontsize=10)

    # Annotate final value
    ax.annotate(
        f"Final MSE = {convergence[-1]:.2e}",
        xy=(iters[-1], convergence[-1]),
        xytext=(iters[-1] * 0.6, convergence[-1] * 3),
        arrowprops=dict(arrowstyle="->", color="gray"),
        fontsize=10, color="#333333",
    )

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
        print(f"  [Plot] Saved → {save_path}")

    return fig


# ---------------------------------------------------------------------------
# 4. Metrics summary bar chart
# ---------------------------------------------------------------------------
def plot_metrics_table(
    results:   list[dict],
    save_path: str | None = None,
) -> plt.Figure:
    """Bar chart comparing metrics across all profiles for both estimators."""
    metric_keys  = ["MSE", "MAE", "MAPE"]
    metric_labels= ["MSE (x1e4)", "MAE (%)", "MAPE (%)"]
    metric_scale = [1e4, 1.0, 1.0]

    profile_names = [r["name"] for r in results]
    n_profiles    = len(results)
    n_metrics     = len(metric_keys)

    # Collect values
    conv_vals = {k: [] for k in metric_keys}
    aco_vals  = {k: [] for k in metric_keys}
    r2_conv   = []
    r2_aco    = []

    for res in results:
        m_conv = compute_metrics(res["soc_conv"], res["soc_ref"])
        m_aco  = compute_metrics(res["soc_aco"],  res["soc_ref"])
        for k in metric_keys:
            conv_vals[k].append(m_conv[k])
            aco_vals[k].append(m_aco[k])
        r2_conv.append(m_conv["R2"])
        r2_aco.append(m_aco["R2"])

    fig = plt.figure(figsize=(14, 9))
    fig.patch.set_facecolor(C_BG)
    gs  = gridspec.GridSpec(2, 2, hspace=0.45, wspace=0.35)
    axes_flat = [fig.add_subplot(gs[i // 2, i % 2]) for i in range(4)]

    x     = np.arange(n_profiles)
    width = 0.35

    for idx, (key, label, scale) in enumerate(zip(metric_keys, metric_labels, metric_scale)):
        ax = axes_flat[idx]
        ax.set_facecolor(C_BG)
        vals_c = [v * scale for v in conv_vals[key]]
        vals_a = [v * scale for v in aco_vals[key]]

        bars_c = ax.bar(x - width/2, vals_c, width, label="Conv. EKF",
                        color=C_EKF, alpha=0.85, edgecolor="white", linewidth=0.8)
        bars_a = ax.bar(x + width/2, vals_a, width, label="ACO-EKF",
                        color=C_ACO, alpha=0.85, edgecolor="white", linewidth=0.8)

        # Value labels on bars
        for bar in bars_c:
            h = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2, h * 1.02,
                    f"{h:.3f}", ha="center", va="bottom", fontsize=7.5, color=C_EKF)
        for bar in bars_a:
            h = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2, h * 1.02,
                    f"{h:.3f}", ha="center", va="bottom", fontsize=7.5, color="#2E7D32")

        ax.set_xticks(x)
        ax.set_xticklabels(
            [p.replace("_", "\n") for p in profile_names],
            fontsize=8,
        )
        ax.set_ylabel(label, fontsize=10)
        ax.set_title(label.split(" ")[0], fontweight="bold")
        ax.legend(fontsize=8)

    # R² panel
    ax_r2 = axes_flat[3]
    ax_r2.set_facecolor(C_BG)
    bars_c = ax_r2.bar(x - width/2, r2_conv, width, label="Conv. EKF",
                       color=C_EKF, alpha=0.85, edgecolor="white", linewidth=0.8)
    bars_a = ax_r2.bar(x + width/2, r2_aco,  width, label="ACO-EKF",
                       color=C_ACO, alpha=0.85, edgecolor="white", linewidth=0.8)
    ax_r2.set_ylim(
        max(0, min(min(r2_conv), min(r2_aco)) - 0.05),
        1.02,
    )
    for bar in bars_c:
        h = bar.get_height()
        ax_r2.text(bar.get_x() + bar.get_width()/2, h + 0.002,
                   f"{h:.4f}", ha="center", va="bottom", fontsize=7.5, color=C_EKF)
    for bar in bars_a:
        h = bar.get_height()
        ax_r2.text(bar.get_x() + bar.get_width()/2, h + 0.002,
                   f"{h:.4f}", ha="center", va="bottom", fontsize=7.5, color="#2E7D32")
    ax_r2.set_xticks(x)
    ax_r2.set_xticklabels(
        [p.replace("_", "\n") for p in profile_names], fontsize=8
    )
    ax_r2.set_ylabel("R²", fontsize=10)
    ax_r2.set_title("R²", fontweight="bold")
    ax_r2.legend(fontsize=8)

    fig.suptitle(
        "Performance Metrics — ACO-EKF vs Conventional EKF",
        fontsize=14, fontweight="bold",
    )

    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
        print(f"  [Plot] Saved → {save_path}")

    return fig


# ---------------------------------------------------------------------------
# 5. Current & voltage profiles overview
# ---------------------------------------------------------------------------
def plot_current_voltage(
    results:   list[dict],
    save_path: str | None = None,
) -> plt.Figure:
    """Four-panel overview of current and voltage for each profile."""
    n   = len(results)
    fig, axes = plt.subplots(n, 2, figsize=(14, 3.0 * n))
    fig.patch.set_facecolor(C_BG)

    for row, res in enumerate(results):
        t = res["time"] / 60.0

        # Current
        ax_I = axes[row, 0]
        ax_I.set_facecolor(C_BG)
        ax_I.plot(t, res["current_meas"], color="#5C6BC0", lw=0.8,
                  alpha=0.7, label="Measured current")
        ax_I.plot(t, res["current_true"], color="#1A237E", lw=1.2,
                  label="True current")
        ax_I.set_ylabel("Current (A)")
        ax_I.set_title(f"{res['name']} — Current", fontweight="bold")
        ax_I.legend(fontsize=8)

        # Voltage
        ax_V = axes[row, 1]
        ax_V.set_facecolor(C_BG)
        ax_V.plot(t, res["voltage_meas"], color="#EF9A9A", lw=0.8,
                  alpha=0.7, label="Measured voltage")
        ax_V.plot(t, res["voltage_true"], color="#B71C1C", lw=1.2,
                  label="True voltage")
        ax_V.set_ylabel("Voltage (V)")
        ax_V.set_title(f"{res['name']} — Voltage", fontweight="bold")
        ax_V.legend(fontsize=8)

    for ax in axes[-1, :]:
        ax.set_xlabel("Time (min)")

    fig.suptitle(
        "Battery Current & Voltage Profiles",
        fontsize=14, fontweight="bold",
    )
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, bbox_inches="tight")
        print(f"  [Plot] Saved → {save_path}")

    return fig


# ---------------------------------------------------------------------------
# 6. Save all figures
# ---------------------------------------------------------------------------
def save_all_figures(
    results:     list[dict],
    convergence: list[float],
    output_dir:  str = "figures",
):
    """Render and save all figures to output_dir."""
    os.makedirs(output_dir, exist_ok=True)
    print(f"\n  [Plot] Saving figures to '{output_dir}/'")

    plot_current_voltage(
        results, save_path=os.path.join(output_dir, "01_current_voltage.png")
    )
    plot_soc_comparison(
        results, save_path=os.path.join(output_dir, "02_soc_comparison.png")
    )
    plot_soc_error(
        results, save_path=os.path.join(output_dir, "03_soc_error.png")
    )
    plot_aco_convergence(
        convergence, save_path=os.path.join(output_dir, "04_aco_convergence.png")
    )
    plot_metrics_table(
        results, save_path=os.path.join(output_dir, "05_metrics_table.png")
    )

    print(f"  [Plot] All figures saved to '{output_dir}/'")
