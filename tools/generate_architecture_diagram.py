"""
generate_architecture_diagram.py
Generates a dark-themed, ultra-professional system architecture diagram matching the user's reference image style for the 2-RC Thermal MO-ACO-EKF project.
"""

import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def create_system_design_diagram(save_path="figures/00_system_architecture.png"):
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    # Dark theme matching user's image
    fig, ax = plt.subplots(figsize=(16, 9), facecolor="#0B0E14")
    ax.set_facecolor("#0B0E14")
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis("off")

    # Helper function to draw outlined box
    def draw_box(x, y, w, h, text, subtext="", bg_color="#131822", border_color="#38BDF8"):
        rect = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1,rounding_size=0.15",
                                     linewidth=1.8, edgecolor=border_color, facecolor=bg_color)
        ax.add_patch(rect)
        if subtext:
            ax.text(x + w/2, y + h/2 + 0.15, text, color="#FFFFFF", fontsize=11, fontweight="bold", ha="center", va="center")
            ax.text(x + w/2, y + h/2 - 0.2, subtext, color="#94A3B8", fontsize=8.5, ha="center", va="center")
        else:
            ax.text(x + w/2, y + h/2, text, color="#FFFFFF", fontsize=10.5, fontweight="bold", ha="center", va="center")

    # Helper function to draw arrow
    def draw_arrow(x1, y1, x2, y2, label=""):
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="-|>", color="#38BDF8", lw=1.8, mutation_scale=15))
        if label:
            mx, my = (x1 + x2)/2, (y1 + y2)/2
            ax.text(mx, my + 0.15, label, color="#94A3B8", fontsize=8, ha="center", va="center")

    # Title
    ax.text(8, 8.5, "SYSTEM ARCHITECTURE: 2-RC THERMAL MO-ACO-EKF BMS", color="#F8FAFC", fontsize=15, fontweight="bold", ha="center")

    # Left Inputs
    draw_box(0.5, 6.8, 2.2, 1.0, "INPUT", "Driving Profiles\n(UDDS, CC, Pulse)", border_color="#38BDF8")
    draw_box(0.5, 5.2, 2.2, 1.0, "THERMAL", "Cell Temp Dynamics\n(24°C → 39°C)", border_color="#38BDF8")
    draw_box(0.5, 3.6, 2.2, 1.0, "NASA DATASET", "Cell B0005 Hardware Logs", border_color="#38BDF8")

    # Multiplier / Summing Node (X)
    circle = patches.Circle((3.6, 5.7), 0.35, linewidth=1.8, edgecolor="#F43F5E", facecolor="#1E1018")
    ax.add_patch(circle)
    ax.text(3.6, 5.7, "✕", color="#F43F5E", fontsize=16, fontweight="bold", ha="center", va="center")

    # Arrows into Multiplier
    draw_arrow(2.7, 7.3, 3.6, 6.05)
    draw_arrow(2.7, 5.7, 3.25, 5.7)
    draw_arrow(2.7, 4.1, 3.6, 5.35)

    # Sensor Measurement Block
    draw_box(4.5, 5.2, 2.2, 1.0, "SENSOR LAYER", "Current, Voltage, Temp Sensors", border_color="#10B981")
    draw_arrow(3.95, 5.7, 4.5, 5.7)

    # Offline AI Layer (Top Branch)
    draw_box(4.5, 7.2, 2.2, 1.0, "MO-ACOR AI", "4D Log-Space Swarm Search", border_color="#F59E0B")
    draw_box(7.5, 7.2, 2.2, 1.0, "OPTIMAL MATRICES", "Q* & R* Covariances", border_color="#F59E0B")
    draw_arrow(6.7, 7.7, 7.5, 7.7)

    # Main Estimator Path
    draw_box(7.5, 5.2, 2.4, 1.0, "2-RC PHYSICAL TWIN", "Arrhenius Parameters R(T)\n3 States: [SOC, V1, V2]", border_color="#10B981")
    draw_arrow(6.7, 5.7, 7.5, 5.7)

    # Inject AI into EKF
    draw_arrow(8.6, 7.2, 8.6, 6.2, "Q*, R* Parameters")

    # EKF Core
    draw_box(10.7, 5.2, 2.4, 1.0, "3-STATE EKF CORE", "Joseph-Form Covariance Update\nP_k = (I-KH)P(I-KH)' + K R K'", border_color="#8B5CF6")
    draw_arrow(9.9, 5.7, 10.7, 5.7)

    # Output Block
    draw_box(14.0, 5.2, 1.7, 1.0, "ESTIMATED SOC", "Sub-0.01% MAE Accuracy", border_color="#EC4899")
    draw_arrow(13.1, 5.7, 14.0, 5.7)

    # Downward Feedback Branch
    draw_arrow(14.85, 5.2, 14.85, 3.2, "SOC (%) Signal")

    # Deployment Nodes (Bottom Path)
    draw_box(13.5, 2.2, 2.4, 1.0, "SIMULINK MBD", "advanced_bms_aco_ekf.slx", border_color="#3B82F6")
    draw_box(9.5, 2.2, 2.4, 1.0, "ANSI C FIRMWARE", "bms_ekf.c (< 2 KB RAM)", border_color="#3B82F6")
    draw_box(5.5, 2.2, 2.4, 1.0, "BMS FUEL GAUGE", "Real-Time EV Display", border_color="#3B82F6")

    # Connect bottom feedback path
    draw_arrow(13.5, 2.7, 11.9, 2.7)
    draw_arrow(9.5, 2.7, 7.9, 2.7)
    draw_arrow(5.5, 2.7, 1.6, 2.7)
    draw_arrow(1.6, 2.7, 1.6, 3.6, "FEEDBACK CORRECTION")

    # Legend / Info Box at bottom left
    fig.savefig(save_path, dpi=300, bbox_inches="tight", facecolor="#0B0E14")
    plt.close(fig)
    print(f"System architecture diagram generated successfully -> {save_path}")

if __name__ == "__main__":
    create_system_design_diagram()
