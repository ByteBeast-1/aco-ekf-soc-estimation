"""
generate_simulink_architecture_diagram.py
Generates a dark-themed system design diagram focused specifically on the Simulink MBD model (advanced_bms_aco_ekf.slx).
"""

import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def create_simulink_design_diagram(save_path="figures/00_simulink_system_design.png"):
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    fig, ax = plt.subplots(figsize=(16, 9), facecolor="#0B0E14")
    ax.set_facecolor("#0B0E14")
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis("off")

    def draw_box(x, y, w, h, text, subtext="", bg_color="#131822", border_color="#38BDF8"):
        rect = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1,rounding_size=0.15",
                                     linewidth=1.8, edgecolor=border_color, facecolor=bg_color)
        ax.add_patch(rect)
        if subtext:
            ax.text(x + w/2, y + h - 0.3, text, color="#FFFFFF", fontsize=10, fontweight="bold", ha="center", va="top")
            ax.text(x + w/2, y + 0.25, subtext, color="#94A3B8", fontsize=7.8, ha="center", va="bottom")
        else:
            ax.text(x + w/2, y + h/2, text, color="#FFFFFF", fontsize=9.5, fontweight="bold", ha="center", va="center")

    def draw_arrow(x1, y1, x2, y2, color="#38BDF8"):
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8, mutation_scale=14))

    # Title
    ax.text(8, 8.5, "SIMULINK MBD MODEL SYSTEM DESIGN (advanced_bms_aco_ekf.slx)", color="#F8FAFC", fontsize=14, fontweight="bold", ha="center")

    # Input Block
    draw_box(0.5, 4.2, 2.3, 1.4, "Drive_Cycle_Current", "Pulse Generator\nI_load = 2.5 A, dt = 1.0 s", border_color="#38BDF8")

    # Subsystem 1: Battery Physics Plant
    draw_box(3.5, 3.2, 3.4, 3.2, "Battery_Physics_Plant\n(Subsystem 1)", "• Arrhenius R(T) Thermal Model\n• 3 States: [SOC, V1, V2]\n• 8th-Order OCV Polynomial\n• Sensor Noise (+/-8mV)", border_color="#10B981")

    # Subsystem 2: ACO-EKF Controller
    draw_box(7.8, 3.2, 3.4, 3.2, "ACO_EKF_Controller\n(Subsystem 2)", "• ACO Parameters Q* & R*\n• 3-State EKF Predict-Correct\n• Measurement Jacobian H\n• Joseph-Form Covariance Update", border_color="#8B5CF6")

    # Signals between inputs and subsystems
    draw_arrow(2.8, 4.9, 3.5, 4.9)
    draw_arrow(2.8, 4.9, 7.8, 5.8)
    draw_arrow(6.9, 4.2, 7.8, 4.2)

    # Mux Blocks & Math Subtraction
    draw_box(12.0, 6.8, 1.6, 0.9, "Mux_SOC", "Combines True & Est", border_color="#F59E0B")
    draw_box(12.0, 4.8, 1.6, 0.9, "Mux_Voltage", "Combines 3 Volt Signals", border_color="#F59E0B")
    draw_box(12.0, 2.8, 1.6, 0.9, "Subtract_Error", "Est_SOC - True_SOC", border_color="#F43F5E")

    # Signal Routing to Mux / Math
    draw_arrow(6.9, 6.0, 12.0, 7.3, color="#10B981")
    draw_arrow(11.2, 6.0, 12.0, 7.0, color="#8B5CF6")

    draw_arrow(6.9, 5.0, 12.0, 5.4, color="#10B981")
    draw_arrow(6.9, 4.2, 12.0, 5.1, color="#38BDF8")
    draw_arrow(11.2, 5.0, 12.0, 4.9, color="#8B5CF6")

    draw_arrow(11.2, 6.0, 12.0, 3.4, color="#8B5CF6")
    draw_arrow(6.9, 3.5, 12.0, 3.0, color="#10B981")

    # Oscilloscopes (Scopes)
    draw_box(14.1, 6.8, 1.6, 0.9, "SOC_Comparison\nScope", "True vs Est SOC", border_color="#EC4899")
    draw_box(14.1, 4.8, 1.6, 0.9, "Voltage_Scope", "3 Voltage Lines", border_color="#EC4899")
    draw_box(14.1, 2.8, 1.6, 0.9, "SOC_Error_Scope", "Error < 0.10%", border_color="#EC4899")

    draw_arrow(13.6, 7.25, 14.1, 7.25)
    draw_arrow(13.6, 5.25, 14.1, 5.25)
    draw_arrow(13.6, 3.25, 14.1, 3.25)

    # Solver Box at bottom
    draw_box(3.5, 1.0, 7.7, 1.0, "SIMULINK SOLVER CONFIGURATION", "Solver: FixedStepDiscrete | Timestep (dt): 1.0 s | Stop Time: 3600 s (1 Hour)", border_color="#3B82F6")

    fig.savefig(save_path, dpi=300, bbox_inches="tight", facecolor="#0B0E14")
    plt.close(fig)
    print(f"Simulink system design diagram generated successfully -> {save_path}")

if __name__ == "__main__":
    create_simulink_design_diagram()
