"""
battery_model.py
================
Thermal-Adaptive Second-Order RC (T-2RC) equivalent-circuit model for a lithium-ion cell.

Model topology
--------------
                 R0(T)       R1(T)        R2(T)
  OCV(SOC) ──── /\/\ ──┬── /\/\ ──┬─── /\/\ ──┬── Vt (terminal)
                       |          |    |          |
                      C1         ---  C2         ---

State vector
    x = [SOC, V1, V2]^T
        SOC – State of charge  (0 … 1)
        V1  – Short-term transient voltage across R1‖C1  [V]  (Fast surface charge transfer)
        V2  – Long-term transient voltage across R2‖C2   [V]  (Slow concentration diffusion)

Input
    u = I   (current, positive = discharge)

Thermal Dependence
    Resistances R0, R1, R2 adapt dynamically with cell temperature T via the Arrhenius equation:
    R(T) = R_ref * exp( (Ea / Rg) * (1/T - 1/T_ref) )

Discrete-time state equations (3x3 State-Space, timestep dt)
    SOC(k+1) = SOC(k) − (eta * I * dt) / Qn
    V1(k+1)  = a1 * V1(k) + b1 * I
    V2(k+1)  = a2 * V2(k) + b2 * I
        a1 = exp(−dt / (R1(T)*C1)),  b1 = R1(T) * (1 − a1)
        a2 = exp(−dt / (R2(T)*C2)),  b2 = R2(T) * (1 − a2)

Measurement equation (nonlinear through 8th-order OCV polynomial)
    Vt(k) = OCV(SOC(k)) − V1(k) − V2(k) − R0(T) * I(k)

Jacobians (for 3-State EKF linearisation)
    F = ∂f/∂x  (3×3 state-transition Jacobian)
    H = ∂h/∂x  (1×3 measurement Jacobian: [dOCV/dSOC, -1, -1])
"""

import numpy as np


# ---------------------------------------------------------------------------
# OCV–SOC relationship (polynomial fit, 8th-order)
# ---------------------------------------------------------------------------
_OCV_POLY_COEFFS = np.array([
    -21.7497,   # SOC^8
     82.2625,   # SOC^7
    -125.3296,  # SOC^6
     99.6313,   # SOC^5
    -44.5010,   # SOC^4
     11.3416,   # SOC^3
     -1.5825,   # SOC^2
      1.5032,   # SOC^1
      3.0130,   # SOC^0
])


def ocv_from_soc(soc: np.ndarray | float) -> np.ndarray | float:
    """Compute open-circuit voltage from SOC using polynomial fit."""
    soc = np.clip(soc, 0.0, 1.0)
    return np.polyval(_OCV_POLY_COEFFS, soc)


def docv_dsoc(soc: np.ndarray | float) -> np.ndarray | float:
    """Derivative of OCV with respect to SOC (used in measurement Jacobian H)."""
    soc = np.clip(soc, 0.0, 1.0)
    deriv_coeffs = np.polyder(_OCV_POLY_COEFFS)
    return np.polyval(deriv_coeffs, soc)


# ---------------------------------------------------------------------------
# BatteryModel class (Thermal-Adaptive 2-RC)
# ---------------------------------------------------------------------------
class BatteryModel:
    """
    Thermal-Adaptive 2-RC Equivalent Circuit Model (T-2RC) for Li-ion cell.
    """

    def __init__(self,
                 Qn: float = 7200.0,     # Nominal capacity [As] (2.0 Ah)
                 R0_ref: float = 0.012,  # Ohmic resistance at 25°C [Ohm]
                 R1_ref: float = 0.008,  # Fast polarization resistance at 25°C [Ohm]
                 C1: float = 1500.0,     # Fast polarization capacitance [F]
                 R2_ref: float = 0.015,  # Slow polarization resistance at 25°C [Ohm]
                 C2: float = 12000.0,    # Slow polarization capacitance [F]
                 eta: float = 1.0,       # Coulombic efficiency
                 dt: float = 1.0,        # Sampling period [s]
                 Ea: float = 18000.0,    # Activation energy [J/mol]
                 T_ref: float = 298.15): # Reference temperature [K] (25°C)
        self.Qn = Qn
        self.R0_ref = R0_ref
        self.R1_ref = R1_ref
        self.C1 = C1
        self.R2_ref = R2_ref
        self.C2 = C2
        self.eta = eta
        self.dt = dt
        self.Ea = Ea
        self.T_ref = T_ref
        self.Rg = 8.314  # Universal gas constant [J/(mol K)]

    def get_thermal_params(self, temp_k: float = 298.15):
        """Calculate Arrhenius temperature-compensated resistances."""
        thermal_factor = np.exp((self.Ea / self.Rg) * (1.0 / temp_k - 1.0 / self.T_ref))
        R0 = self.R0_ref * thermal_factor
        R1 = self.R1_ref * thermal_factor
        R2 = self.R2_ref * thermal_factor
        return R0, R1, R2

    def state_transition_matrices(self, temp_k: float = 298.15):
        """
        Get discrete-time state-space system matrices (F, B) for 3-state system x = [SOC, V1, V2]^T.
        """
        R0, R1, R2 = self.get_thermal_params(temp_k)
        a1 = np.exp(-self.dt / (R1 * self.C1))
        b1 = R1 * (1.0 - a1)
        a2 = np.exp(-self.dt / (R2 * self.C2))
        b2 = R2 * (1.0 - a2)

        F = np.array([
            [1.0, 0.0, 0.0],
            [0.0, a1,  0.0],
            [0.0, 0.0, a2 ]
        ], dtype=float)

        B = np.array([
            [-(self.eta * self.dt) / self.Qn],
            [b1],
            [b2]
        ], dtype=float)

        return F, B

    def step(self, x: np.ndarray, current: float, temp_k: float = 298.15) -> np.ndarray:
        """
        Advance state x = [SOC, V1, V2]^T by one timestep dt.
        """
        x = np.asarray(x, dtype=float).flatten()
        F, B = self.state_transition_matrices(temp_k)
        x_next = F @ x + (B.flatten() * current)
        x_next[0] = np.clip(x_next[0], 0.0, 1.0)
        return x_next

    def terminal_voltage(self, x: np.ndarray, current: float, temp_k: float = 298.15) -> float:
        """
        Compute terminal voltage Vt = OCV(SOC) - V1 - V2 - R0(T)*I.
        """
        x = np.asarray(x, dtype=float).flatten()
        soc, v1, v2 = x[0], x[1], x[2]
        R0, _, _ = self.get_thermal_params(temp_k)
        ocv = ocv_from_soc(soc)
        vt = ocv - v1 - v2 - (R0 * current)
        return float(vt)

    def measurement_jacobian(self, x: np.ndarray) -> np.ndarray:
        """
        Compute 1x3 measurement Jacobian H = [dOCV/dSOC, -1, -1].
        """
        x = np.asarray(x, dtype=float).flatten()
        soc = x[0]
        dH_dSOC = docv_dsoc(soc)
        return np.array([[dH_dSOC, -1.0, -1.0]], dtype=float)
