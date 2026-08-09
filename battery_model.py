"""
battery_model.py
================
First-order RC (Thevenin) equivalent-circuit model for a lithium-ion cell.

Model topology
--------------
                 R0          R1
  OCV(SOC) ──── /\/\ ──┬── /\/\ ──┬── Vt  (terminal)
                       |           |
                      C1          ---

State vector
    x = [SOC, Vp]
        SOC  – state of charge  (0 … 1)
        Vp   – polarisation voltage across R1‖C1  [V]

Input
    u = I   (current, positive = discharge)

Output
    y = Vt  (terminal voltage [V])

Discrete-time state equations (Euler forward, timestep dt)
    SOC(k+1) = SOC(k) − (eta * I * dt) / Qn
    Vp(k+1)  = a * Vp(k) + b * I
        a = exp(−dt / (R1*C1))
        b = R1 * (1 − a)

Measurement equation (nonlinear through OCV)
    Vt(k) = OCV(SOC(k)) − Vp(k) − R0 * I(k)

Jacobians (for EKF linearisation)
    F = ∂f/∂x  (2×2 state-transition Jacobian)
    H = ∂h/∂x  (1×2 measurement Jacobian)

Default parameters are representative of a small 18650-type LFP/NMC cell
(2 Ah, 3.2–4.2 V range).  Replace with your identified values as needed.
"""

import numpy as np


# ---------------------------------------------------------------------------
# OCV–SOC relationship (polynomial fit, 8th-order)
# ---------------------------------------------------------------------------
# Coefficients for  OCV = sum_i  p[i] * SOC^(n-i)
# These approximate a typical NMC cell OCV curve.
# Generated from typical data; replace with your measured values.
_OCV_POLY_COEFFS = np.array([
    -21.7497,   # SOC^8
     82.2625,   # SOC^7
    -125.3296,  # SOC^6
     99.6313,   # SOC^5
    -44.5010,   # SOC^4
     11.3416,   # SOC^3
     -1.5825,   # SOC^2
      1.5032,   # SOC^1
      3.0130,   # SOC^0  (intercept, ≈ OCV at SOC=0)
])


def ocv_from_soc(soc: np.ndarray | float) -> np.ndarray | float:
    """
    Compute open-circuit voltage from SOC using a polynomial fit.

    Parameters
    ----------
    soc : float or array-like
        State of charge in [0, 1].

    Returns
    -------
    ocv : same shape as soc
        Open-circuit voltage [V].
    """
    soc = np.clip(soc, 0.0, 1.0)
    return np.polyval(_OCV_POLY_COEFFS, soc)


def docv_dsoc(soc: np.ndarray | float) -> np.ndarray | float:
    """
    Derivative of OCV with respect to SOC  (used in measurement Jacobian H).

    Parameters
    ----------
    soc : float or array-like

    Returns
    -------
    dOCV/dSOC
    """
    soc = np.clip(soc, 0.0, 1.0)
    # Derivative of polyval polynomial
    deriv_coeffs = np.polyder(_OCV_POLY_COEFFS)
    return np.polyval(deriv_coeffs, soc)


# ---------------------------------------------------------------------------
# BatteryModel class
# ---------------------------------------------------------------------------
class BatteryModel:
    """
    First-order RC equivalent-circuit model for a Li-ion cell.

    Parameters
    ----------
    R0  : float  – ohmic (series) resistance   [Ω]
    R1  : float  – polarisation resistance     [Ω]
    C1  : float  – polarisation capacitance    [F]
    Qn  : float  – nominal capacity            [A·s]  (1 Ah = 3600 As)
    eta : float  – Coulombic efficiency        [-]    (1.0 for discharge)
    dt  : float  – sampling interval           [s]
    """

    def __init__(
        self,
        R0: float  = 0.015,
        R1: float  = 0.010,
        C1: float  = 3000.0,
        Qn: float  = 7200.0,   # 2 Ah = 7200 As
        eta: float = 1.0,
        dt: float  = 1.0,
    ):
        self.R0  = R0
        self.R1  = R1
        self.C1  = C1
        self.Qn  = Qn
        self.eta = eta
        self.dt  = dt

        # Pre-compute discrete-time RC coefficients
        self._a = np.exp(-dt / (R1 * C1))
        self._b = R1 * (1.0 - self._a)

    # ------------------------------------------------------------------
    # State-transition function  f(x, u)
    # ------------------------------------------------------------------
    def state_transition(
        self,
        x: np.ndarray,
        I: float,
    ) -> np.ndarray:
        """
        Propagate state one time-step forward.

        Parameters
        ----------
        x : shape (2,)  – [SOC, Vp]
        I : float       – current (A), positive = discharge

        Returns
        -------
        x_next : shape (2,)
        """
        soc, vp = x[0], x[1]

        soc_next = soc - (self.eta * I * self.dt) / self.Qn
        vp_next  = self._a * vp + self._b * I

        soc_next = float(np.clip(soc_next, 0.0, 1.0))
        return np.array([soc_next, vp_next])

    # ------------------------------------------------------------------
    # Measurement function  h(x, u)
    # ------------------------------------------------------------------
    def terminal_voltage(
        self,
        x: np.ndarray,
        I: float,
    ) -> float:
        """
        Compute terminal voltage from state and current.

        Parameters
        ----------
        x : shape (2,)  – [SOC, Vp]
        I : float       – current (A)

        Returns
        -------
        Vt : float  – terminal voltage [V]
        """
        soc, vp = x[0], x[1]
        return float(ocv_from_soc(soc) - vp - self.R0 * I)

    # ------------------------------------------------------------------
    # Jacobian of state-transition  F = ∂f/∂x
    # ------------------------------------------------------------------
    def state_jacobian(self) -> np.ndarray:
        """
        Return the 2×2 state-transition Jacobian matrix.

        Because the SOC equation is linear in x and Vp equation is linear in x,
        F is constant (does not depend on the operating point).

        Returns
        -------
        F : ndarray, shape (2, 2)
        """
        F = np.array([
            [1.0,      0.0],   # ∂SOC_next/∂SOC, ∂SOC_next/∂Vp
            [0.0, self._a],    # ∂Vp_next /∂SOC, ∂Vp_next /∂Vp
        ])
        return F

    # ------------------------------------------------------------------
    # Jacobian of measurement  H = ∂h/∂x
    # ------------------------------------------------------------------
    def measurement_jacobian(self, soc: float) -> np.ndarray:
        """
        Return the 1×2 measurement Jacobian at the current SOC estimate.

        H = [dOCV/dSOC,  -1]

        Parameters
        ----------
        soc : float

        Returns
        -------
        H : ndarray, shape (1, 2)
        """
        H = np.array([[docv_dsoc(soc), -1.0]])
        return H

    # ------------------------------------------------------------------
    # Convenience: simulate noise-free trajectory
    # ------------------------------------------------------------------
    def simulate(
        self,
        current_profile: np.ndarray,
        soc0: float = 1.0,
        vp0: float  = 0.0,
    ) -> dict:
        """
        Run the battery model forward through a current profile.

        Parameters
        ----------
        current_profile : shape (N,)  – current at each timestep [A]
        soc0 : float  – initial SOC
        vp0  : float  – initial polarisation voltage

        Returns
        -------
        dict with keys 'soc', 'vp', 'voltage', 'time'
        """
        N   = len(current_profile)
        soc = np.zeros(N + 1)
        vp  = np.zeros(N + 1)
        vt  = np.zeros(N)

        soc[0] = soc0
        vp[0]  = vp0

        for k in range(N):
            x_k = np.array([soc[k], vp[k]])
            vt[k] = self.terminal_voltage(x_k, current_profile[k])
            x_next = self.state_transition(x_k, current_profile[k])
            soc[k + 1] = x_next[0]
            vp[k + 1]  = x_next[1]

        return {
            "soc":     soc[:-1],          # SOC at start of each step
            "vp":      vp[:-1],
            "voltage": vt,
            "time":    np.arange(N) * self.dt,
        }
