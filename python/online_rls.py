"""
online_rls.py
=============
Online Recursive Least Squares (RLS) parameter identification for 2-RC Battery Model.
Estimates R0, R1, C1, R2, C2 in real time.
"""

import numpy as np


class OnlineRLS:
    """
    Recursive Least Squares (RLS) estimator for a 2nd-order RC battery network.
    Identifies:
        y(k) + c1*y(k-1) + c2*y(k-2) = d0*I(k) + d1*I(k-1) + d2*I(k-2)
    where:
        y(k) = V_ocv(SOC(k)) - V_t(k)  (polarization voltage drop)
    """

    def __init__(self, dt: float = 1.0, lmb: float = 0.995, p_init: float = 1e4):
        self.dt = dt
        self.lmb = lmb  # Forgetting factor

        # Parameters to identify: theta = [c1, c2, d0, d1, d2]^T
        self.theta = np.array([
            -1.95,      # c1 = -(a1 + a2)
             0.95,      # c2 = a1 * a2
             0.02,      # d0 = R0 + b1 + b2
            -0.038,     # d1
             0.018      # d2
        ], dtype=float)

        # Covariance matrix P (5x5)
        self.P = np.eye(5) * p_init

        # Data history: y_hist = [y(k-1), y(k-2)], I_hist = [I(k), I(k-1), I(k-2)]
        self.y_hist = np.zeros(2)
        self.I_hist = np.zeros(3)

        # Output physical parameters (starting with nominal estimates)
        self.R0 = 0.012
        self.R1 = 0.008
        self.C1 = 1500.0
        self.R2 = 0.015
        self.C2 = 12000.0

    def step(self, current: float, terminal_voltage: float, ocv: float) -> tuple:
        """
        Perform one step of RLS parameter estimation.
        current: Measured current [A] (positive = discharge)
        terminal_voltage: Measured terminal voltage [V]
        ocv: Open circuit voltage [V] estimated from EKF SOC
        """
        # 1. Compute polarization voltage drop (system output)
        y = ocv - terminal_voltage

        # We need at least history to run regression
        # phi = [-y(k-1), -y(k-2), I(k), I(k-1), I(k-2)]^T
        phi = np.array([
            -self.y_hist[0],
            -self.y_hist[1],
            current,
            self.I_hist[0],
            self.I_hist[1]
        ], dtype=float)

        # 2. RLS Gain & Covariance Update
        # denominator = lmb + phi^T * P * phi
        den = self.lmb + phi.T @ self.P @ phi
        if np.abs(den) > 1e-6:
            # K = P * phi / den
            K = (self.P @ phi) / den
            # error = y - phi^T * theta
            err = y - phi.T @ self.theta
            # Update theta
            self.theta += K * err
            # Update P = (P - K * phi^T * P) / lmb
            self.P = (self.P - np.outer(K, phi @ self.P)) / self.lmb

        # 3. Update data history
        self.y_hist[1] = self.y_hist[0]
        self.y_hist[0] = y
        self.I_hist[2] = self.I_hist[1]
        self.I_hist[1] = self.I_hist[0]
        self.I_hist[0] = current

        # 4. Map identified coefficients to physical parameters
        self._map_to_physical()

        return self.R0, self.R1, self.C1, self.R2, self.C2

    def _map_to_physical(self):
        """
        Solve algebraic mapping from theta = [c1, c2, d0, d1, d2] to R0, R1, C1, R2, C2.
        """
        c1, c2, d0, d1, d2 = self.theta

        # Enforce mathematical bounds to keep the systems stable and physical
        c2 = np.clip(c2, 0.01, 0.9999)
        disc = c1**2 - 4.0 * c2

        # If roots are complex, default back to nominal RC rates but update R0
        if disc < 0.0:
            a1 = 0.999
            a2 = 0.995
        else:
            roots = np.sqrt(disc)
            a1 = (-c1 + roots) / 2.0
            a2 = (-c1 - roots) / 2.0
            # Ensure roots are bounded inside (0, 1)
            a1 = np.clip(a1, 0.5, 0.9999)
            a2 = np.clip(a2, 0.5, 0.9999)

        # Avoid roots being identical to prevent division by zero
        if np.abs(a1 - a2) < 1e-4:
            a2 = a1 * 0.98

        # R0 mapping: d2 = R0 * a1 * a2 => R0 = d2 / (a1 * a2)
        R0 = d2 / (a1 * a2)
        # Enforce realistic resistance range
        R0 = np.clip(R0, 0.001, 0.150)

        # Solve system:
        # b1 + b2 = d0 - R0
        # a2*b1 + a1*b2 = -d1 - R0*(a1 + a2)
        y1 = d0 - R0
        y2 = -d1 + R0 * c1

        # Solving the system using Cramer's rule
        det = a1 - a2
        b1 = (a1 * y1 - y2) / det
        b2 = y1 - b1

        # Map polarization parameters R = b / (1 - a), C = -dt / (R * ln(a))
        R1 = b1 / (1.0 - a1)
        R2 = b2 / (1.0 - a2)

        # Clamp resistance values to keep EKF well-conditioned
        R1 = np.clip(R1, 0.001, 0.100)
        R2 = np.clip(R2, 0.001, 0.150)

        # Calculate capacities C
        ln_a1 = np.log(a1)
        ln_a2 = np.log(a2)
        C1 = -self.dt / (R1 * ln_a1) if ln_a1 != 0 else 1500.0
        C2 = -self.dt / (R2 * ln_a2) if ln_a2 != 0 else 12000.0

        # Cap capacitances
        C1 = np.clip(C1, 100.0, 10000.0)
        C2 = np.clip(C2, 1000.0, 50000.0)

        # Set variables
        self.R0 = R0
        self.R1 = R1
        self.C1 = C1
        self.R2 = R2
        self.C2 = C2
