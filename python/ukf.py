"""
ukf.py
======
3-State Unscented Kalman Filter (UKF) for lithium-ion battery SOC estimation.

State vector:   x = [SOC, V1, V2]^T   (n = 3 states)
Number of sigma points: 2n + 1 = 7
"""

import numpy as np
from battery_model import BatteryModel


class UnscentedKalmanFilter:
    """
    3-State Unscented Kalman Filter for SOC estimation.
    """

    def __init__(
        self,
        model: BatteryModel,
        Q:     np.ndarray,
        R:     float,
        P0:    np.ndarray | None = None,
        alpha: float = 1e-3,
        beta:  float = 2.0,
        kappa: float = 0.0,
    ):
        self.model = model
        self.Q     = np.atleast_2d(Q).astype(float)
        self.R     = float(R)

        self.n = 3  # 3 state variables [SOC, V1, V2]
        self.alpha = alpha
        self.beta  = beta
        self.kappa = kappa

        # Lambda scaling factor
        self.lam = alpha**2 * (self.n + kappa) - self.n

        # Calculate weights for mean and covariance propagation
        self.wm = np.zeros(2 * self.n + 1)
        self.wc = np.zeros(2 * self.n + 1)

        self.wm[0] = self.lam / (self.n + self.lam)
        self.wc[0] = self.wm[0] + (1 - alpha**2 + beta)

        for i in range(1, 2 * self.n + 1):
            self.wm[i] = 1.0 / (2.0 * (self.n + self.lam))
            self.wc[i] = self.wm[i]

        # Initial state x0 = [0.95, 0.0, 0.0]
        self.x = np.array([0.95, 0.0, 0.0], dtype=float)

        # Initial error covariance P0
        if P0 is None:
            self.P = np.diag([0.01, 0.0025, 0.0025])
        else:
            self.P = np.atleast_2d(P0).astype(float)

    def _generate_sigma_points(self) -> np.ndarray:
        """Generate 7 sigma points using Cholesky matrix square root."""
        sigma = np.zeros((2 * self.n + 1, self.n))
        sigma[0] = self.x

        # Cholesky factorization of (n + lambda) * P
        P_scaled = (self.n + self.lam) * self.P
        # Ensure numerical positive-definiteness
        P_scaled = 0.5 * (P_scaled + P_scaled.T) + 1e-9 * np.eye(self.n)
        try:
            L = np.linalg.cholesky(P_scaled)
        except np.linalg.LinAlgError:
            eigval, eigvec = np.linalg.eigh(P_scaled)
            eigval = np.maximum(eigval, 1e-9)
            L = eigvec @ np.diag(np.sqrt(eigval))

        for i in range(self.n):
            sigma[i + 1]          = self.x + L[:, i]
            sigma[i + 1 + self.n] = self.x - L[:, i]

        return sigma

    def step(self, current: float, voltage_meas: float, temp_k: float = 298.15):
        """
        Execute one UKF step.
        """
        # 1. Generate 7 sigma points
        sigma = self._generate_sigma_points()

        # 2. Propagate sigma points through non-linear state equation
        sigma_pred = np.zeros_like(sigma)
        for i in range(2 * self.n + 1):
            sigma_pred[i] = self.model.step(sigma[i], current, temp_k)

        # 3. Compute predicted state mean
        x_pred = np.zeros(self.n)
        for i in range(2 * self.n + 1):
            x_pred += self.wm[i] * sigma_pred[i]

        # 4. Compute predicted state covariance
        P_pred = np.copy(self.Q)
        for i in range(2 * self.n + 1):
            diff = (sigma_pred[i] - x_pred).reshape(-1, 1)
            P_pred += self.wc[i] * (diff @ diff.T)

        # 5. Predict measurement for each sigma point
        z_sig = np.zeros(2 * self.n + 1)
        for i in range(2 * self.n + 1):
            z_sig[i] = self.model.terminal_voltage(sigma_pred[i], current, temp_k)

        # 6. Compute predicted measurement mean
        z_pred = float(np.sum(self.wm * z_sig))

        # 7. Compute measurement covariance S and cross-covariance Pxz
        S = float(self.R)
        Pxz = np.zeros((self.n, 1))

        for i in range(2 * self.n + 1):
            dz = z_sig[i] - z_pred
            S += float(self.wc[i] * (dz**2))
            dx = (sigma_pred[i] - x_pred).reshape(-1, 1)
            Pxz += self.wc[i] * dx * dz

        # 8. Compute Kalman gain
        K = Pxz / S  # shape (3, 1)

        # 9. State and covariance correction
        innovation = voltage_meas - z_pred
        self.x = x_pred + (K.flatten() * innovation)
        self.x[0] = np.clip(self.x[0], 0.0, 1.0)

        self.P = P_pred - S * (K @ K.T)
        self.P = 0.5 * (self.P + self.P.T)  # Ensure symmetry

        return self.x, self.P
