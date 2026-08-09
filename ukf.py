"""
ukf.py
======
Unscented Kalman Filter (UKF) for lithium-ion battery SOC estimation.

The UKF uses the Unscented Transform (UT) to propagate state mean and covariance
through nonlinear equations using a set of deterministically chosen sigma points.

State vector:   x = [SOC, Vp]   (n = 2 states)
Number of sigma points: 2n + 1 = 5

Parameters of Unscented Transform:
    alpha = 1e-3  (spread of sigma points)
    beta  = 2.0   (prior knowledge of distribution, 2 for Gaussian)
    kappa = 0.0   (secondary scaling parameter)
    lambda = alpha^2 * (n + kappa) - n
"""

import numpy as np
from battery_model import BatteryModel


class UnscentedKalmanFilter:
    """
    Unscented Kalman Filter for SOC estimation.

    Parameters
    ----------
    model : BatteryModel
        Battery physics model.
    Q : ndarray, shape (2, 2)
        Process noise covariance.
    R : float
        Measurement noise variance.
    P0 : ndarray, shape (2, 2), optional
        Initial error covariance.
    alpha, beta, kappa : float
        Unscented transform scaling parameters.
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

        self.n = 2  # state dimension
        self.alpha = alpha
        self.beta  = beta
        self.kappa = kappa

        # Lambda scaling factor
        self.lam = alpha**2 * (self.n + kappa) - self.n
        self.gamma = np.sqrt(self.n + self.lam)

        # Compute weights for mean and covariance
        n_sigma = 2 * self.n + 1
        self.Wm = np.zeros(n_sigma)
        self.Wc = np.zeros(n_sigma)

        self.Wm[0] = self.lam / (self.n + self.lam)
        self.Wc[0] = self.Wm[0] + (1 - alpha**2 + beta)

        for i in range(1, n_sigma):
            self.Wm[i] = 1.0 / (2 * (self.n + self.lam))
            self.Wc[i] = self.Wm[i]

        if P0 is None:
            self.P0 = np.diag([0.1**2, 0.05**2])
        else:
            self.P0 = np.atleast_2d(P0).astype(float)

        self._x_hat = None
        self._P     = None

    def reset(self, soc0: float = 1.0, vp0: float = 0.0):
        """Initialize UKF state and covariance."""
        self._x_hat = np.array([soc0, vp0], dtype=float)
        self._P     = self.P0.copy()

    def _generate_sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """Generate 2n + 1 sigma points around state mean x."""
        n = self.n
        sigma = np.zeros((2 * n + 1, n))
        sigma[0] = x

        # Matrix square root using Cholesky decomposition
        try:
            L = np.linalg.cholesky(P)
        except np.linalg.LinAlgError:
            # Fallback to SVD / eig for non-positive definite P
            eigval, eigvec = np.linalg.eigh(P)
            eigval = np.maximum(eigval, 1e-9)
            L = eigvec @ np.diag(np.sqrt(eigval))

        for i in range(n):
            sigma[i + 1]     = x + self.gamma * L[:, i]
            sigma[n + i + 1] = x - self.gamma * L[:, i]

        return sigma

    def step(self, I: float, Vt_measured: float) -> float:
        """Run one UKF prediction-correction cycle."""
        if self._x_hat is None:
            raise RuntimeError("Call reset() before step().")

        model = self.model
        n_sigma = 2 * self.n + 1

        # ── 1. Generate Sigma Points ──────────────────────────────────
        sigmas = self._generate_sigma_points(self._x_hat, self._P)

        # ── 2. State Prediction (Propagate through f) ────────────────
        sigmas_f = np.zeros_like(sigmas)
        for i in range(n_sigma):
            sigmas_f[i] = model.state_transition(sigmas[i], I)

        # Predicted state mean
        x_pred = np.zeros(self.n)
        for i in range(n_sigma):
            x_pred += self.Wm[i] * sigmas_f[i]

        # Predicted state covariance
        P_pred = self.Q.copy()
        for i in range(n_sigma):
            diff = (sigmas_f[i] - x_pred).reshape(-1, 1)
            P_pred += self.Wc[i] * (diff @ diff.T)

        # ── 3. Measurement Prediction (Propagate through h) ──────────
        sigmas_h = np.zeros(n_sigma)
        for i in range(n_sigma):
            sigmas_h[i] = model.terminal_voltage(sigmas_f[i], I)

        # Predicted measurement mean
        y_pred = float(np.sum(self.Wm * sigmas_h))

        # ── 4. Innovation & Cross Covariances ─────────────────────────
        Pyy = float(self.R)
        Pxy = np.zeros((self.n, 1))

        for i in range(n_sigma):
            y_diff = sigmas_h[i] - y_pred
            x_diff = (sigmas_f[i] - x_pred).reshape(-1, 1)

            Pyy += self.Wc[i] * (y_diff**2)
            Pxy += self.Wc[i] * (x_diff * y_diff)

        # ── 5. Kalman Gain & State Correction ────────────────────────
        K = Pxy / Pyy
        innovation = Vt_measured - y_pred

        x_upd = x_pred + (K.flatten() * innovation)
        x_upd[0] = float(np.clip(x_upd[0], 0.0, 1.0))

        P_upd = P_pred - K @ (Pyy * K.T)

        self._x_hat = x_upd
        self._P     = P_upd

        return float(x_upd[0])

    def run(
        self,
        currents: np.ndarray,
        voltages: np.ndarray,
        soc0: float = 1.0,
        vp0: float = 0.0,
    ) -> np.ndarray:
        """Run UKF batch over dataset."""
        N = len(currents)
        self.reset(soc0, vp0)
        soc_est = np.zeros(N)
        for k in range(N):
            soc_est[k] = self.step(currents[k], voltages[k])
        return soc_est
