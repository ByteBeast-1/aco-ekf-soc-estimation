"""
ekf.py
======
3-State Extended Kalman Filter (EKF) for lithium-ion battery SOC estimation.

The EKF operates on the Thermal-Adaptive 2-RC battery model (battery_model.py).

State vector  x = [SOC, V1, V2]^T
Input         u = I   (current [A])
Measurement   y = Vt  (terminal voltage [V])

EKF Recursion (Joseph Form for Numerical Stability):
-----------------------------------------------------
Step 1 – State prediction
    x_pred = f(x_prev, u_k, T_k)

Step 2 – Covariance prediction
    P_pred = F_k * P_prev * F_k^T + Q

Step 3 – Innovation (voltage residual)
    r_k = y_k - h(x_pred, u_k, T_k)

Step 4 – Innovation covariance
    S_k = H_k * P_pred * H_k^T + R

Step 5 – Kalman gain
    K_k = P_pred * H_k^T * S_k^-1

Step 6 – State correction
    x_corr = x_pred + K_k * r_k

Step 7 – Covariance correction (Joseph Form)
    P_corr = (I - K_k*H_k) * P_pred * (I - K_k*H_k)^T + K_k * R * K_k^T
"""

import numpy as np
from battery_model import BatteryModel


class ExtendedKalmanFilter:
    """
    3-State EKF for SOC estimation using a 2-RC thermal battery model.
    """

    def __init__(self,
                 model: BatteryModel,
                 Q: np.ndarray = None,
                 R: float = 2.455e-3,
                 x0: np.ndarray = None,
                 P0: np.ndarray = None):
        self.model = model

        # Default 3x3 Q matrix
        if Q is None:
            self.Q = np.diag([1e-7, 1e-7, 1e-7])
        else:
            self.Q = np.asarray(Q, dtype=float)

        self.R = float(R)

        # Default initial state x0 = [0.95, 0.0, 0.0]
        if x0 is None:
            self.x = np.array([0.95, 0.0, 0.0], dtype=float)
        else:
            self.x = np.asarray(x0, dtype=float).flatten()

        # Default 3x3 initial error covariance matrix P0
        if P0 is None:
            self.P = np.diag([0.01, 0.0025, 0.0025])
        else:
            self.P = np.asarray(P0, dtype=float)

        self.I3 = np.eye(3, dtype=float)

    def set_covariances(self, Q: np.ndarray, R: float):
        """Update Q (3x3) and R (scalar) covariances from ACO optimization."""
        self.Q = np.asarray(Q, dtype=float)
        self.R = float(R)

    def step(self, current: float, voltage_meas: float, temp_k: float = 298.15):
        """
        Execute one full EKF Predict-Correct step.

        Parameters
        ----------
        current : float
            Load current [A] (positive = discharge).
        voltage_meas : float
            Measured terminal voltage [V].
        temp_k : float
            Cell temperature in Kelvin (default 298.15 K = 25°C).

        Returns
        -------
        x_corr : ndarray, shape (3,)
            Corrected state vector [SOC, V1, V2]^T.
        P_corr : ndarray, shape (3, 3)
            Corrected error covariance matrix.
        """
        # 1. State prediction
        x_pred = self.model.step(self.x, current, temp_k)

        # 2. Covariance prediction
        F, _ = self.model.state_transition_matrices(temp_k)
        P_pred = F @ self.P @ F.T + self.Q

        # 3. Measurement prediction & residual
        v_pred = self.model.terminal_voltage(x_pred, current, temp_k)
        residual = voltage_meas - v_pred

        # 4. Measurement Jacobian (1x3)
        H = self.model.measurement_jacobian(x_pred)

        # 5. Innovation covariance S (scalar)
        S = float(np.asarray(H @ P_pred @ H.T + self.R).item())

        # 6. Kalman gain K (3x1)
        K = (P_pred @ H.T) / S  # shape (3, 1)

        # 7. State correction
        x_corr = x_pred + (K.flatten() * residual)
        x_corr[0] = np.clip(x_corr[0], 0.0, 1.0)

        # 8. Joseph-form covariance correction for 32-bit numerical stability
        IKH = self.I3 - K @ H
        P_corr = IKH @ P_pred @ IKH.T + K @ np.array([[self.R]]) @ K.T

        # Update internal states
        self.x = x_corr
        self.P = P_corr

        return self.x, self.P
