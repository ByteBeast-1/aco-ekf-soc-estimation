"""
adaptive_hybrid_ekf.py
======================
Sage-Husa Adaptive Hybrid Extended Kalman Filter (AH-EKF).
Uses Joseph-Form updates, online parameters (from RLS), and residual corrections (from MLP NN).
"""

import numpy as np
from battery_model import BatteryModel, ocv_from_soc, docv_dsoc


class AdaptiveHybridEKF:
    """
    Sage-Husa Adaptive Hybrid Extended Kalman Filter.
    Estimates state x = [SOC, V1, V2]^T under parameter drift and model mismatch.
    """

    def __init__(self,
                 model: BatteryModel,
                 x0: list = None,
                 P0: np.ndarray = None,
                 Q_init: np.ndarray = None,
                 R_init: float = 1e-3,
                 d_cov: float = 0.05):
        """
        model: Standard BatteryModel
        x0: Initial state [SOC, V1, V2]
        P0: Initial error covariance matrix (3x3)
        Q_init: Initial process noise covariance matrix (3x3)
        R_init: Initial measurement noise covariance variance (float)
        d_cov: Sage-Husa forgetting coefficient (usually 0.01 to 0.1)
        """
        self.model = model
        self.dt = model.dt

        # Initial state
        if x0 is None:
            self.x = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            self.x = np.array(x0, dtype=float).flatten()

        # Initial covariance P
        if P0 is None:
            self.P = np.diag([0.01, 0.0025, 0.0025])
        else:
            self.P = np.array(P0, dtype=float)

        # Process noise covariance matrix Q (3x3)
        if Q_init is None:
            self.Q = np.diag([1e-6, 1e-6, 1e-6])
        else:
            self.Q = np.array(Q_init, dtype=float)

        # Measurement noise covariance variance R (scalar)
        self.R = float(R_init)

        # Sage-Husa parameter
        self.d_cov = d_cov

    def step(self,
             current: float,
             voltage_meas: float,
             temp_k: float,
             rls_params: tuple = None,
             residual_corrector=None) -> tuple:
        """
        Run one iteration of the Adaptive Hybrid EKF.
        current: Measured current [A]
        voltage_meas: Measured terminal voltage [V]
        temp_k: Measured temperature [K]
        rls_params: Optional RLS parameter tuple (R0, R1, C1, R2, C2)
        residual_corrector: Optional ResidualCorrector neural network
        """
        # Save previous state and covariance for Sage-Husa updates
        x_prev = self.x.copy()
        P_prev = self.P.copy()

        # -------------------------------------------------------------------
        # 1. Prediction (Time Update)
        # -------------------------------------------------------------------
        # Determine model parameters (RLS or default temperature compensated)
        if rls_params is not None:
            R0, R1, C1, R2, C2 = rls_params
        else:
            R0, R1, R2 = self.model.get_thermal_params(temp_k)
            C1, C2 = self.model.C1, self.model.C2

        # State-space transition parameters
        a1 = np.exp(-self.dt / (R1 * C1))
        b1 = R1 * (1.0 - a1)
        a2 = np.exp(-self.dt / (R2 * C2))
        b2 = R2 * (1.0 - a2)

        F = np.array([
            [1.0, 0.0, 0.0],
            [0.0, a1,  0.0],
            [0.0, 0.0, a2 ]
        ], dtype=float)

        B = np.array([
            -(self.model.eta * self.dt) / self.model.Qn,
            b1,
            b2
        ], dtype=float)

        # Predict State: x_pred = F * x + B * I
        x_pred = F @ self.x + B * current
        x_pred[0] = np.clip(x_pred[0], 0.0, 1.0)

        # Predict Covariance: P_pred = F * P * F^T + Q
        P_pred = F @ self.P @ F.T + self.Q

        # -------------------------------------------------------------------
        # 2. Correction (Measurement Update)
        # -------------------------------------------------------------------
        # Base physical voltage prediction
        v_ocv = ocv_from_soc(x_pred[0])
        voltage_pred_physics = v_ocv - x_pred[1] - x_pred[2] - current * R0

        # AI residual correction if NN is provided
        if residual_corrector is not None:
            v_correction = residual_corrector.predict(x_pred[0], current, temp_k, voltage_pred_physics)
            voltage_pred_hybrid = voltage_pred_physics + v_correction
        else:
            voltage_pred_hybrid = voltage_pred_physics

        # Innovation (residual)
        r = voltage_meas - voltage_pred_hybrid

        # Measurement Jacobian H
        H = np.array([docv_dsoc(x_pred[0]), -1.0, -1.0], dtype=float)

        # Innovation covariance
        S = H @ P_pred @ H.T + self.R

        # Kalman Gain
        K = (P_pred @ H.T) / S

        # State Update
        self.x = x_pred + K * r
        self.x[0] = np.clip(self.x[0], 0.0, 1.0)

        # Joseph-Form Covariance Update
        I = np.eye(3)
        IKH = I - np.outer(K, H)
        self.P = IKH @ P_pred @ IKH.T + np.outer(K * self.R, K)

        # -------------------------------------------------------------------
        # 3. Sage-Husa Covariance Adaptation
        # -------------------------------------------------------------------
        d = self.d_cov

        # Update R (measurement noise variance)
        # R_new = (1-d)*R + d*(r^2 - H * P_pred * H^T)
        R_SH = r**2 - H @ P_pred @ H.T
        R_new = (1.0 - d) * self.R + d * R_SH
        # Enforce positive lower bound and upper cap
        self.R = float(np.clip(R_new, 1e-5, 5e-2))

        # Update Q (process noise matrix)
        # Q_new = (1-d)*Q + d*(K * r * r^T * K^T + P - F * P_prev * F^T)
        Q_SH = np.outer(K * r, K * r) + self.P - F @ P_prev @ F.T
        Q_new = (1.0 - d) * self.Q + d * Q_SH
        
        # Enforce positive diagonal elements for semi-definiteness
        Q_diag = np.diagonal(Q_new)
        Q_diag = np.clip(Q_diag, 1e-9, 1e-4)
        self.Q = np.diag(Q_diag)

        return self.x, self.P, self.Q, self.R
