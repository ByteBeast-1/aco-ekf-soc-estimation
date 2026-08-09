"""
ekf.py
======
Extended Kalman Filter (EKF) for lithium-ion battery SOC estimation.

The EKF operates on the first-order RC battery model defined in battery_model.py.

State vector  x = [SOC, Vp]
Input         u = I   (current)
Measurement   y = Vt  (terminal voltage)

EKF recursion (executed once per measurement sample)
─────────────────────────────────────────────────────
Notation:
    x̂_{k|k-1}  – prior (predicted) state
    x̂_{k|k}    – posterior (corrected) state
    P_{k|k-1}  – prior covariance
    P_{k|k}    – posterior covariance
    Q           – process-noise covariance  (2×2)
    R           – measurement-noise covariance  (1×1 scalar)
    F_k         – state-transition Jacobian   ∂f/∂x
    H_k         – measurement Jacobian        ∂h/∂x

Step 1 – State prediction
    x̂_{k|k-1} = f(x̂_{k-1|k-1}, u_k)

Step 2 – Covariance prediction
    P_{k|k-1} = F_k · P_{k-1|k-1} · F_kᵀ + Q

Step 3 – Innovation (voltage residual)
    r_k = y_k − h(x̂_{k|k-1}, u_k)

Step 4 – Innovation covariance
    S_k = H_k · P_{k|k-1} · H_kᵀ + R

Step 5 – Kalman gain
    K_k = P_{k|k-1} · H_kᵀ · S_k⁻¹

Step 6 – State correction
    x̂_{k|k} = x̂_{k|k-1} + K_k · r_k

Step 7 – Covariance correction  (Joseph form for numerical stability)
    P_{k|k} = (I − K_k·H_k)·P_{k|k-1}·(I − K_k·H_k)ᵀ + K_k·R·K_kᵀ
"""

import numpy as np
from battery_model import BatteryModel


class ExtendedKalmanFilter:
    """
    EKF for SOC estimation using a first-order RC battery model.

    Parameters
    ----------
    model : BatteryModel
        Battery model that provides f, h, F, H.
    Q : ndarray, shape (2, 2)
        Process-noise covariance matrix.
        Diagonal elements represent uncertainty in SOC integration
        and polarisation voltage dynamics.
    R : float
        Measurement-noise variance  [V²].
        Represents uncertainty in the voltage sensor reading.
    P0 : ndarray, shape (2, 2), optional
        Initial state-estimation error covariance.
        Larger diagonal → more uncertainty in initial state.
    """

    def __init__(
        self,
        model:   BatteryModel,
        Q:       np.ndarray,
        R:       float,
        P0:      np.ndarray | None = None,
    ):
        self.model = model
        self.Q     = np.atleast_2d(Q).astype(float)
        self.R     = float(R)

        if P0 is None:
            self.P0 = np.diag([0.1**2, 0.05**2])   # moderate initial uncertainty
        else:
            self.P0 = np.atleast_2d(P0).astype(float)

        # These will be set on reset()
        self._x_hat = None
        self._P     = None

    # ------------------------------------------------------------------
    # Initialise / reset the filter
    # ------------------------------------------------------------------
    def reset(self, soc0: float = 1.0, vp0: float = 0.0):
        """
        Initialise the filter state.

        Parameters
        ----------
        soc0 : float  – initial SOC estimate  (0 … 1)
        vp0  : float  – initial polarisation voltage estimate  [V]
        """
        self._x_hat = np.array([soc0, vp0], dtype=float)
        self._P     = self.P0.copy()

    # ------------------------------------------------------------------
    # Single filter step
    # ------------------------------------------------------------------
    def step(self, I: float, Vt_measured: float) -> float:
        """
        Run one EKF update cycle given current and terminal voltage.

        Parameters
        ----------
        I           : float  – measured current (A), positive = discharge
        Vt_measured : float  – measured terminal voltage (V)

        Returns
        -------
        soc_estimate : float  – corrected SOC estimate after this step
        """
        if self._x_hat is None:
            raise RuntimeError("Call reset() before the first step().")

        model = self.model
        x     = self._x_hat
        P     = self._P

        # ── Step 1: State prediction ──────────────────────────────────
        x_pred = model.state_transition(x, I)

        # ── Step 2: Covariance prediction ────────────────────────────
        F      = model.state_jacobian()          # constant 2×2
        P_pred = F @ P @ F.T + self.Q

        # ── Step 3: Innovation ───────────────────────────────────────
        Vt_pred   = model.terminal_voltage(x_pred, I)
        innovation = Vt_measured - Vt_pred

        # ── Step 4: Innovation covariance ────────────────────────────
        H   = model.measurement_jacobian(x_pred[0])   # 1×2
        S   = H @ P_pred @ H.T + self.R               # 1×1 scalar

        # ── Step 5: Kalman gain ──────────────────────────────────────
        K   = (P_pred @ H.T) / float(S)               # 2×1 vector

        # ── Step 6: State correction ─────────────────────────────────
        x_upd = x_pred + K.flatten() * float(innovation)
        x_upd[0] = float(np.clip(x_upd[0], 0.0, 1.0))   # SOC ∈ [0, 1]

        # ── Step 7: Covariance correction (Joseph form) ───────────────
        I_mat  = np.eye(2)
        IKH    = I_mat - np.outer(K.flatten(), H.flatten())
        P_upd  = IKH @ P_pred @ IKH.T + np.outer(K, K) * self.R

        # Store for next step
        self._x_hat = x_upd
        self._P     = P_upd

        return float(x_upd[0])   # return SOC estimate

    # ------------------------------------------------------------------
    # Batch run over a current/voltage profile
    # ------------------------------------------------------------------
    def run(
        self,
        currents:  np.ndarray,
        voltages:  np.ndarray,
        soc0:      float = 1.0,
        vp0:       float = 0.0,
    ) -> np.ndarray:
        """
        Run the EKF over an entire data sequence.

        Parameters
        ----------
        currents : shape (N,)  – current profile [A]
        voltages : shape (N,)  – measured voltage profile [V]
        soc0     : float       – initial SOC estimate
        vp0      : float       – initial Vp estimate

        Returns
        -------
        soc_estimates : shape (N,)
        """
        N = len(currents)
        assert len(voltages) == N, "currents and voltages must have the same length."

        self.reset(soc0, vp0)
        soc_est = np.zeros(N)

        for k in range(N):
            soc_est[k] = self.step(currents[k], voltages[k])

        return soc_est

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def state(self) -> np.ndarray:
        """Current state estimate [SOC, Vp]."""
        return self._x_hat.copy() if self._x_hat is not None else None

    @property
    def covariance(self) -> np.ndarray:
        """Current error covariance matrix P."""
        return self._P.copy() if self._P is not None else None
