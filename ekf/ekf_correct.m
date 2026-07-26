function [x_upd, P_upd, VT_pred] = ekf_correct(x_pred, P_pred, I_k, VT_meas, R)
% EKF_CORRECT  Measurement-update (correction) step of the EKF.
%
%   [x_upd, P_upd, VT_pred] = ekf_correct(x_pred, P_pred, I_k, VT_meas, R)
%
%   Compares the model's predicted terminal voltage against the REAL
%   measured terminal voltage, and uses the mismatch (scaled by the
%   Kalman Gain) to correct the state estimate. This is the closed-loop
%   part of the EKF -- without a real VT_meas, this step (and the whole
%   filter's self-correcting behaviour) would not exist.
%
%   INPUT
%       x_pred  - predicted state (a priori) [SoC; Vp; Vc]  (3x1)
%       P_pred  - predicted error covariance (a priori)     (3x3)
%       I_k     - current at this timestep (A)
%       VT_meas - REAL measured terminal voltage (V)
%       R       - measurement noise covariance (scalar)
%
%   OUTPUT
%       x_upd   - corrected (a posteriori) state estimate   (3x1)
%       P_upd   - corrected (a posteriori) error covariance (3x3)
%       VT_pred - the model's predicted voltage, returned for logging/debugging

    SoC_pred = x_pred(1);
    Vp_pred  = x_pred(2);
    Vc_pred  = x_pred(3);

    [OCV, Ri, ~, ~, ~, ~] = battery_params(SoC_pred);
    dOCV = ocv_derivative(SoC_pred);

    % Linearized measurement matrix, C = [dOCV/dSoC, -1, -1]  (Eq. table II)
    C = [dOCV, -1, -1];

    % Nonlinear predicted terminal voltage, Eq. (2)
    VT_pred = OCV - Vp_pred - Vc_pred - I_k * Ri;

    % Innovation (measurement residual) and its covariance
    innovation = VT_meas - VT_pred;
    S = C * P_pred * C' + R;

    % Kalman gain
    K = (P_pred * C') / S;

    % Correct the state and its covariance
    x_upd = x_pred + K * innovation;
    P_upd = (eye(3) - K * C) * P_pred;
end
