function [x_pred, P_pred] = ekf_predict(x_prev, P_prev, I_k, dt, Qn_As, Q)
% EKF_PREDICT  Time-update (prediction) step of the Extended Kalman Filter.
%
%   [x_pred, P_pred] = ekf_predict(x_prev, P_prev, I_k, dt, Qn_As, Q)
%
%   State vector: x = [SoC; Vp; Vc]
%
%   Builds the A and B matrices (Eq. 14 in the base paper) using the
%   PREVIOUS state's SoC estimate to look up Rp, Cp, Rc, Cc -- this is
%   exactly why battery_params.m is recomputed every step instead of
%   being a fixed lookup.
%
%   INPUT
%       x_prev - previous state estimate [SoC; Vp; Vc]  (3x1)
%       P_prev - previous error covariance                (3x3)
%       I_k    - current at this timestep (A), positive = discharging
%       dt     - sample time (s)
%       Qn_As  - nominal capacity in Amp-seconds (Ah * 3600)
%       Q      - process noise covariance                 (3x3)
%
%   OUTPUT
%       x_pred - predicted state (a priori)                (3x1)
%       P_pred - predicted error covariance (a priori)      (3x3)

    SoC_prev = x_prev(1);
    [~, ~, Rp, Rc, Cp, Cc] = battery_params(SoC_prev); %#ok<ASGLU>
    % (OCV, Ri from battery_params aren't needed for the predict step --
    %  only the correct step uses them -- so they're discarded here.)

    aP = exp(-dt / (Rp * Cp));
    aC = exp(-dt / (Rc * Cc));

    A = [1,  0,  0;
         0, aP,  0;
         0,  0, aC];

    B = [-dt / Qn_As;
         Rp * (1 - aP);
         Rc * (1 - aC)];

    x_pred = A * x_prev + B * I_k;
    P_pred = A * P_prev * A' + Q;
end
