% MAIN_WEEK2  Week 2 deliverable: baseline EKF vs. plain Coulomb Counting.
%
%   Uses Week 1's plant model (simulate_battery.m) to generate a TRUE
%   battery trajectory plus NOISY current/voltage measurements, then:
%     1. Runs the baseline EKF (fixed, literature-style Q and R) to
%        estimate SoC from the noisy measurements.
%     2. Runs plain Coulomb Counting on the SAME noisy current, starting
%        from the SAME (deliberately imperfect) initial guess, so you
%        can see EKF's self-correction vs. CCM's open-loop drift.
%
%   HOW TO RUN
%       In MATLAB: cd into this folder, then run main_week2

clear; clc; close all;

this_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(this_dir, '..', 'models'));
addpath(fullfile(this_dir, '..', 'simulation'));

%% --- Setup ---
Qn_Ah  = 12;              % nominal capacity, Ah
Qn_As  = Qn_Ah * 3600;    % Amp-seconds
dt     = 1;               % 1 s sampling
SoC0_true = 1.0;          % battery actually starts fully charged

rng(1);   % fixed random seed so results are repeatable while you're testing

%% --- Generate ground truth + noisy sensor data (Week 1 machinery) ---
[t, I] = generate_profile('pulse', dt, 6000);
out = simulate_battery(t, I, SoC0_true, Qn_Ah, true);   % add_noise = true
n = length(t);

%% --- Baseline EKF setup ---
% NOTE: these are hand-picked, literature-style values -- exactly the
% kind of guess Week 3's ACO reproduction will replace with a tuned
% value, and Week 4's MOACO will replace with a whole Pareto front.
Q_ekf = diag([1e-6, 1e-6, 1e-6]);
R_ekf = 0.01;

% Deliberately start the EKF with a WRONG initial guess (0.9 instead of
% the true 1.0) to demonstrate that it self-corrects using measurements,
% rather than just trusting its own starting point.
SoC0_guess = 0.9;

x_ekf = zeros(3, n);
x_ekf(:, 1) = [SoC0_guess; 0; 0];
P = diag([0.01, 0.001, 0.001]);   % initial uncertainty

%% --- Plain Coulomb Counting, same wrong initial guess, same noisy current ---
SoC_ccm = zeros(n, 1);
SoC_ccm(1) = SoC0_guess;

%% --- Run both estimators forward in time ---
for k = 1:n-1
    % EKF: predict using current, then correct using measured voltage
    [x_pred, P_pred] = ekf_predict(x_ekf(:, k), P, out.I_meas(k), dt, Qn_As, Q_ekf);
    [x_upd, P, ~] = ekf_correct(x_pred, P_pred, out.I_meas(k), out.VT_meas(k), R_ekf);
    x_ekf(:, k+1) = x_upd;

    % Coulomb Counting: pure open-loop integration, no correction at all
    SoC_ccm(k+1) = SoC_ccm(k) - out.I_meas(k) * dt / Qn_As;
end

SoC_ekf = x_ekf(1, :)';

%% --- Compare accuracy: EKF vs. CCM, both against true SoC ---
mse_ekf = mean((SoC_ekf - out.SoC_true).^2);
mse_ccm = mean((SoC_ccm - out.SoC_true).^2);
mae_ekf = mean(abs(SoC_ekf - out.SoC_true));
mae_ccm = mean(abs(SoC_ccm - out.SoC_true));

fprintf('--- Week 2 baseline EKF vs. plain Coulomb Counting ---\n');
fprintf('%-22s %10s %10s\n', '', 'MSE', 'MAE');
fprintf('%-22s %10.6f %10.6f\n', 'EKF (this project)', mse_ekf, mae_ekf);
fprintf('%-22s %10.6f %10.6f\n', 'Coulomb Counting only', mse_ccm, mae_ccm);
fprintf('\nBoth started from the SAME wrong initial guess (SoC=%.2f, true=%.2f).\n', ...
        SoC0_guess, SoC0_true);
fprintf('EKF should recover toward the true trajectory; CCM should not.\n');

%% --- Plot ---
figure('Position', [100, 100, 900, 500]);
plot(t, out.SoC_true, 'k-',  'LineWidth', 2); hold on;
plot(t, SoC_ekf,      'b-',  'LineWidth', 1.5);
plot(t, SoC_ccm,      'r--', 'LineWidth', 1.5);
legend('True SoC (ground truth)', 'EKF estimate', 'Coulomb Counting only', ...
       'Location', 'best');
xlabel('Time (s)');
ylabel('SoC');
title('Week 2: Baseline EKF vs. Coulomb Counting (both start from a wrong initial guess)');
grid on;

results_dir = fullfile(this_dir, '..', 'results', 'plots');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
saveas(gcf, fullfile(results_dir, 'week2_ekf_vs_ccm.png'));
fprintf('\nSaved plot to results/plots/week2_ekf_vs_ccm.png\n');

% ---------------------------------------------------------------------
% Sanity checks to confirm by eye:
%   1. The EKF line should start at 0.9, then bend toward the true SoC
%      line within the first few hundred seconds -- that's the
%      closed-loop correction in action.
%   2. The Coulomb Counting line should stay parallel to the true SoC
%      line, offset by roughly the same initial 0.1 error the whole way
%      -- it never corrects itself.
%   3. EKF's MSE/MAE printed above should be clearly lower than CCM's.
% ---------------------------------------------------------------------
