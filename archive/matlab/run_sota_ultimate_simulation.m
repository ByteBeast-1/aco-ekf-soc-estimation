function run_sota_ultimate_simulation()
% RUN_SOTA_ULTIMATE_SIMULATION
% Fuses Online RLS, Multi-Variable Polynomial Residual Correction, 
% and Sage-Husa Adaptive EKF on NASA B0005 lab data in MATLAB.

    clc; close all;
    fprintf('=================================================================\n');
    fprintf('  SOTA BMS MATLAB UPGRADE: ADAPTIVE HYBRID EKF + ONLINE RLS\n');
    fprintf('=================================================================\n\n');

    % Step 1: Load NASA B0005 dataset
    fprintf('[Step 1/5] Loading NASA Ames Cell B0005 lab data...\n');
    csv_file = '../data/NASA_B0005.csv';
    if ~exist(csv_file, 'file')
        % If running inside simulink_matlab folder, check one level up
        csv_file = 'data/NASA_B0005.csv';
        if ~exist(csv_file, 'file')
            error('NASA B0005 dataset not found! Please run download_nasa_dataset.py first.');
        end
    end
    
    data = readmatrix(csv_file);
    time_s = data(:, 1);
    current_A = data(:, 2);
    voltage_V = data(:, 3);
    temp_C = data(:, 4);
    soc_ref = data(:, 5);
    
    if max(soc_ref) > 1.0
        soc_ref = soc_ref / 100.0;
    end
    temp_k = temp_C + 273.15;
    N = length(time_s);

    % Configure Battery Model Parameters
    cfg.Qn = 7200.0;
    cfg.R0_ref = 0.030;
    cfg.R1_ref = 0.015;
    cfg.C1 = 2500.0;
    cfg.R2_ref = 0.020;
    cfg.C2 = 10000.0;
    cfg.eta = 1.0;
    cfg.dt = 1.0;
    cfg.Ea = 18000.0;
    cfg.T_ref = 298.15;
    cfg.Rg = 8.314;
    cfg.ocv_coeffs = [-21.7497, 82.2625, -125.3296, 99.6313, -44.5010, 11.3416, -1.5825, 1.5032, 3.0130];

    % Split data: 50% Train, 50% Test
    split_idx = floor(N / 2);
    
    % Step 2: Collect training residuals
    fprintf('[Step 2/5] Collecting training residuals from physical data slice...\n');
    [X_train, y_train] = collect_training_residuals(time_s(1:split_idx), current_A(1:split_idx), ...
        voltage_V(1:split_idx), temp_k(1:split_idx), soc_ref(1:split_idx), cfg);
    
    % Step 3: Train Multi-Variable Regression model for residuals
    % Feature columns: [SOC, I, T, V_pred]
    fprintf('[Step 3/5] Fitting Multi-Variable Polynomial residual model...\n');
    % We fit a polynomial: dy = w1*SOC + w2*I + w3*T + w4*V + w5*SOC^2 + w6*I^2 + w7*1
    X_reg = [X_train(:,1), X_train(:,2), X_train(:,3), X_train(:,4), ...
             X_train(:,1).^2, X_train(:,2).^2, ones(size(X_train,1), 1)];
    weights = X_reg \ y_train; % Linear least squares regression (Zero dependencies!)

    % Step 4: Race Observers on test slice
    fprintf('[Step 4/5] Racing estimators on test slice under -15%% initial guess offset...\n');
    t_test = time_s(split_idx:end);
    I_test = current_A(split_idx:end);
    V_test = voltage_V(split_idx:end);
    T_test = temp_k(split_idx:end);
    soc_test = soc_ref(split_idx:end);
    
    res = run_evaluation(t_test, I_test, V_test, T_test, soc_test, weights, cfg);

    % Step 5: Metrics and Plotting
    m_cc = get_metrics(soc_test, res.soc_cc);
    m_conv = get_metrics(soc_test, res.soc_conv);
    m_aekf = get_metrics(soc_test, res.soc_aekf);
    m_hybrid = get_metrics(soc_test, res.soc_hybrid);

    fprintf('\nNASA B0005 MATLAB Hardware Benchmark Results Summary:\n');
    fprintf('==========================================================================================\n');
    fprintf('%-26s | %-10s | %-9s | %-14s | %-8s\n', 'Observer Algorithm', 'MSE', 'MAE (%)', 'Max Error (%)', 'R^2');
    fprintf('------------------------------------------------------------------------------------------\n');
    fprintf('%-26s | %-10.3e | %-9.4f | %-14.4f | %-8.4f\n', 'Coulomb Counting', m_cc.mse, m_cc.mae*100, max(abs(soc_test-res.soc_cc))*100, m_cc.r2);
    fprintf('%-26s | %-10.3e | %-9.4f | %-14.4f | %-8.4f\n', 'Conventional EKF', m_conv.mse, m_conv.mae*100, max(abs(soc_test-res.soc_conv))*100, m_conv.r2);
    fprintf('%-26s | %-10.3e | %-9.4f | %-14.4f | %-8.4f\n', 'Adaptive EKF + RLS', m_aekf.mse, m_aekf.mae*100, max(abs(soc_test-res.soc_aekf))*100, m_aekf.r2);
    fprintf('%-26s | %-10.3e | %-9.4f | %-14.4f | %-8.4f\n', 'Adaptive Hybrid EKF (Ours)', m_hybrid.mse, m_hybrid.mae*100, max(abs(soc_test-res.soc_hybrid))*100, m_hybrid.r2);
    fprintf('==========================================================================================\n');

    % Plot Figures
    plot_results(t_test, soc_test, res);
end

%% ====================================================================
function [X, y] = collect_training_residuals(t, I, V, T, soc_ref, cfg)
    N = length(t);
    X = zeros(N, 4);
    y = zeros(N, 1);
    
    % Initialize filters
    theta_rls = [-1.95; 0.95; 0.02; -0.038; 0.018];
    P_rls = eye(5) * 1e4;
    y_hist = zeros(2,1);
    I_hist = zeros(3,1);
    
    % EKF states
    x = [soc_ref(1); 0.0; 0.0];
    P = diag([0.01, 0.0025, 0.0025]);
    Q = diag([1e-6, 1e-6, 1e-6]);
    R = 1e-3;

    for k = 1:N
        curr = I(k);
        volt = V(k);
        temp = T(k);
        
        soc_est = x(1);
        v_ocv = polyval(cfg.ocv_coeffs, soc_est);
        
        % Extract parameters from RLS
        [R0, R1, C1, R2, C2] = map_rls(theta_rls, cfg);
        v_pred = v_ocv - x(2) - x(3) - curr * R0;
        
        X(k, :) = [soc_est, curr, temp, v_pred];
        y(k) = volt - v_pred;
        
        % Update standard EKF
        [x, P] = step_ekf(x, P, curr, volt, temp, R0, R1, C1, R2, C2, Q, R, cfg);
        
        % Update RLS
        [theta_rls, P_rls, y_hist, I_hist] = step_rls(theta_rls, P_rls, y_hist, I_hist, curr, volt, v_ocv);
    end
end

%% ====================================================================
function res = run_evaluation(t, I, V, T, soc_ref, weights, cfg)
    N = length(t);
    soc0_bad = min(max(soc_ref(1) - 0.15, 0.05), 0.95);
    
    res.soc_cc = zeros(N, 1);
    res.soc_conv = zeros(N, 1);
    res.soc_aekf = zeros(N, 1);
    res.soc_hybrid = zeros(N, 1);
    res.r0_rls = zeros(N, 1);
    res.r1_rls = zeros(N, 1);
    res.sh_r = zeros(N, 1);
    res.sh_q = zeros(N, 1);
    res.corr = zeros(N, 1);

    % 1. CC
    soc_cc_val = soc0_bad;
    for k = 1:N
        soc_cc_val = soc_cc_val - (cfg.eta * I(k) * cfg.dt) / cfg.Qn;
        res.soc_cc(k) = min(max(soc_cc_val, 0.0), 1.0);
    end

    % 2. Conv EKF
    x_conv = [soc0_bad; 0.0; 0.0];
    P_conv = diag([0.01, 0.0025, 0.0025]);
    for k = 1:N
        % Nominal NASA parameters
        [x_conv, P_conv] = step_ekf(x_conv, P_conv, I(k), V(k), T(k), 0.030, 0.015, 2500.0, 0.020, 10000.0, diag([1e-5, 1e-5, 1e-5]), 1e-3, cfg);
        res.soc_conv(k) = x_conv(1);
    end

    % 3. Adaptive EKF + RLS
    theta_rls_only = [-1.95; 0.95; 0.02; -0.038; 0.018];
    P_rls_only = eye(5) * 1e4;
    y_hist_only = zeros(2,1);
    I_hist_only = zeros(3,1);
    
    x_aekf = [soc0_bad; 0.0; 0.0];
    P_aekf = diag([0.01, 0.0025, 0.0025]);
    Q_aekf = diag([1e-6, 1e-6, 1e-6]);
    R_aekf = 1e-3;

    for k = 1:N
        curr = I(k); volt = V(k); temp = T(k);
        [R0, R1, C1, R2, C2] = map_rls(theta_rls_only, cfg);
        
        [x_aekf, P_aekf, Q_aekf, R_aekf] = step_aekf(x_aekf, P_aekf, Q_aekf, R_aekf, curr, volt, temp, R0, R1, C1, R2, C2, 0, weights, cfg);
        res.soc_aekf(k) = x_aekf(1);
        [theta_rls_only, P_rls_only, y_hist_only, I_hist_only] = step_rls(theta_rls_only, P_rls_only, y_hist_only, I_hist_only, curr, volt, polyval(cfg.ocv_coeffs, x_aekf(1)));
    end

    % 4. Ultimate Hybrid EKF + RLS
    theta_rls_hybrid = [-1.95; 0.95; 0.02; -0.038; 0.018];
    P_rls_hybrid = eye(5) * 1e4;
    y_hist_hybrid = zeros(2,1);
    I_hist_hybrid = zeros(3,1);
    
    x_hyb = [soc0_bad; 0.0; 0.0];
    P_hyb = diag([0.01, 0.0025, 0.0025]);
    Q_hyb = diag([1e-6, 1e-6, 1e-6]);
    R_hyb = 1e-3;

    for k = 1:N
        curr = I(k); volt = V(k); temp = T(k);
        [R0, R1, C1, R2, C2] = map_rls(theta_rls_hybrid, cfg);
        res.r0_rls(k) = R0;
        res.r1_rls(k) = R1;

        [x_hyb, P_hyb, Q_hyb, R_hyb, dy] = step_aekf(x_hyb, P_hyb, Q_hyb, R_hyb, curr, volt, temp, R0, R1, C1, R2, C2, 1, weights, cfg);
        res.soc_hybrid(k) = x_hyb(1);
        res.sh_q(k) = Q_hyb(1,1);
        res.sh_r(k) = R_hyb;
        res.corr(k) = dy;

        [theta_rls_hybrid, P_rls_hybrid, y_hist_hybrid, I_hist_hybrid] = step_rls(theta_rls_hybrid, P_rls_hybrid, y_hist_hybrid, I_hist_hybrid, curr, volt, polyval(cfg.ocv_coeffs, x_hyb(1)));
    end
end

%% ====================================================================
function [x, P] = step_ekf(x, P, I, V, T, R0, R1, C1, R2, C2, Q, R, cfg)
    a1 = exp(-cfg.dt / (R1 * C1));
    b1 = R1 * (1 - a1);
    a2 = exp(-cfg.dt / (R2 * C2));
    b2 = R2 * (1 - a2);
    
    F = diag([1.0, a1, a2]);
    B = [-(cfg.eta * cfg.dt)/cfg.Qn; b1; b2];
    
    % Predict
    x_pred = F * x + B * I;
    x_pred(1) = min(max(x_pred(1), 0.0), 1.0);
    P_pred = F * P * F' + Q;
    
    % Correct
    v_ocv = polyval(cfg.ocv_coeffs, x_pred(1));
    v_pred = v_ocv - x_pred(2) - x_pred(3) - I * R0;
    r = V - v_pred;
    
    % Jacobian
    deriv_coeffs = polyder(cfg.ocv_coeffs);
    H = [polyval(deriv_coeffs, x_pred(1)), -1.0, -1.0];
    
    S = H * P_pred * H' + R;
    K = (P_pred * H') / S;
    
    x = x_pred + K * r;
    x(1) = min(max(x(1), 0.0), 1.0);
    
    IKH = eye(3) - K*H;
    P = IKH * P_pred * IKH' + K*R*K';
end

%% ====================================================================
function [x, P, Q, R, dy] = step_aekf(x, P, Q, R, I, V, T, R0, R1, C1, R2, C2, use_nn, weights, cfg)
    a1 = exp(-cfg.dt / (R1 * C1));
    b1 = R1 * (1 - a1);
    a2 = exp(-cfg.dt / (R2 * C2));
    b2 = R2 * (1 - a2);
    
    F = diag([1.0, a1, a2]);
    B = [-(cfg.eta * cfg.dt)/cfg.Qn; b1; b2];
    
    x_prev = x;
    P_prev = P;
    
    % Predict
    x_pred = F * x + B * I;
    x_pred(1) = min(max(x_pred(1), 0.0), 1.0);
    P_pred = F * P * F' + Q;
    
    % Correct with Hybrid Compensation
    v_ocv = polyval(cfg.ocv_coeffs, x_pred(1));
    v_pred_physics = v_ocv - x_pred(2) - x_pred(3) - I * R0;
    
    dy = 0;
    if use_nn
        % We evaluate our regression feature matrix:
        % [SOC, I, T, V_pred, SOC^2, I^2, 1]
        feat = [x_pred(1), I, T, v_pred_physics, x_pred(1)^2, I^2, 1];
        dy = feat * weights;
    end
    
    v_pred_hybrid = v_pred_physics + dy;
    r = V - v_pred_hybrid;
    
    deriv_coeffs = polyder(cfg.ocv_coeffs);
    H = [polyval(deriv_coeffs, x_pred(1)), -1.0, -1.0];
    
    S = H * P_pred * H' + R;
    K = (P_pred * H') / S;
    
    x = x_pred + K * r;
    x(1) = min(max(x(1), 0.0), 1.0);
    
    IKH = eye(3) - K*H;
    P = IKH * P_pred * IKH' + K*R*K';
    
    % Sage-Husa Covariance Adaptation
    d = 0.05;
    R_new = (1 - d)*R + d*(r^2 - H * P_pred * H');
    R = min(max(R_new, 1e-5), 5e-2);
    
    Q_new = (1 - d)*Q + d*( (K*r)*(K*r)' + P - F*P_prev*F' );
    Q_diag = diag(Q_new);
    Q_diag = min(max(Q_diag, 1e-9), 1e-4);
    Q = diag(Q_diag);
end

%% ====================================================================
function [theta, P, y_hist, I_hist] = step_rls(theta, P, y_hist, I_hist, I, V, ocv)
    y = ocv - V;
    phi = [-y_hist(1); -y_hist(2); I; I_hist(1); I_hist(2)];
    
    lambda = 0.995;
    den = lambda + phi' * P * phi;
    if abs(den) > 1e-6
        K = (P * phi) / den;
        err = y - phi' * theta;
        theta = theta + K * err;
        P = (P - K * phi' * P) / lambda;
    end
    
    y_hist(2) = y_hist(1); y_hist(1) = y;
    I_hist(3) = I_hist(2); I_hist(2) = I_hist(1); I_hist(1) = I;
end

%% ====================================================================
function [R0, R1, C1, R2, C2] = map_rls(theta, cfg)
    c1 = theta(1); c2 = theta(2); d0 = theta(3); d1 = theta(4); d2 = theta(5);
    
    c2 = min(max(c2, 0.01), 0.9999);
    disc = c1^2 - 4 * c2;
    if disc < 0
        a1 = 0.999; a2 = 0.995;
    else
        roots_val = sqrt(disc);
        a1 = (-c1 + roots_val) / 2;
        a2 = (-c1 - roots_val) / 2;
        a1 = min(max(a1, 0.5), 0.9999);
        a2 = min(max(a2, 0.5), 0.9999);
    end
    
    if abs(a1 - a2) < 1e-4
        a2 = a1 * 0.98;
    end
    
    R0 = d2 / (a1 * a2);
    R0 = min(max(R0, 0.001), 0.150);
    
    y1 = d0 - R0;
    y2 = -d1 + R0 * c1;
    det_val = a1 - a2;
    b1 = (a1 * y1 - y2) / det_val;
    b2 = y1 - b1;
    
    R1 = b1 / (1 - a1);
    R2 = b2 / (1 - a2);
    R1 = min(max(R1, 0.001), 0.100);
    R2 = min(max(R2, 0.001), 0.150);
    
    C1 = -cfg.dt / (R1 * log(a1));
    C2 = -cfg.dt / (R2 * log(a2));
    C1 = min(max(C1, 100), 10000);
    C2 = min(max(C2, 1000), 50000);
end

%% ====================================================================
function m = get_metrics(ref, est)
    err = ref - est;
    m.mse = mean(err.^2);
    m.mae = mean(abs(err));
    ss_res = sum(err.^2);
    ss_tot = sum((ref - mean(ref)).^2);
    m.r2 = 1.0 - (ss_res / (ss_tot + 1e-12));
end

%% ====================================================================
function plot_results(t, soc_test, res)
    % Figure 1: SOC Tracking
    figure('Color', [1 1 1], 'Position', [100, 100, 850, 420]);
    plot(t, soc_test * 100, 'k', 'LineWidth', 2); hold on;
    plot(t, res.soc_cc * 100, '--', 'Color', [0.5 0.5 0.5]);
    plot(t, res.soc_conv * 100, ':', 'Color', [0.9 0.5 0]);
    plot(t, res.soc_aekf * 100, '-.', 'Color', [0 0.4 0.8]);
    plot(t, res.soc_hybrid * 100, 'g', 'LineWidth', 1.5);
    xlabel('Time (seconds)'); ylabel('State of Charge (SOC %)');
    title('SOC Estimation Race (MATLAB Upgrade) - NASA Cell B0005');
    legend('NASA Reference SOC', 'Coulomb Counting', 'Conventional EKF', 'Adaptive EKF + RLS', 'Adaptive Hybrid EKF (Ours)', 'Location', 'best');
    grid on; ax = gca; ax.GridAlpha = 0.3;
    saveas(gcf, '../figures/13_matlab_sota_soc.png');

    % Figure 2: Error Zoom
    figure('Color', [1 1 1], 'Position', [150, 150, 850, 380]);
    plot(t, (soc_test - res.soc_cc)*100, 'Color', [0.5 0.5 0.5]); hold on;
    plot(t, (soc_test - res.soc_conv)*100, 'Color', [0.9 0.5 0]);
    plot(t, (soc_test - res.soc_aekf)*100, 'Color', [0 0.4 0.8]);
    plot(t, (soc_test - res.soc_hybrid)*100, 'g', 'LineWidth', 1.5);
    yline(0, 'r--', 'LineWidth', 1.0);
    xlabel('Time (seconds)'); ylabel('SOC Estimation Error (%)');
    title('SOC Error Convergence and Tracking Profile');
    legend('Coulomb Counting', 'Conventional EKF', 'Adaptive EKF + RLS', 'Adaptive Hybrid EKF Error', 'Location', 'best');
    grid on; ax = gca; ax.GridAlpha = 0.3;
    saveas(gcf, '../figures/14_matlab_sota_error.png');
end
