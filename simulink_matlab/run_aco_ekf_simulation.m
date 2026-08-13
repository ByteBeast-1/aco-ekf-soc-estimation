%% ACO-EKF Lithium-Ion Battery SOC Estimation Simulation (MATLAB)
%  ========================================================================
%  Description: Complete MATLAB implementation of the Ant Colony Optimized
%               Extended Kalman Filter (ACO-EKF) for State of Charge (SOC)
%               estimation of Lithium-Ion Batteries.
%
%  Modules included:
%    1. First-Order RC Battery Model (Thevenin Equivalent Circuit)
%    2. Extended Kalman Filter (EKF) with Joseph-Form Covariance Update
%    3. Ant Colony Optimization for Continuous Domains (ACOR)
%    4. Synthetic Battery Data Generator (4 Operating Profiles)
%    5. Comparative Benchmarks & Performance Metrics (MSE, MAE, MAPE, R2)
%    6. Automatic Publication-Quality Plotting
%  ========================================================================

function run_aco_ekf_simulation()
    clc; close all;
    fprintf('=================================================================\n');
    fprintf('  ACO-EKF Lithium-Ion Battery SOC Estimation (MATLAB)\n');
    fprintf('=================================================================\n\n');

    % -------------------------------------------------------------------
    % Configuration & Hyperparameters
    % -------------------------------------------------------------------
    cfg.R0  = 0.015;      % Series resistance [Ohm]
    cfg.R1  = 0.010;      % Polarization resistance [Ohm]
    cfg.C1  = 3000.0;     % Polarization capacitance [F]
    cfg.Qn  = 7200.0;     % Nominal capacity [A*s] (2 Ah)
    cfg.eta = 1.0;        % Coulombic efficiency
    cfg.dt  = 1.0;        % Sampling time [s]

    % ACO settings
    cfg.aco_k_archive = 20;   % Archive size
    cfg.aco_n_ants    = 8;    % Ants per iteration
    cfg.aco_max_iter  = 30;   % Max iterations
    cfg.aco_q         = 0.1;  % Diversity parameter
    cfg.aco_xi        = 0.85; % Locality parameter

    % ACO search bounds (log10 scale: [q_SOC, q_Vp, r_V])
    cfg.aco_bounds = [-7, -2; -7, -2; -5, -1];

    % Conventional EKF default covariances
    cfg.q_soc_default = 1e-5;
    cfg.q_vp_default  = 1e-5;
    cfg.r_v_default   = 1e-3;

    % Initial SOC uncertainty & error (+10% initial error)
    cfg.P0 = diag([0.10^2, 0.05^2]);
    cfg.soc_init_error = 0.10;

    % -------------------------------------------------------------------
    % Step 1: Initialize Battery Model
    % -------------------------------------------------------------------
    fprintf('[1/5] Building First-Order RC Battery Model...\n');
    model = create_battery_model(cfg);

    % -------------------------------------------------------------------
    % Step 2: Generate Operating Profiles
    % -------------------------------------------------------------------
    fprintf('[2/5] Generating 4 Battery Operating Profiles...\n');
    datasets = generate_profiles(model);

    % -------------------------------------------------------------------
    % Step 3: Run ACO Optimization
    % -------------------------------------------------------------------
    fprintf('[3/5] Running Ant Colony Optimization (ACOR) for Q and R...\n');
    train_ds = datasets(1:2); % Use CC Discharge and Pulse for training
    [best_params, best_mse, conv_curve] = run_acor(cfg, model, train_ds);

    q_soc_opt = best_params(1);
    q_vp_opt  = best_params(2);
    r_v_opt   = best_params(3);

    Q_opt = diag([q_soc_opt, q_vp_opt]);
    R_opt = r_v_opt;

    Q_conv = diag([cfg.q_soc_default, cfg.q_vp_default]);
    R_conv = cfg.r_v_default;

    fprintf('\n      ACO Optimization Completed!\n');
    fprintf('      Optimal Q = diag([%.3e, %.3e])\n', q_soc_opt, q_vp_opt);
    fprintf('      Optimal R = %.3e\n', r_v_opt);
    fprintf('      Best Training MSE = %.6f\n\n', best_mse);

    % -------------------------------------------------------------------
    % Step 4: Benchmarking & Performance Evaluation
    % -------------------------------------------------------------------
    fprintf('[4/5] Evaluating ACO-EKF vs Conventional EKF on all profiles...\n\n');
    
    results = struct();
    for p = 1:length(datasets)
        ds = datasets(p);
        soc_init = min(max(ds.soc0 + cfg.soc_init_error * sign(0.5 - ds.soc0), 0.02), 0.98);

        soc_aco  = run_ekf(model, Q_opt,  R_opt,  cfg.P0, ds.current_meas, ds.voltage_meas, soc_init);
        soc_conv = run_ekf(model, Q_conv, R_conv, cfg.P0, ds.current_meas, ds.voltage_meas, soc_init);

        results(p).name         = ds.name;
        results(p).time         = ds.time;
        results(p).soc_ref      = ds.soc_ref;
        results(p).soc_aco      = soc_aco;
        results(p).soc_conv     = soc_conv;
        results(p).current_true = ds.current_true;
        results(p).current_meas = ds.current_meas;
        results(p).voltage_true = ds.voltage_true;
        results(p).voltage_meas = ds.voltage_meas;
    end

    % Print Performance Metrics Table
    fprintf('---------------------------------------------------------------------------\n');
    fprintf('%-26s %-16s %10s %10s %10s %8s\n', 'Profile', 'Method', 'MSE', 'MAE (%)', 'MAPE (%)', 'R2');
    fprintf('---------------------------------------------------------------------------\n');

    for p = 1:length(results)
        m_conv = compute_metrics(results(p).soc_conv, results(p).soc_ref);
        m_aco  = compute_metrics(results(p).soc_aco,  results(p).soc_ref);

        fprintf('  %-24s %-16s %10.6f %10.4f %10.4f %8.4f\n', results(p).name, 'Conv. EKF', m_conv.MSE, m_conv.MAE, m_conv.MAPE, m_conv.R2);
        fprintf('  %-24s %-16s %10.6f %10.4f %10.4f %8.4f\n', results(p).name, 'ACO-EKF',   m_aco.MSE,  m_aco.MAE,  m_aco.MAPE,  m_aco.R2);
        fprintf('\n');
    end
    fprintf('---------------------------------------------------------------------------\n\n');

    % -------------------------------------------------------------------
    % Step 5: Render Plots
    % -------------------------------------------------------------------
    fprintf('[5/5] Generating MATLAB Visualizations...\n');
    render_plots(results, conv_curve);
    fprintf('\n=================================================================\n');
    fprintf('  Simulation Complete! Figures displayed.\n');
    fprintf('=================================================================\n');
end

% =========================================================================
% Battery Model Functions
% =========================================================================
function model = create_battery_model(cfg)
    model.R0  = cfg.R0;
    model.R1  = cfg.R1;
    model.C1  = cfg.C1;
    model.Qn  = cfg.Qn;
    model.eta = cfg.eta;
    model.dt  = cfg.dt;

    model.a = exp(-cfg.dt / (cfg.R1 * cfg.C1));
    model.b = cfg.R1 * (1.0 - model.a);

    % OCV Polynomial Coefficients (8th order)
    model.ocv_poly = [-21.7497, 82.2625, -125.3296, 99.6313, -44.5010, 11.3416, -1.5825, 1.5032, 3.0130];
    model.docv_poly = polyder(model.ocv_poly);
end

function ocv = get_ocv(model, soc)
    soc = min(max(soc, 0.0), 1.0);
    ocv = polyval(model.ocv_poly, soc);
end

function docv = get_docv(model, soc)
    soc = min(max(soc, 0.0), 1.0);
    docv = polyval(model.docv_poly, soc);
end

function x_next = state_transition(model, x, I)
    soc_next = x(1) - (model.eta * I * model.dt) / model.Qn;
    vp_next  = model.a * x(2) + model.b * I;
    x_next   = [min(max(soc_next, 0.0), 1.0); vp_next];
end

function vt = terminal_voltage(model, x, I)
    vt = get_ocv(model, x(1)) - x(2) - model.R0 * I;
end

function F = state_jacobian(model)
    F = [1.0, 0.0; 0.0, model.a];
end

function H = measurement_jacobian(model, soc)
    H = [get_docv(model, soc), -1.0];
end

% =========================================================================
% EKF Core Implementation
% =========================================================================
function soc_est = run_ekf(model, Q, R, P0, currents, voltages, soc0)
    N = length(currents);
    soc_est = zeros(N, 1);

    x_hat = [soc0; 0.0];
    P     = P0;
    I_mat = eye(2);

    for k = 1:N
        I_k  = currents(k);
        Vt_m = voltages(k);

        % Predict
        x_pred = state_transition(model, x_hat, I_k);
        F      = state_jacobian(model);
        P_pred = F * P * F' + Q;

        % Innovate
        Vt_pred = terminal_voltage(model, x_pred, I_k);
        r_k     = Vt_m - Vt_pred;

        % Gain
        H = measurement_jacobian(model, x_pred(1));
        S = H * P_pred * H' + R;
        K = (P_pred * H') / S;

        % Correct
        x_upd    = x_pred + K * r_k;
        x_upd(1) = min(max(x_upd(1), 0.0), 1.0);

        % Joseph Form Covariance Update
        IKH   = I_mat - K * H;
        P_upd = IKH * P_pred * IKH' + (K * R * K');

        x_hat = x_upd;
        P     = P_upd;

        soc_est(k) = x_hat(1);
    end
end

% =========================================================================
% Ant Colony Optimizer (ACOR)
% =========================================================================
function [best_params, best_mse, conv_curve] = run_acor(cfg, model, train_ds)
    k = cfg.aco_k_archive;
    n_ants = cfg.aco_n_ants;
    max_iter = cfg.aco_max_iter;

    bounds = cfg.aco_bounds;
    n_dim  = size(bounds, 1);

    % Initial Random Archive
    archive_log = zeros(k, n_dim);
    archive_obj = zeros(k, 1);

    for i = 1:k
        archive_log(i, :) = bounds(:, 1)' + rand(1, n_dim) .* (bounds(:, 2)' - bounds(:, 1)');
        archive_obj(i)    = eval_objective(archive_log(i, :), model, train_ds, cfg);
    end

    [archive_obj, order] = sort(archive_obj);
    archive_log          = archive_log(order, :);

    best_log = archive_log(1, :);
    best_mse = archive_obj(1);
    conv_curve = zeros(max_iter, 1);

    % Rank Weights
    ranks = (1:k)';
    w = exp(-(ranks - 1).^2 ./ (2 * cfg.aco_q^2 * k^2));
    w = w / sum(w);

    for iter = 1:max_iter
        % Compute Sigma
        sigma = zeros(k, n_dim);
        for l = 1:k
            diff_m = abs(archive_log - archive_log(l, :));
            sigma(l, :) = cfg.aco_xi / (k - 1) * sum(diff_m, 1);
        end
        sigma = max(sigma, 1e-6);

        % Ant Sampling
        new_log = zeros(n_ants, n_dim);
        new_obj = zeros(n_ants, 1);

        for ant = 1:n_ants
            % Choose Kernel (Base MATLAB weighted sampling without toolboxes)
            cum_w = cumsum(w);
            r = rand();
            idx = find(r <= cum_w, 1, 'first');
            if isempty(idx), idx = k; end
            cand = archive_log(idx, :) + randn(1, n_dim) .* sigma(idx, :);

            % Clip to Bounds
            cand = min(max(cand, bounds(:, 1)'), bounds(:, 2)');
            new_log(ant, :) = cand;
            new_obj(ant)    = eval_objective(cand, model, train_ds, cfg);
        end

        % Merge & Select Top k
        comb_log = [archive_log; new_log];
        comb_obj = [archive_obj; new_obj];

        [comb_obj, order] = sort(comb_obj);
        archive_log = comb_log(order(1:k), :);
        archive_obj = comb_obj(1:k);

        if archive_obj(1) < best_mse
            best_mse = archive_obj(1);
            best_log = archive_log(1, :);
        end

        conv_curve(iter) = best_mse;

        if mod(iter, 5) == 0 || iter == 1
            p_lin = 10.^best_log;
            fprintf('  [ACO] Iter %2d/%2d | Best MSE = %.6f | Q_soc=%.2e, R=%.2e\n', ...
                iter, max_iter, best_mse, p_lin(1), p_lin(3));
        end
    end

    best_params = 10.^best_log;
end

function mse_avg = eval_objective(candidate_log, model, train_ds, cfg)
    params = 10.^candidate_log;
    Q = diag([params(1), params(2)]);
    R = params(3);

    mses = zeros(length(train_ds), 1);
    for p = 1:length(train_ds)
        ds = train_ds(p);
        soc_init = min(max(ds.soc0 + cfg.soc_init_error * sign(0.5 - ds.soc0), 0.02), 0.98);
        soc_est = run_ekf(model, Q, R, cfg.P0, ds.current_meas, ds.voltage_meas, soc_init);
        mses(p) = mean((soc_est - ds.soc_ref).^2);
    end
    mse_avg = mean(mses);
end

% =========================================================================
% Synthetic Data Generator
% =========================================================================
function datasets = generate_profiles(model)
    % 1. CC Discharge
    N1 = 1800; I1 = 0.5 * (model.Qn/3600) * ones(N1, 1);
    sim1 = sim_profile(model, I1, 0.95);
    ds1.name = 'CC Discharge 0.5C'; ds1.time = (0:N1-1)'*model.dt;
    ds1.current_true = I1; ds1.current_meas = I1 + 0.005*(model.Qn/3600)*randn(N1, 1);
    ds1.voltage_true = sim1.vt; ds1.voltage_meas = sim1.vt + 0.005*randn(N1, 1);
    ds1.soc_ref = sim1.soc; ds1.soc0 = 0.95;

    % 2. Pulse Discharge
    cycle = [ones(30,1)*(model.Qn/3600); zeros(30,1)];
    I2 = repmat(cycle, 20, 1); N2 = length(I2);
    sim2 = sim_profile(model, I2, 0.90);
    ds2.name = 'Pulse Discharge 1C'; ds2.time = (0:N2-1)'*model.dt;
    ds2.current_true = I2; ds2.current_meas = I2 + 0.008*(model.Qn/3600)*randn(N2, 1);
    ds2.voltage_true = sim2.vt; ds2.voltage_meas = sim2.vt + 0.005*randn(N2, 1);
    ds2.soc_ref = sim2.soc; ds2.soc0 = 0.90;

    % 3. Dynamic Drive Cycle
    N3 = 1800; I3 = zeros(N3, 1); k = 1;
    while k <= N3
        len = randi([15, 90]); end_k = min(k + len - 1, N3);
        type = rand();
        if type < 0.3, amp = (0.8 + 0.7*rand())*(model.Qn/3600);
        elseif type < 0.75, amp = (0.2 + 0.5*rand())*(model.Qn/3600);
        else, amp = 0.0; end
        I3(k:end_k) = amp; k = end_k + 1;
    end
    sim3 = sim_profile(model, I3, 0.85);
    ds3.name = 'Dynamic Drive Cycle'; ds3.time = (0:N3-1)'*model.dt;
    ds3.current_true = I3; ds3.current_meas = I3 + 0.01*(model.Qn/3600)*randn(N3, 1);
    ds3.voltage_true = sim3.vt; ds3.voltage_meas = sim3.vt + 0.005*randn(N3, 1);
    ds3.soc_ref = sim3.soc; ds3.soc0 = 0.85;

    % 4. CCCV Charge
    I_cc = -0.5 * (model.Qn/3600); soc = 0.20; vp = 0.0; x = [soc; vp];
    curr4 = []; vt4 = []; soc4 = [];
    for t = 1:3000
        vt = terminal_voltage(model, x, I_cc);
        curr4 = [curr4; I_cc]; vt4 = [vt4; vt]; soc4 = [soc4; x(1)];
        if vt >= 4.18 || x(1) >= 0.99, break; end
        x = state_transition(model, x, I_cc);
    end
    taper = exp(-(0:1799)'/300.0);
    for t = 1:length(taper)
        I_cv = I_cc * taper(t); vt = terminal_voltage(model, x, I_cv);
        curr4 = [curr4; I_cv]; vt4 = [vt4; vt]; soc4 = [soc4; x(1)];
        if abs(I_cv) < 0.02*(model.Qn/3600), break; end
        x = state_transition(model, x, I_cv);
    end
    N4 = length(curr4);
    ds4.name = 'CCCV Charge 0.5C'; ds4.time = (0:N4-1)'*model.dt;
    ds4.current_true = curr4; ds4.current_meas = curr4 + 0.005*(model.Qn/3600)*randn(N4, 1);
    ds4.voltage_true = vt4; ds4.voltage_meas = vt4 + 0.005*randn(N4, 1);
    ds4.soc_ref = soc4; ds4.soc0 = 0.20;

    datasets = [ds1, ds2, ds3, ds4];
end

function sim = sim_profile(model, I, soc0)
    N = length(I); soc = zeros(N, 1); vp = zeros(N, 1); vt = zeros(N, 1);
    x = [soc0; 0.0];
    for k = 1:N
        vt(k) = terminal_voltage(model, x, I(k));
        x = state_transition(model, x, I(k));
        soc(k) = x(1); vp(k) = x(2);
    end
    sim.soc = soc; sim.vp = vp; sim.vt = vt;
end

% =========================================================================
% Performance Metrics
% =========================================================================
function m = compute_metrics(soc_est, soc_ref)
    err = soc_est - soc_ref;
    m.MSE = mean(err.^2);
    m.MAE = mean(abs(err)) * 100;
    m.MAPE = mean(abs(err(soc_ref > 0.01) ./ soc_ref(soc_ref > 0.01))) * 100;
    ss_res = sum(err.^2); ss_tot = sum((soc_ref - mean(soc_ref)).^2);
    m.R2 = 1.0 - (ss_res / (ss_tot + 1e-12));
end

% =========================================================================
% Plotting Functions
% =========================================================================
function render_plots(results, conv_curve)
    % 1. SOC Comparison Plot
    figure('Name', 'SOC Estimation Comparison', 'NumberTitle', 'off', 'Color', [1 1 1]);
    for p = 1:length(results)
        subplot(2, 2, p);
        t_min = results(p).time / 60;
        plot(t_min, results(p).soc_ref*100, 'b-', 'LineWidth', 1.8); hold on;
        plot(t_min, results(p).soc_conv*100, 'r--', 'LineWidth', 1.2);
        plot(t_min, results(p).soc_aco*100, 'g-.', 'LineWidth', 1.5);
        grid on; xlabel('Time (min)'); ylabel('SOC (%)');
        title(results(p).name, 'FontWeight', 'bold');
        legend('Reference SOC', 'Conv. EKF', 'ACO-EKF', 'Location', 'best');
    end
    sgtitle('State of Charge (SOC) Estimation: ACO-EKF vs Conv. EKF');

    % 2. Absolute Error Plot
    figure('Name', 'Absolute SOC Error', 'NumberTitle', 'off', 'Color', [1 1 1]);
    for p = 1:length(results)
        subplot(2, 2, p);
        t_min = results(p).time / 60;
        err_conv = abs(results(p).soc_conv - results(p).soc_ref)*100;
        err_aco  = abs(results(p).soc_aco  - results(p).soc_ref)*100;
        plot(t_min, err_conv, 'r-', 'LineWidth', 1.2); hold on;
        plot(t_min, err_aco,  'g-', 'LineWidth', 1.5);
        grid on; xlabel('Time (min)'); ylabel('|Error| (%)');
        title(results(p).name, 'FontWeight', 'bold');
        legend('Conv. EKF Error', 'ACO-EKF Error', 'Location', 'best');
    end
    sgtitle('Absolute SOC Estimation Error (%)');

    % 3. ACO Convergence Curve
    figure('Name', 'ACO Convergence', 'NumberTitle', 'off', 'Color', [1 1 1]);
    semilogy(1:length(conv_curve), conv_curve, '-o', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.8);
    grid on; xlabel('ACO Iteration'); ylabel('Best Training MSE');
    title('Ant Colony Optimization (ACOR) Convergence Curve', 'FontWeight', 'bold');
end
