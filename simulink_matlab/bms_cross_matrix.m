function tbl = bms_cross_matrix(mode)
%BMS_CROSS_MATRIX Generate cross-feeding test matrix across estimators & scenarios.
%   tbl = bms_cross_matrix() runs full 12-scenario cross-feeding matrix.
%   tbl = bms_cross_matrix('quick') runs 4 representative scenarios.
%
%   Evaluates:
%     1. Baseline ACO-EKF (Mode 0)
%     2. SOTA Adaptive EKF (Sage-Husa R, Mode 2)
%     3. Online Capacity (SOH) EKF (Mode 10)
%   Across:
%     - Simulated plant (known truth, faults, Monte Carlo)
%     - NASA replay with and without faults injected (BMS_InjectFaultsOnReplay)
%
%   Metrics per (estimator, scenario):
%     - MAE (%) [excluding first 120 s]
%     - RMSE (%) [excluding first 120 s]
%     - 95th Percentile Error (%) [excluding first 120 s]
%     - Max Error (%) [excluding first 120 s]
%     - Convergence Time (s) [time to permanently settle within 1.0% error]
%     - Fault-flag Detection Latency (s) [seconds from fault onset t=900s to detection]
%
%   Outputs:
%     - simulink_matlab/results/cross_matrix.csv
%     - simulink_matlab/results/fig_cross_matrix.png
%     - figures/17_simulink_cross_matrix.png

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    addpath(fullfile(here, 'lib'));
    resdir = fullfile(here, 'results');
    if ~isfolder(resdir),  mkdir(resdir);  end
    figdir = fullfile(here, '..', 'figures');
    if ~isfolder(figdir),  mkdir(figdir);  end

    if nargin < 1 || isempty(mode)
        mode = 'full';
    end

    bms_setup;

    % Define cross-feeding scenario suite
    if strcmpi(mode, 'quick')
        sc_defs = { ...
            struct('name', 'sim_clean_udds',    'type', 'synthetic', 'prof', 'udds',    'fault', 0, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_fault2_V_spike','type', 'synthetic', 'prof', 'fault2',  'fault', 2, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_clean',  'type', 'nasa',      'cell', 'B0005_1', 'fault', 0, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_fault2', 'type', 'nasa',      'cell', 'B0005_1', 'fault', 2, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 1) ...
        };
    else
        sc_defs = { ...
            struct('name', 'sim_clean_udds',      'type', 'synthetic', 'prof', 'udds',      'fault', 0, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_pulse',           'type', 'synthetic', 'prof', 'pulse',     'fault', 0, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_fault1_I_offset', 'type', 'synthetic', 'prof', 'fault1',    'fault', 1, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_fault2_V_spike',  'type', 'synthetic', 'prof', 'fault2',    'fault', 2, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_fault3_T_open',   'type', 'synthetic', 'prof', 'fault3',    'fault', 3, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_monte_carlo_42',  'type', 'synthetic', 'prof', 'udds',      'fault', 0, 'seed', 42, 'init_err', -0.20, 'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_clean',    'type', 'nasa',      'cell', 'B0005_1',   'fault', 0, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_fault1',   'type', 'nasa',      'cell', 'B0005_1',   'fault', 1, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 1), ...
            struct('name', 'nasa_B0005_fault2',   'type', 'nasa',      'cell', 'B0005_1',   'fault', 2, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 1), ...
            struct('name', 'nasa_B0005_fault3',   'type', 'nasa',      'cell', 'B0005_1',   'fault', 3, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 1), ...
            struct('name', 'nasa_B0007_clean',    'type', 'nasa',      'cell', 'B0007_1',   'fault', 0, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_aged_120', 'type', 'nasa',      'cell', 'B0005_120', 'fault', 0, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 0) ...
        };
    end

    n_sc = numel(sc_defs);
    fprintf('=========================================================================\n');
    fprintf(' BMS Cross-Feeding Test Matrix: Running %d Scenarios x 3 Estimators\n', n_sc);
    fprintf(' Output: %s\n', fullfile(resdir, 'cross_matrix.csv'));
    fprintf('=========================================================================\n\n');

    rows = {};
    estimators = {'Baseline_EKF', 'Adaptive_EKF', 'Capacity_SOH_EKF'};

    for s_idx = 1:n_sc
        sd = sc_defs{s_idx};
        fprintf('[%2d/%2d] Simulating: %-22s (source: %s, fault: %d) ...\n', ...
            s_idx, n_sc, sd.name, sd.type, sd.fault);

        if strcmp(sd.type, 'synthetic')
            sc = bms_scenario(sd.prof, 'seed', sd.seed, 'init_err', sd.init_err, 'fault_mode', sd.fault);
            t_end = numel(sc.I) - 1;
            in = Simulink.SimulationInput('bms_unified');
            in = in.setModelParameter('StopTime', num2str(t_end));
            in = in.setVariable('BMS_StopTime', t_end);
            in = in.setVariable('BMS_Source', 1);
            in = in.setVariable('Drive_Profile', [(0:t_end)', sc.I(:)]);
            in = in.setVariable('BMS_SOC0', sc.soc0);
            in = in.setVariable('BMS_SOC_Guess', sc.soc0 + sc.init_err);
            in = in.setVariable('BMS_Seed', sc.seed);
            in = in.setVariable('BMS_FaultMode', sc.fault_mode);
            in = in.setVariable('BMS_InjectFaultsOnReplay', 0);
        else
            sc = bms_scenario(sd.cell, 'seed', sd.seed, 'init_err', sd.init_err, 'fault_mode', sd.fault);
            t_end = numel(sc.I) - 1;
            in = Simulink.SimulationInput('bms_unified');
            in = in.setModelParameter('StopTime', num2str(t_end));
            in = in.setVariable('BMS_StopTime', t_end);
            in = in.setVariable('BMS_Source', 2);
            in = in.setVariable('NASA_Replay', [(0:t_end)', sc.I(:), sc.V_ref(:), sc.T(:), sc.soc_ref(:)]);
            in = in.setVariable('NASA_SOC_Guess', sc.soc_ref(1) + sc.init_err);
            in = in.setVariable('NASA_StopTime', t_end);
            in = in.setVariable('NASA_CapRatio', 1.0);
            in = in.setVariable('BMS_SOC_Guess', sc.soc_ref(1) + sc.init_err);
            in = in.setVariable('BMS_FaultMode', sc.fault_mode);
            in = in.setVariable('BMS_InjectFaultsOnReplay', sd.inject_fault_replay);
        end

        out = sim(in);
        d = out.bms_log;

        N = size(d, 1);
        time_s  = (0:N - 1)';
        soc_ref = d(:, 1);
        est_series = {d(:, 2), d(:, 3), d(:, 12)};
        flags   = d(:, 9);

        skip = min(120, floor(N / 4));

        % Compute fault detection latency for the scenario
        fault_active = (sd.fault > 0) && (strcmp(sd.type, 'synthetic') || sd.inject_fault_replay == 1);
        t_fault_start = 900.0;
        if fault_active
            idx_flag = find(flags(time_s >= t_fault_start) > 0, 1);
            if ~isempty(idx_flag)
                fault_latency = idx_flag - 1; % seconds from fault onset
            else
                fault_latency = NaN; % silent fault (e.g. current offset) or undetected
            end
        else
            fault_latency = NaN; % no fault injected
        end

        for e_idx = 1:numel(estimators)
            est_name = estimators{e_idx};
            soc_est = est_series{e_idx};

            err = soc_est(:) - soc_ref(:);
            err_post = err(skip+1:end);

            mae_val  = mean(abs(err_post));
            rmse_val = sqrt(mean(err_post.^2));
            p95_val  = prctile(abs(err_post), 95);
            max_val  = max(abs(err_post));

            % Convergence settling time to |err| <= 1.0%
            tol1 = 1.0;
            idx_out1 = find(abs(err) > tol1);
            if isempty(idx_out1)
                conv_time = 0.0;
            else
                k_settle = idx_out1(end) + 1;
                if k_settle <= N
                    conv_time = time_s(k_settle);
                else
                    % Settle to 2.0% if 1.0% is not permanently maintained
                    tol2 = 2.0;
                    idx_out2 = find(abs(err) > tol2);
                    if isempty(idx_out2)
                        conv_time = 0.0;
                    else
                        k_settle2 = idx_out2(end) + 1;
                        if k_settle2 <= N
                            conv_time = time_s(k_settle2);
                        else
                            conv_time = NaN;
                        end
                    end
                end
            end

            rows(end + 1, :) = { ...
                est_name, sd.name, sd.type, sd.fault, ...
                mae_val, rmse_val, p95_val, max_val, conv_time, fault_latency ...
            }; %#ok<AGROW>
        end
    end

    tbl = cell2table(rows, 'VariableNames', { ...
        'estimator', 'scenario', 'source', 'fault_mode', ...
        'mae_pct', 'rmse_pct', 'p95_pct', 'max_err_pct', 'conv_time_s', 'fault_latency_s' ...
    });

    % Export CSV
    csv_file = fullfile(resdir, 'cross_matrix.csv');
    writetable(tbl, csv_file);
    fprintf('\n[OK] Cross-feeding matrix exported to: %s\n', csv_file);

    % Generate Summary Figure
    generate_cross_figure(tbl, resdir, figdir);
end

%% ======================================================================
function generate_cross_figure(tbl, resdir, figdir)
    scenarios = unique(tbl.scenario, 'stable');
    n_sc = numel(scenarios);

    mae_base = tbl.mae_pct(strcmp(tbl.estimator, 'Baseline_EKF'));
    mae_adap = tbl.mae_pct(strcmp(tbl.estimator, 'Adaptive_EKF'));
    mae_cap  = tbl.mae_pct(strcmp(tbl.estimator, 'Capacity_SOH_EKF'));

    max_base = tbl.max_err_pct(strcmp(tbl.estimator, 'Baseline_EKF'));
    max_adap = tbl.max_err_pct(strcmp(tbl.estimator, 'Adaptive_EKF'));
    max_cap  = tbl.max_err_pct(strcmp(tbl.estimator, 'Capacity_SOH_EKF'));

    f = figure('Visible', 'off', 'Position', [80 80 1200 850], 'Color', 'w');

    % Panel 1: MAE Comparison
    subplot(2, 2, 1);
    b1 = bar(1:n_sc, [mae_base, mae_adap, mae_cap], 'grouped');
    b1(1).FaceColor = [0.20 0.45 0.75];
    b1(2).FaceColor = [0.85 0.30 0.20];
    b1(3).FaceColor = [0.25 0.65 0.35];
    grid on;  box on;
    set(gca, 'XTick', 1:n_sc, 'XTickLabel', scenarios, 'XTickLabelRotation', 45, 'FontSize', 8);
    ylabel('Mean Absolute Error (%)', 'FontWeight', 'bold');
    title('Cross-Feeding: SOC Mean Absolute Error (MAE)', 'FontSize', 10, 'FontWeight', 'bold');
    legend({'Baseline EKF (Mode 0)', 'Adaptive EKF (Mode 2)', 'Capacity SOH EKF (Mode 10)'}, ...
        'Location', 'northwest', 'FontSize', 7);

    % Panel 2: Max Error Comparison
    subplot(2, 2, 2);
    b2 = bar(1:n_sc, [max_base, max_adap, max_cap], 'grouped');
    b2(1).FaceColor = [0.20 0.45 0.75];
    b2(2).FaceColor = [0.85 0.30 0.20];
    b2(3).FaceColor = [0.25 0.65 0.35];
    grid on;  box on;
    set(gca, 'XTick', 1:n_sc, 'XTickLabel', scenarios, 'XTickLabelRotation', 45, 'FontSize', 8);
    ylabel('Worst-Case Error (%)', 'FontWeight', 'bold');
    title('Cross-Feeding: Worst-Case Max Error', 'FontSize', 10, 'FontWeight', 'bold');

    % Panel 3: Convergence Time
    subplot(2, 2, 3);
    conv_base = tbl.conv_time_s(strcmp(tbl.estimator, 'Baseline_EKF'));
    conv_adap = tbl.conv_time_s(strcmp(tbl.estimator, 'Adaptive_EKF'));
    conv_cap  = tbl.conv_time_s(strcmp(tbl.estimator, 'Capacity_SOH_EKF'));
    % replace NaN with 0 for plotting
    conv_base_plot = conv_base;  conv_base_plot(isnan(conv_base_plot)) = 0;
    conv_adap_plot = conv_adap;  conv_adap_plot(isnan(conv_adap_plot)) = 0;
    conv_cap_plot  = conv_cap;   conv_cap_plot(isnan(conv_cap_plot))   = 0;
    b3 = bar(1:n_sc, [conv_base_plot, conv_adap_plot, conv_cap_plot], 'grouped');
    b3(1).FaceColor = [0.20 0.45 0.75];
    b3(2).FaceColor = [0.85 0.30 0.20];
    b3(3).FaceColor = [0.25 0.65 0.35];
    grid on;  box on;
    set(gca, 'XTick', 1:n_sc, 'XTickLabel', scenarios, 'XTickLabelRotation', 45, 'FontSize', 8);
    ylabel('Settling Time (s)', 'FontWeight', 'bold');
    title('Convergence Settling Time (to \leq 1.0% error)', 'FontSize', 10, 'FontWeight', 'bold');

    % Panel 4: Fault Detection Latency
    subplot(2, 2, 4);
    lat_vals = tbl.fault_latency_s(strcmp(tbl.estimator, 'Adaptive_EKF'));
    fault_sc_mask = ~isnan(lat_vals);
    bar_vals = zeros(n_sc, 1);
    bar_vals(fault_sc_mask) = lat_vals(fault_sc_mask);
    b4 = bar(1:n_sc, bar_vals, 'FaceColor', [0.70 0.20 0.80]);
    grid on;  box on;
    set(gca, 'XTick', 1:n_sc, 'XTickLabel', scenarios, 'XTickLabelRotation', 45, 'FontSize', 8);
    ylabel('Detection Latency (s)', 'FontWeight', 'bold');
    title('Fault Detection Latency from Onset (t = 900 s)', 'FontSize', 10, 'FontWeight', 'bold');

    sgtitle('BMS Unified Cross-Feeding Performance Matrix (Phase 2)', 'FontSize', 12, 'FontWeight', 'bold');

    out_fig1 = fullfile(resdir, 'fig_cross_matrix.png');
    exportgraphics(f, out_fig1, 'Resolution', 150);
    out_fig2 = fullfile(figdir, '17_simulink_cross_matrix.png');
    exportgraphics(f, out_fig2, 'Resolution', 150);
    close(f);
    fprintf('[OK] Saved cross-feeding figures:\n  -> %s\n  -> %s\n', out_fig1, out_fig2);
end
