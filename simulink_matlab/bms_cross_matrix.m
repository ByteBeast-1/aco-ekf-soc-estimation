function tbl = bms_cross_matrix(mode)
%BMS_CROSS_MATRIX Generate cross-feeding test matrix across estimators & scenarios.
%   tbl = bms_cross_matrix() runs full 12-scenario cross-feeding matrix.
%   tbl = bms_cross_matrix('quick') runs 4 representative scenarios.
%
%   Evaluates:
%     1. Baseline ACO-EKF (Mode 0)
%     2. Adaptive EKF (Sage-Husa) (Mode 2)
%     3. Online Capacity (SOH) EKF (Mode 10)
%   Across:
%     - Simulated plant (known truth, sensor faults, 40-run Monte Carlo ensemble)
%     - NASA replay with and without faults injected (BMS_InjectFaultsOnReplay)
%
%   Metrics per (estimator, scenario):
%     - MAE (%) [excluding first 120 s]
%     - RMSE (%) [excluding first 120 s]
%     - 95th Percentile Error (%) [excluding first 120 s]
%     - Max Error (%) [excluding first 120 s]
%     - t_enter_1pct_s: time (s) to first enter |err| <= 1.0% and remain <= 1.0%
%       for at least 60 consecutive seconds. If never satisfied, NaN (no fallback).
%     - frac_err_gt_1pct: fraction of post-120s samples (t > 120 s) with |err| > 1.0% (0.0 to 1.0).
%     - spike_flag_rate: fraction of periodic spike faults flagged (Fault Mode 2, spikes every 25 samples).
%     - first_flag_delay_s: delay in seconds from the first spike (Mode 2) or fault onset (Mode 3)
%       until the first diagnostic flag is raised.
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
            struct('name', 'sim_clean_udds',    'type', 'synthetic',   'prof', 'udds',     'fault', 0, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_fault2_V_spike','type', 'synthetic',   'prof', 'fault2',   'fault', 2, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_clean',  'type', 'nasa',        'cell', 'B0005_1',  'fault', 0, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_fault2', 'type', 'nasa',        'cell', 'B0005_1',  'fault', 2, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 1) ...
        };
    else
        sc_defs = { ...
            struct('name', 'sim_clean_udds',      'type', 'synthetic',   'prof', 'udds',      'fault', 0, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_pulse',           'type', 'synthetic',   'prof', 'pulse',     'fault', 0, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_fault1_I_offset', 'type', 'synthetic',   'prof', 'fault1',    'fault', 1, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_fault2_V_spike',  'type', 'synthetic',   'prof', 'fault2',    'fault', 2, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_fault3_T_open',   'type', 'synthetic',   'prof', 'fault3',    'fault', 3, 'seed', 1,  'init_err', -0.10, 'inject_fault_replay', 0), ...
            struct('name', 'sim_monte_carlo',     'type', 'monte_carlo', 'prof', 'ensemble',  'fault', 0, 'seed', 0,  'init_err', 0,     'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_clean',    'type', 'nasa',        'cell', 'B0005_1',   'fault', 0, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_fault1',   'type', 'nasa',        'cell', 'B0005_1',   'fault', 1, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 1), ...
            struct('name', 'nasa_B0005_fault2',   'type', 'nasa',        'cell', 'B0005_1',   'fault', 2, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 1), ...
            struct('name', 'nasa_B0005_fault3',   'type', 'nasa',        'cell', 'B0005_1',   'fault', 3, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 1), ...
            struct('name', 'nasa_B0007_clean',    'type', 'nasa',        'cell', 'B0007_1',   'fault', 0, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 0), ...
            struct('name', 'nasa_B0005_aged_120', 'type', 'nasa',        'cell', 'B0005_120', 'fault', 0, 'seed', 1,  'init_err', -0.15, 'inject_fault_replay', 0) ...
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

        if strcmp(sd.type, 'monte_carlo')
            % -----------------------------------------------------------------
            % Full Monte Carlo Ensemble (40 randomized runs)
            % -----------------------------------------------------------------
            n_mc = 40;
            mc_profs = {'udds', 'pulse', 'cc', 'cccv'};
            rng(2024, 'twister');

            mc_maes = zeros(n_mc, 3);
            mc_rmses = zeros(n_mc, 3);
            mc_p95s = zeros(n_mc, 3);
            mc_maxs = zeros(n_mc, 3);
            mc_tenters = zeros(n_mc, 3);
            mc_fracs = zeros(n_mc, 3);

            for mc_i = 1:n_mc
                p_name = mc_profs{mod(mc_i - 1, 4) + 1};
                [mc_I, mc_soc0] = bms_profile(p_name);
                mc_I = mc_I(1:min(numel(mc_I), 1800));
                t_end = numel(mc_I) - 1;
                ie = (2 * rand() - 1) * 0.20;
                if mc_soc0 + ie < 0.02 || mc_soc0 + ie > 0.98, ie = -ie; end

                in_mc = Simulink.SimulationInput('bms_unified');
                in_mc = in_mc.setModelParameter('StopTime', num2str(t_end));
                in_mc = in_mc.setVariable('BMS_StopTime', t_end);
                in_mc = in_mc.setVariable('BMS_Source', 1);
                in_mc = in_mc.setVariable('Drive_Profile', [(0:t_end)', mc_I(:)]);
                in_mc = in_mc.setVariable('BMS_SOC0', mc_soc0);
                in_mc = in_mc.setVariable('BMS_SOC_Guess', mc_soc0 + ie);
                in_mc = in_mc.setVariable('BMS_Seed', 5000 + mc_i);
                in_mc = in_mc.setVariable('BMS_FaultMode', 0);
                in_mc = in_mc.setVariable('BMS_InjectFaultsOnReplay', 0);

                out_mc = sim(in_mc);
                d_mc = out_mc.bms_log;
                N_mc = size(d_mc, 1);
                t_mc = (0:N_mc - 1)';
                ref_mc = d_mc(:, 1);
                skip_mc = min(120, floor(N_mc / 4));

                col_map = [2, 3, 12];
                for e_i = 1:3
                    e_est = d_mc(:, col_map(e_i));
                    e_err = e_est - ref_mc;
                    e_post = e_err(skip_mc+1:end);

                    mc_maes(mc_i, e_i)   = mean(abs(e_post));
                    mc_rmses(mc_i, e_i)  = sqrt(mean(e_post.^2));
                    mc_p95s(mc_i, e_i)   = prctile(abs(e_post), 95);
                    mc_maxs(mc_i, e_i)   = max(abs(e_post));
                    mc_tenters(mc_i, e_i)= compute_conv_stay(e_err, t_mc, 1.0, 60);
                    mc_fracs(mc_i, e_i)  = compute_frac_err_gt_1pct(e_err, t_mc);
                end
            end

            for e_idx = 1:numel(estimators)
                est_name = estimators{e_idx};
                mae_val   = mean(mc_maes(:, e_idx));
                rmse_val  = sqrt(mean(mc_rmses(:, e_idx).^2));
                p95_val   = mean(mc_p95s(:, e_idx));
                max_val   = max(mc_maxs(:, e_idx));
                valid_t   = mc_tenters(~isnan(mc_tenters(:, e_idx)), e_idx);
                if isempty(valid_t)
                    t_enter_val = NaN;
                else
                    t_enter_val = round(mean(valid_t));
                end
                frac_val  = mean(mc_fracs(:, e_idx));

                rows(end + 1, :) = { ...
                    est_name, sd.name, 'synthetic', 0, ...
                    mae_val, rmse_val, p95_val, max_val, ...
                    t_enter_val, frac_val, NaN, NaN ...
                }; %#ok<AGROW>
            end
            continue;
        end

        % ---------------------------------------------------------------------
        % Standard Scenarios (Synthetic & NASA Replay)
        % ---------------------------------------------------------------------
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

        % Compute fault metrics (per-spike detection & first-flag delay)
        fault_active = (sd.fault > 0) && (strcmp(sd.type, 'synthetic') || sd.inject_fault_replay == 1);
        [spike_flag_rate, first_flag_delay_s] = compute_fault_metrics(sd.fault, fault_active, flags, time_s);

        for e_idx = 1:numel(estimators)
            est_name = estimators{e_idx};
            soc_est = est_series{e_idx};

            err = soc_est(:) - soc_ref(:);
            err_post = err(skip+1:end);

            mae_val  = mean(abs(err_post));
            rmse_val = sqrt(mean(err_post.^2));
            p95_val  = prctile(abs(err_post), 95);
            max_val  = max(abs(err_post));

            % Metric 1: Time to first enter |err| <= 1.0% and stay for 60 s (no fallback)
            t_enter_val = compute_conv_stay(err, time_s, 1.0, 60);

            % Metric 2: Fraction of samples after 120 s with |err| > 1.0%
            frac_val = compute_frac_err_gt_1pct(err, time_s);

            rows(end + 1, :) = { ...
                est_name, sd.name, sd.type, sd.fault, ...
                mae_val, rmse_val, p95_val, max_val, ...
                t_enter_val, frac_val, spike_flag_rate, first_flag_delay_s ...
            }; %#ok<AGROW>
        end
    end

    tbl = cell2table(rows, 'VariableNames', { ...
        'estimator', 'scenario', 'source', 'fault_mode', ...
        'mae_pct', 'rmse_pct', 'p95_pct', 'max_err_pct', ...
        't_enter_1pct_s', 'frac_err_gt_1pct', 'spike_flag_rate', 'first_flag_delay_s' ...
    });

    % Export CSV
    csv_file = fullfile(resdir, 'cross_matrix.csv');
    writetable(tbl, csv_file);
    fprintf('\n[OK] Cross-feeding matrix exported to: %s\n', csv_file);

    % Generate Summary Figure
    generate_cross_figure(tbl, resdir, figdir);
end

%% ======================================================================
%  Metric Computation Helpers
%% ======================================================================
function t_enter = compute_conv_stay(err, time_s, tol, win_s)
%COMPUTE_CONV_STAY Time (s) to first enter |err| <= tol and stay for win_s consecutive seconds.
%   If never satisfied, returns NaN. (No fallback to looser tolerance).
    if nargin < 3, tol = 1.0; end
    if nargin < 4, win_s = 60; end
    N = numel(err);
    abs_err = abs(err);
    t_enter = NaN;
    for k = 1:(N - win_s + 1)
        if all(abs_err(k : k + win_s - 1) <= tol)
            t_enter = time_s(k);
            return;
        end
    end
end

function frac = compute_frac_err_gt_1pct(err, time_s)
%COMPUTE_FRAC_ERR_GT_1PCT Fraction of samples strictly after t = 120 s with |err| > 1.0%.
    idx_post = find(time_s > 120);
    if isempty(idx_post)
        frac = 0.0;
    else
        frac = mean(abs(err(idx_post)) > 1.0);
    end
end

function [spike_flag_rate, first_flag_delay_s] = compute_fault_metrics(fault_mode, fault_active, flags, time_s)
%COMPUTE_FAULT_METRICS Computes per-spike detection rate and first-flag delay.
%   Spikes occur every 25 samples between sample 900 (t=899s) and 1200 (t=1199s).
    if ~fault_active || fault_mode == 0
        spike_flag_rate = NaN;
        first_flag_delay_s = NaN;
        return;
    end

    if fault_mode == 1
        % Current bias (+0.25 A): silent bias drift, not a spike fault
        spike_flag_rate = NaN;
        first_flag_delay_s = NaN;
    elseif fault_mode == 2
        % Voltage glitches (+-0.5 V): periodic spikes at mod(k, 25) == 0 in k in [900, 1200]
        spike_samples = 900:25:1200; % 13 spike samples
        N_spikes = numel(spike_samples);
        flagged_spikes = sum(flags(spike_samples) > 0);
        spike_flag_rate = flagged_spikes / N_spikes;

        first_spike_k = spike_samples(1);
        flag_idx = find(flags(first_spike_k:end) > 0, 1);
        if ~isempty(flag_idx)
            first_flag_k = first_spike_k + flag_idx - 1;
            first_flag_delay_s = time_s(first_flag_k) - time_s(first_spike_k);
        else
            first_flag_delay_s = NaN;
        end
    elseif fault_mode == 3
        % Thermistor open: step fault at sample 900 (t=899s)
        spike_flag_rate = NaN; % step fault, not periodic spikes
        fault_start_k = 900;
        flag_idx = find(flags(fault_start_k:end) > 0, 1);
        if ~isempty(flag_idx)
            first_flag_k = fault_start_k + flag_idx - 1;
            first_flag_delay_s = time_s(first_flag_k) - time_s(fault_start_k);
        else
            first_flag_delay_s = NaN;
        end
    else
        spike_flag_rate = NaN;
        first_flag_delay_s = NaN;
    end
end

%% ======================================================================
%  Visualization Generator
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
    legend({'Baseline EKF (Mode 0)', 'Adaptive EKF (Sage-Husa, Mode 2)', 'Capacity SOH EKF (Mode 10)'}, ...
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

    % Panel 3: Post-120s Samples with |Error| > 1.0% (%)
    subplot(2, 2, 3);
    frac_base = tbl.frac_err_gt_1pct(strcmp(tbl.estimator, 'Baseline_EKF')) * 100;
    frac_adap = tbl.frac_err_gt_1pct(strcmp(tbl.estimator, 'Adaptive_EKF')) * 100;
    frac_cap  = tbl.frac_err_gt_1pct(strcmp(tbl.estimator, 'Capacity_SOH_EKF')) * 100;
    b3 = bar(1:n_sc, [frac_base, frac_adap, frac_cap], 'grouped');
    b3(1).FaceColor = [0.20 0.45 0.75];
    b3(2).FaceColor = [0.85 0.30 0.20];
    b3(3).FaceColor = [0.25 0.65 0.35];
    grid on;  box on;
    set(gca, 'XTick', 1:n_sc, 'XTickLabel', scenarios, 'XTickLabelRotation', 45, 'FontSize', 8);
    ylabel('Samples with |err| > 1% (%)', 'FontWeight', 'bold');
    title('Post-120 s Error Distribution (|err| > 1.0%)', 'FontSize', 10, 'FontWeight', 'bold');

    % Panel 4: Spike Flag Detection & First-Spike Delay
    subplot(2, 2, 4);
    rate_vals = tbl.spike_flag_rate(strcmp(tbl.estimator, 'Baseline_EKF')) * 100;
    delay_vals = tbl.first_flag_delay_s(strcmp(tbl.estimator, 'Baseline_EKF'));
    rate_plot = rate_vals;  rate_plot(isnan(rate_plot)) = 0;
    delay_plot = delay_vals; delay_plot(isnan(delay_plot)) = 0;
    b4 = bar(1:n_sc, [rate_plot, delay_plot], 'grouped');
    b4(1).FaceColor = [0.70 0.20 0.80];
    b4(2).FaceColor = [0.95 0.60 0.15];
    grid on;  box on;
    set(gca, 'XTick', 1:n_sc, 'XTickLabel', scenarios, 'XTickLabelRotation', 45, 'FontSize', 8);
    ylabel('Rate (%) / Delay (s)', 'FontWeight', 'bold');
    title('Fault Diagnostic: Spike Detection Rate & First-Spike Delay', 'FontSize', 10, 'FontWeight', 'bold');
    legend({'Spike Detection Rate (%)', 'First Flag Delay (s)'}, 'Location', 'northeast', 'FontSize', 7);

    sgtitle('BMS Cross-Feeding Test Matrix (Simulated Plant vs. NASA Replay with Faults)', 'FontSize', 12, 'FontWeight', 'bold');

    out_fig1 = fullfile(resdir, 'fig_cross_matrix.png');
    exportgraphics(f, out_fig1, 'Resolution', 150);
    out_fig2 = fullfile(figdir, '17_simulink_cross_matrix.png');
    exportgraphics(f, out_fig2, 'Resolution', 150);
    close(f);
    fprintf('[OK] Saved cross-feeding figures:\n  -> %s\n  -> %s\n', out_fig1, out_fig2);
end
