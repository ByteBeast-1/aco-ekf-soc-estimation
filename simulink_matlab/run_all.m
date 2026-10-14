function summary_table = run_all(scenarios, varargin)
%RUN_ALL Run scenario sweeps through bms_unified.slx and legacy models.
%   summary_table = run_all() runs all 16 default scenarios.
%   summary_table = run_all('quick') runs 2 representative scenarios (1 synthetic, 1 NASA).
%   summary_table = run_all(scenario_list) runs the specified scenarios.
%
%   For each scenario:
%     1. Runs bms_unified.slx via Simulink.SimulationInput
%     2. Runs corresponding legacy model (advanced_bms_aco_ekf or sota_bms_aco_ekf)
%     3. Verifies numerical equivalence between unified and legacy models
%     4. Computes error metrics using lib/bms_metrics.m (excluding 120s convergence window)
%     5. Writes per-scenario time-series CSV to simulink_matlab/results/scenario_<name>.csv
%     6. Generates per-scenario 4-panel comparison figure to simulink_matlab/results/fig_scenario_<name>.png
%     7. Summarizes all results in simulink_matlab/results/all_scenarios_summary.csv

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    addpath(fullfile(here, 'lib'));
    outdir = fullfile(here, 'results');
    if ~isfolder(outdir),  mkdir(outdir);  end

    % Parse inputs
    if nargin < 1 || isempty(scenarios)
        sc_list = bms_scenario('list');
    elseif ischar(scenarios) || isstring(scenarios)
        if strcmpi(scenarios, 'quick')
            sc_list = {bms_scenario('synthetic_udds'), bms_scenario('B0005_1')};
        elseif strcmpi(scenarios, 'all')
            sc_list = bms_scenario('list');
        else
            sc_list = {bms_scenario(char(scenarios))};
        end
    elseif iscell(scenarios)
        if ischar(scenarios{1}) || isstring(scenarios{1})
            sc_list = cell(1, numel(scenarios));
            for i = 1:numel(scenarios)
                sc_list{i} = bms_scenario(char(scenarios{i}));
            end
        else
            sc_list = scenarios;
        end
    else
        error('run_all:invalidInput', 'Invalid scenarios argument.');
    end

    n_sc = numel(sc_list);
    fprintf('=========================================================================\n');
    fprintf(' BMS Unified Batch Runner: Running %d Scenarios\n', n_sc);
    fprintf(' Output Directory: %s\n', outdir);
    fprintf('=========================================================================\n\n');

    summary_rows = cell(n_sc, 15);

    % Color palette for figures
    c_ref   = [0.10 0.10 0.10];
    c_ekf   = [0.15 0.45 0.80];
    c_adapt = [0.85 0.25 0.20];
    c_cap   = [0.20 0.65 0.35];
    c_cc    = [0.55 0.55 0.55];

    for idx = 1:n_sc
        sc = sc_list{idx};
        fprintf('[%2d/%2d] Running scenario: %-24s (source: %s, duration: %d s) ...\n', ...
            idx, n_sc, sc.name, sc.source, numel(sc.I));

        N = numel(sc.I);
        t_end = N - 1;
        skip = min(120, floor(N / 4));

        % --- 1. Configure and run bms_unified.slx ----------------------
        in_uni = Simulink.SimulationInput('bms_unified');
        in_uni = in_uni.setModelParameter('StopTime', num2str(t_end));
        in_uni = in_uni.setVariable('BMS_StopTime', t_end);

        if strcmp(sc.source, 'synthetic')
            in_uni = in_uni.setVariable('BMS_Source', 1);
            in_uni = in_uni.setVariable('Drive_Profile', [(0:t_end)', sc.I(:)]);
            in_uni = in_uni.setVariable('BMS_SOC0', sc.soc0);
            in_uni = in_uni.setVariable('BMS_SOC_Guess', sc.soc0 + sc.init_err);
            in_uni = in_uni.setVariable('BMS_Seed', sc.seed);
            in_uni = in_uni.setVariable('BMS_FaultMode', sc.fault_mode);
            in_uni = in_uni.setVariable('BMS_InjectFaultsOnReplay', 0);

            % Configure legacy advanced_bms_aco_ekf
            in_leg = Simulink.SimulationInput('advanced_bms_aco_ekf');
            in_leg = in_leg.setModelParameter('StopTime', num2str(t_end));
            in_leg = in_leg.setVariable('Drive_Profile', [(0:t_end)', sc.I(:)]);
            in_leg = in_leg.setVariable('BMS_SOC0', sc.soc0);
            in_leg = in_leg.setVariable('BMS_SOC_Guess', sc.soc0 + sc.init_err);
            in_leg = in_leg.setVariable('BMS_Seed', sc.seed);
            in_leg = in_leg.setVariable('BMS_FaultMode', sc.fault_mode);
            in_leg = in_leg.setVariable('BMS_StopTime', t_end);
            leg_name = 'advanced_bms_aco_ekf';
        else
            in_uni = in_uni.setVariable('BMS_Source', 2);
            in_uni = in_uni.setVariable('NASA_Replay', [(0:t_end)', sc.I(:), sc.V_ref(:), sc.T(:), sc.soc_ref(:)]);
            in_uni = in_uni.setVariable('NASA_SOC_Guess', sc.soc_ref(1) + sc.init_err);
            in_uni = in_uni.setVariable('NASA_StopTime', t_end);
            in_uni = in_uni.setVariable('NASA_CapRatio', 1.0);
            in_uni = in_uni.setVariable('BMS_SOC_Guess', sc.soc_ref(1) + sc.init_err);
            in_uni = in_uni.setVariable('BMS_FaultMode', sc.fault_mode);
            in_uni = in_uni.setVariable('BMS_InjectFaultsOnReplay', double(sc.fault_mode > 0));

            % Configure legacy sota_bms_aco_ekf
            in_leg = Simulink.SimulationInput('sota_bms_aco_ekf');
            in_leg = in_leg.setModelParameter('StopTime', num2str(t_end));
            in_leg = in_leg.setVariable('NASA_Replay', [(0:t_end)', sc.I(:), sc.V_ref(:), sc.T(:), sc.soc_ref(:)]);
            in_leg = in_leg.setVariable('NASA_SOC_Guess', sc.soc_ref(1) + sc.init_err);
            in_leg = in_leg.setVariable('NASA_StopTime', t_end);
            in_leg = in_leg.setVariable('NASA_CapRatio', 1.0);
            leg_name = 'sota_bms_aco_ekf';
        end

        out_uni = sim(in_uni);
        d_uni = out_uni.bms_log;

        out_leg = sim(in_leg);
        d_leg = out_leg.bms_log;

        % Verify equivalence
        if strcmp(sc.source, 'synthetic')
            diff_ekf = max(abs(d_uni(:, 2) - d_leg(:, 2)));
        else
            diff_ekf = max(abs(d_uni(:, 2) - d_leg(:, 2)));
        end

        % Extract logged signals
        time_s     = (0:size(d_uni, 1) - 1)';
        soc_ref    = d_uni(:, 1);
        soc_ekf    = d_uni(:, 2);
        soc_adapt  = d_uni(:, 3);
        soc_cc     = d_uni(:, 4);
        v_meas     = d_uni(:, 5);
        v_est      = d_uni(:, 6);
        innov      = d_uni(:, 7);
        nis        = d_uni(:, 8);
        fault_code = d_uni(:, 9);
        r0_ohm     = d_uni(:, 10);
        r_cov      = d_uni(:, 11);
        soc_cap    = d_uni(:, 12);
        cap_ratio  = d_uni(:, 13);
        src_tag    = d_uni(:, 14);

        % Compute metrics (excluding initial convergence window)
        m_ekf   = bms_metrics(soc_ekf,   soc_ref, skip);
        m_adapt = bms_metrics(soc_adapt, soc_ref, skip);
        m_cap   = bms_metrics(soc_cap,   soc_ref, skip);
        m_cc    = bms_metrics(soc_cc,    soc_ref, skip);

        % --- 2. Write CSV time-series ---------------------------------
        T_sc = table(time_s, soc_ref, soc_ekf, soc_adapt, soc_cc, soc_cap, ...
                     v_meas, v_est, innov, nis, fault_code, r0_ohm, r_cov, cap_ratio, src_tag, ...
                     'VariableNames', {'Time_s', 'SOC_Ref_pct', 'SOC_EKF_pct', 'SOC_Adaptive_pct', ...
                                       'SOC_Coulomb_pct', 'SOC_Cap_pct', 'Voltage_Meas_V', 'Voltage_Est_V', ...
                                       'Innovation_V', 'NIS', 'FaultCode', 'R0_Ohm', 'R_cov_V2', 'CapacityRatio', 'SourceTag'});
        csv_path = fullfile(outdir, sprintf('scenario_%s.csv', sc.name));
        writetable(T_sc, csv_path);

        % --- 3. Generate comparison figure ----------------------------
        fig = figure('Visible', 'off', 'Position', [80 80 1200 780], 'Color', 'w');

        % Subplot 1: SOC Tracking
        subplot(2, 2, 1);
        plot(time_s, soc_ref,   'k-',  'LineWidth', 2.0); hold on;
        plot(time_s, soc_ekf,   '-',   'Color', c_ekf,   'LineWidth', 1.4);
        plot(time_s, soc_adapt, '--',  'Color', c_adapt, 'LineWidth', 1.4);
        plot(time_s, soc_cap,   '-.',  'Color', c_cap,   'LineWidth', 1.4);
        plot(time_s, soc_cc,    ':',   'Color', c_cc,    'LineWidth', 1.2);
        grid on; xlabel('Time (s)'); ylabel('SOC (%)');
        title(sprintf('SOC Tracking [%s]', sc.name), 'FontWeight', 'bold');
        legend('Reference', 'Baseline EKF', 'Adaptive EKF', 'Capacity EKF', 'Coulomb Counting', ...
               'Location', 'best');

        % Subplot 2: SOC Tracking Error
        subplot(2, 2, 2);
        plot(time_s, soc_ekf   - soc_ref, '-',  'Color', c_ekf,   'LineWidth', 1.4); hold on;
        plot(time_s, soc_adapt - soc_ref, '--', 'Color', c_adapt, 'LineWidth', 1.4);
        plot(time_s, soc_cap   - soc_ref, '-.', 'Color', c_cap,   'LineWidth', 1.4);
        yline(0, 'k:');
        xline(skip, 'r--', sprintf('t=%ds', skip), 'LabelVerticalAlignment', 'bottom');
        grid on; xlabel('Time (s)'); ylabel('SOC Error (%)');
        title(sprintf('Error (MAE: EKF %.2f%%, Adapt %.2f%%, Cap %.2f%%)', ...
              m_ekf.mae, m_adapt.mae, m_cap.mae), 'FontWeight', 'bold');
        legend('EKF Error', 'Adaptive Error', 'Capacity Error', 'Location', 'best');

        % Subplot 3: Voltage & Innovation Tracking
        subplot(2, 2, 3);
        yyaxis left
        plot(time_s, v_meas, 'k-',  'LineWidth', 1.2); hold on;
        plot(time_s, v_est,  'b--', 'LineWidth', 1.2);
        ylabel('Terminal Voltage (V)');
        yyaxis right
        plot(time_s, innov, 'r-', 'LineWidth', 1.0);
        ylabel('Innovation (V)');
        grid on; xlabel('Time (s)');
        title('Terminal Voltage & Innovation Tracking', 'FontWeight', 'bold');

        % Subplot 4: Diagnostics & Model Parameters
        subplot(2, 2, 4);
        yyaxis left
        plot(time_s, nis, 'm-', 'LineWidth', 1.2); hold on;
        yline(1.0, 'k:');
        ylabel('NIS (Nominal ~ 1.0)');
        yyaxis right
        plot(time_s, fault_code, 'r-', 'LineWidth', 1.2); hold on;
        plot(time_s, cap_ratio,  'g--', 'LineWidth', 1.2);
        ylabel('Fault Code / Capacity Ratio');
        grid on; xlabel('Time (s)');
        title('NIS, Fault Code & Capacity Tracking', 'FontWeight', 'bold');
        legend('NIS', 'Fault Code', 'Cap Ratio', 'Location', 'best');

        sgtitle(sprintf('BMS Unified Analysis: %s (Legacy match: %.1e %%)', sc.name, diff_ekf), ...
                'FontSize', 12, 'FontWeight', 'bold');

        fig_path = fullfile(outdir, sprintf('fig_scenario_%s.png', sc.name));
        exportgraphics(fig, fig_path, 'Resolution', 150);
        close(fig);

        % Record summary row
        summary_rows(idx, :) = { ...
            sc.name, sc.source, N, ...
            m_ekf.mae,   m_ekf.rmse,   m_ekf.max, ...
            m_adapt.mae, m_adapt.rmse, m_adapt.max, ...
            m_cap.mae,   m_cap.rmse,   m_cap.max, ...
            m_cc.mae,    m_cc.max,     diff_ekf ...
        };

        fprintf('       EKF MAE: %5.2f%% | Adapt MAE: %5.2f%% | Cap MAE: %5.2f%% | Legacy Equiv: %.1e %%\n', ...
            m_ekf.mae, m_adapt.mae, m_cap.mae, diff_ekf);
    end

    % --- 4. Write summary table CSV -----------------------------------
    summary_table = cell2table(summary_rows, 'VariableNames', { ...
        'Scenario', 'Source', 'Duration_s', ...
        'EKF_MAE_pct', 'EKF_RMSE_pct', 'EKF_MaxErr_pct', ...
        'Adapt_MAE_pct', 'Adapt_RMSE_pct', 'Adapt_MaxErr_pct', ...
        'Cap_MAE_pct', 'Cap_RMSE_pct', 'Cap_MaxErr_pct', ...
        'CC_MAE_pct', 'CC_MaxErr_pct', 'Legacy_Diff_pct' ...
    });

    summary_csv = fullfile(outdir, 'all_scenarios_summary.csv');
    writetable(summary_table, summary_csv);
    fprintf('\n=========================================================================\n');
    fprintf(' Summary table saved to: %s\n', summary_csv);
    fprintf('=========================================================================\n');
    disp(summary_table);

    % Close loaded models
    bdclose('bms_unified');
    bdclose('advanced_bms_aco_ekf');
    bdclose('sota_bms_aco_ekf');
end
