function build_bms_models(which)
%BUILD_BMS_MODELS Programmatically (re)build the BMS Simulink models from the shared library.
%   build_bms_models()            builds both models
%   build_bms_models('synthetic') advanced_bms_aco_ekf : simulated plant + sensors (+ fault injection) + EKF
%   build_bms_models('nasa')      sota_bms_aco_ekf     : replay of REAL NASA cell data, EKF vs adaptive EKF
%
%   All algorithm code lives in lib/*.m (real files, unit-testable, codegen-ready). The MATLAB Function
%   blocks are thin wrappers, so the model, the test-bench and the C firmware share one algorithm.
    if nargin < 1,  which = 'all';  end
    here = fileparts(mfilename('fullpath'));
    addpath(here);  addpath(fullfile(here, 'lib'));
    if any(strcmp(which, {'all', 'synthetic'})),  build_synthetic(here);  end
    if any(strcmp(which, {'all', 'nasa'})),       build_nasa(here);       end
end

%% ======================================================================
function build_synthetic(here)
    m = 'advanced_bms_aco_ekf';
    fprintf('Building %s ...\n', m);
    bms_setup('synthetic', true);
    new_model(m, here, 'BMS_StopTime', 'bms_setup(''synthetic'')');

    % --- sources / scenario constants --------------------------------------
    add_block('simulink/Sources/From Workspace', [m '/Drive_Cycle_Current'], 'VariableName', 'Drive_Profile', ...
        'SampleTime', '1', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [40 150 150 180]);
    const(m, 'Initial_SOC_True',  'BMS_SOC0',      [40 230 150 255]);
    const(m, 'Initial_SOC_Guess', 'BMS_SOC_Guess', [40 290 150 315]);
    const(m, 'Fault_Mode',        'BMS_FaultMode', [40 350 150 375]);
    const(m, 'Noise_Seed',        'BMS_Seed',      [40 410 150 435]);
    const(m, 'ACO_Q_diag',        'BMS_Q',         [40 470 150 495]);
    const(m, 'ACO_R',             'BMS_R',         [40 530 150 555]);

    % --- plant -> sensors -> estimator ------------------------------------
    mfb(m, 'Battery_Physics_Plant', [260 130 420 260], plant_code('synthetic'),  [0.80 0.90 1.00]);
    mfb(m, 'Sensor_Measurement',    [500 130 660 290], sensor_code('synthetic'), [1.00 0.90 0.75]);
    mfb(m, 'ACO_EKF_Controller',    [760 130 960 400], ctrl_code('synthetic', 0, 'ACO_EKF_Controller'), [0.80 1.00 0.80]);

    ln(m, 'Drive_Cycle_Current/1',  'Battery_Physics_Plant/1');
    ln(m, 'Initial_SOC_True/1',     'Battery_Physics_Plant/2');
    ln(m, 'Battery_Physics_Plant/2','Sensor_Measurement/1');
    ln(m, 'Drive_Cycle_Current/1',  'Sensor_Measurement/2');
    ln(m, 'Battery_Physics_Plant/3','Sensor_Measurement/3');
    ln(m, 'Fault_Mode/1',           'Sensor_Measurement/4');
    ln(m, 'Noise_Seed/1',           'Sensor_Measurement/5');
    ln(m, 'Sensor_Measurement/2',   'ACO_EKF_Controller/1');   % I_meas
    ln(m, 'Sensor_Measurement/1',   'ACO_EKF_Controller/2');   % V_meas
    ln(m, 'Sensor_Measurement/3',   'ACO_EKF_Controller/3');   % T_meas
    ln(m, 'ACO_Q_diag/1',           'ACO_EKF_Controller/4');
    ln(m, 'ACO_R/1',                'ACO_EKF_Controller/5');
    ln(m, 'Initial_SOC_Guess/1',    'ACO_EKF_Controller/6');

    % --- scopes ---------------------------------------------------------------
    % controller outs: 1 SOC 2 Vest 3 Innov 4 Ptrace 5 NIS 6 Fault 7 R0 8 SOC_CC 9 R
    x0 = 1100;
    mux_scope(m, 'SOC_Comparison_Scope', {'Battery_Physics_Plant/1', 'ACO_EKF_Controller/1', 'ACO_EKF_Controller/8'}, ...
        [x0 40], 'True SOC | EKF SOC | Coulomb counting');
    sub_scope(m, 'SOC_Error_Scope', 'ACO_EKF_Controller/1', 'Battery_Physics_Plant/1', [x0 120], 'EKF - True [%]');
    mux_scope(m, 'Voltage_Tracking_Scope', {'Battery_Physics_Plant/2', 'Sensor_Measurement/1', 'ACO_EKF_Controller/2'}, ...
        [x0 200], 'True V | Measured V | EKF predicted V');
    scope(m, 'EKF_Covariance_Scope', 'ACO_EKF_Controller/4', [x0 280], 'trace(P)');
    scope(m, 'Innovation_NIS_Scope', 'ACO_EKF_Controller/5', [x0 340], 'Normalised innovation squared (expect ~1)');
    scope(m, 'Diagnostic_Fault_Code_Scope', 'ACO_EKF_Controller/6', [x0 400], ...
        '1 V range | 2 T range | 4 I range | 8 innovation rejected | 16 filter inconsistent');
    scope(m, 'Drive_Current_Scope', 'Drive_Cycle_Current/1', [x0 460], 'Current [A]');
    scope(m, 'Temperature_Scope', 'Battery_Physics_Plant/3', [x0 520], 'Cell temperature [K]');

    add_block('simulink/Sinks/Display', [m '/Live_SOC_Display'], 'Position', [x0 + 230 40 x0 + 310 70]);
    ln(m, 'ACO_EKF_Controller/1', 'Live_SOC_Display/1');

    % --- machine-readable log for the test bench ----------------------------------
    names = {'Battery_Physics_Plant/1', 'ACO_EKF_Controller/1', 'ACO_EKF_Controller/8', 'Battery_Physics_Plant/2', ...
             'Sensor_Measurement/1', 'ACO_EKF_Controller/2', 'ACO_EKF_Controller/3', 'ACO_EKF_Controller/4', ...
             'ACO_EKF_Controller/5', 'ACO_EKF_Controller/6', 'Battery_Physics_Plant/3'};
    logger(m, names, [x0 600]);

    note(m, ['Advanced BMS: 2-RC thermal plant (with parameter mismatch) -> noisy/faultable sensors -> ' ...
             'ACO-tuned 3-state Joseph EKF with diagnostics. Algorithms: lib/bms_*.m. ' ...
             'Scenario variables are set by bms_setup (InitFcn) and can be overridden with Simulink.SimulationInput.'], [40 30]);
    finish(m, here);
end

%% ======================================================================
function build_nasa(here)
    m = 'sota_bms_aco_ekf';
    fprintf('Building %s ...\n', m);
    bms_setup('nasa', true);
    new_model(m, here, 'NASA_StopTime', 'bms_setup(''nasa'')');

    add_block('simulink/Sources/From Workspace', [m '/NASA_Cell_Replay'], 'VariableName', 'NASA_Replay', ...
        'SampleTime', '1', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [40 150 150 190]);
    add_block('simulink/Signal Routing/Demux', [m '/Demux_Replay'], 'Outputs', '4', 'Position', [190 130 195 230]);
    ln(m, 'NASA_Cell_Replay/1', 'Demux_Replay/1');
    add_block('simulink/Math Operations/Gain', [m '/Ref_SOC_Percent'], 'Gain', '100', 'Position', [240 205 280 235]);
    ln(m, 'Demux_Replay/4', 'Ref_SOC_Percent/1');
    const(m, 'Initial_SOC_Guess', 'NASA_SOC_Guess', [40 290 150 315]);
    const(m, 'ACO_Q_diag',        'NASA_Q',         [40 350 150 375]);
    const(m, 'ACO_R',             'NASA_R',         [40 410 150 435]);

    % Demux: 1 current, 2 measured voltage, 3 temperature [K], 4 reference SOC
    mfb(m, 'EKF_Baseline',   [400 120 600 340], ctrl_code('nasa', 0, 'EKF_Baseline'), [0.80 1.00 0.80]);
    mfb(m, 'SOTA_Adaptive_Controller', [400 380 600 600], ctrl_code('nasa', 2, 'SOTA_Adaptive_Controller'), [0.95 0.80 1.00]);
    mfb(m, 'SOH_Capacity_Estimator', [400 640 600 860], ctrl_code('nasa', 10, 'SOH_Capacity_Estimator'), [1.00 0.95 0.75]);
    const(m, 'True_Capacity_Ratio', 'NASA_CapRatio', [40 470 150 495]);
    for c = {'EKF_Baseline', 'SOTA_Adaptive_Controller', 'SOH_Capacity_Estimator'}
        ln(m, 'Demux_Replay/1', [c{1} '/1']);
        ln(m, 'Demux_Replay/2', [c{1} '/2']);
        ln(m, 'Demux_Replay/3', [c{1} '/3']);
        ln(m, 'ACO_Q_diag/1',        [c{1} '/4']);
        ln(m, 'ACO_R/1',             [c{1} '/5']);
        ln(m, 'Initial_SOC_Guess/1', [c{1} '/6']);
    end

    x0 = 800;
    mux_scope(m, 'SOC_Comparison_Scope', {'Ref_SOC_Percent/1', 'EKF_Baseline/1', 'SOTA_Adaptive_Controller/1', 'EKF_Baseline/8', 'SOH_Capacity_Estimator/1'}, ...
        [x0 40], 'Reference SOC | EKF | Adaptive EKF | Coulomb counting | EKF+capacity state');
    mux_sub(m, 'SOC_Error_Scope', {'EKF_Baseline/1', 'SOTA_Adaptive_Controller/1', 'SOH_Capacity_Estimator/1'}, 'Ref_SOC_Percent/1', [x0 120], ...
        'EKF error | Adaptive EKF error | EKF+capacity error  [%]');
    mux_scope(m, 'Voltage_Tracking_Scope', {'Demux_Replay/2', 'SOTA_Adaptive_Controller/2'}, [x0 200], ...
        'Measured V (real cell) | Adaptive EKF predicted V');
    scope(m, 'R_Adaptation_Scope', 'SOTA_Adaptive_Controller/9', [x0 280], 'Sage-Husa measurement-noise variance R');
    scope(m, 'R0_Tracking_Scope', 'SOTA_Adaptive_Controller/7', [x0 340], 'R0 used by adaptive model [ohm]');
    scope(m, 'Innovation_NIS_Scope', 'SOTA_Adaptive_Controller/5', [x0 400], 'NIS (expect ~1)');
    scope(m, 'Diagnostic_Fault_Code_Scope', 'SOTA_Adaptive_Controller/6', [x0 460], 'Diagnostic bit-mask');
    mux_scope(m, 'Capacity_SOH_Scope', {'SOH_Capacity_Estimator/10', 'True_Capacity_Ratio/1'}, [x0 520], ...
        'Estimated capacity ratio (SOH) | true capacity ratio');
    add_block('simulink/Sinks/Display', [m '/Live_SOC_Display'], 'Position', [x0 + 230 40 x0 + 310 70]);
    ln(m, 'SOTA_Adaptive_Controller/1', 'Live_SOC_Display/1');

    names = {'Ref_SOC_Percent/1', 'EKF_Baseline/1', 'SOTA_Adaptive_Controller/1', 'EKF_Baseline/8', 'Demux_Replay/2', ...
             'SOTA_Adaptive_Controller/2', 'SOTA_Adaptive_Controller/3', 'SOTA_Adaptive_Controller/5', ...
             'SOTA_Adaptive_Controller/6', 'SOTA_Adaptive_Controller/7', 'SOTA_Adaptive_Controller/9', ...
             'SOH_Capacity_Estimator/1', 'SOH_Capacity_Estimator/10'};
    logger(m, names, [x0 620]);

    note(m, ['Real-data BMS: replays a measured NASA Ames discharge (data_raw/*.mat). The OCV/RC model was calibrated on ' ...
             'cell B0006 and the ACO covariances tuned on B0006, so replays of B0005/B0007 are out-of-sample. ' ...
             'Top: conventional ACO-tuned EKF. Middle: EKF + Sage-Husa R adaptation. Bottom: EKF + online capacity (SOH) state.'], [40 30]);
    finish(m, here);
end

%% ======================================================================
%  helpers
%% ======================================================================
function new_model(m, here, stop_var, init_cmd)
    if bdIsLoaded(m),  close_system(m, 0);  end
    f = fullfile(here, [m '.slx']);
    if isfile(f),  delete(f);  end
    new_system(m);
    set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '1', ...
        'StopTime', stop_var, 'InitFcn', init_cmd, 'SaveOutput', 'off', 'SaveTime', 'on', ...
        'ReturnWorkspaceOutputs', 'on');
    save_system(m, f);
end

function finish(m, here)
    set_param(m, 'ZoomFactor', 'FitSystem');
    save_system(m, fullfile(here, [m '.slx']));
    fprintf('  saved %s.slx\n', m);
end

function const(m, name, value, pos)
    add_block('simulink/Sources/Constant', [m '/' name], 'Value', value, 'Position', pos, ...
        'SampleTime', '-1', 'ShowName', 'on');
end

function ln(m, src, dst)
    add_line(m, src, dst, 'autorouting', 'on');
end

function mfb(m, name, pos, code, rgb)
    p = [m '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', p, 'Position', pos);
    save_system(m);
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', '-and', 'Path', p);
    ch.Script = code;
    set_param(p, 'BackgroundColor', mat2str(rgb));
end

function scope(m, name, src, pos, desc)
    p = [m '/' name];
    add_block('simulink/Sinks/Scope', p, 'Position', [pos(1) pos(2) pos(1) + 40 pos(2) + 30]);
    set_param(p, 'Description', desc);
    ln(m, src, [name '/1']);
    try
        sc = get_param(p, 'ScopeConfiguration');
        sc.Title = desc;  sc.ShowGrid = true;
    catch
    end
end

function mux_scope(m, name, srcs, pos, desc)
    mx = ['Mux_' name];
    add_block('simulink/Signal Routing/Mux', [m '/' mx], 'Inputs', num2str(numel(srcs)), ...
        'Position', [pos(1) - 30 pos(2) pos(1) - 25 pos(2) + 20 + 10 * numel(srcs)]);
    for i = 1:numel(srcs),  ln(m, srcs{i}, sprintf('%s/%d', mx, i));  end
    set_param([m '/' mx], 'ShowName', 'off');
    scope(m, name, [mx '/1'], pos, desc);
end

function sub_scope(m, name, a, b, pos, desc)
    sb = ['Sub_' name];
    add_block('simulink/Math Operations/Subtract', [m '/' sb], 'Inputs', '+-', ...
        'Position', [pos(1) - 70 pos(2) pos(1) - 45 pos(2) + 30]);
    ln(m, a, [sb '/1']);  ln(m, b, [sb '/2']);
    set_param([m '/' sb], 'ShowName', 'off');
    scope(m, name, [sb '/1'], pos, desc);
end

function mux_sub(m, name, ests, ref, pos, desc)
    mx = ['Mux_' name];
    add_block('simulink/Signal Routing/Mux', [m '/' mx], 'Inputs', num2str(numel(ests)), ...
        'Position', [pos(1) - 30 pos(2) pos(1) - 25 pos(2) + 30]);
    for i = 1:numel(ests)
        sb = sprintf('Sub_%s_%d', name, i);
        add_block('simulink/Math Operations/Subtract', [m '/' sb], 'Inputs', '+-', ...
            'Position', [pos(1) - 100 pos(2) + 35 * (i - 1) pos(1) - 75 pos(2) + 25 + 35 * (i - 1)]);
        ln(m, ests{i}, [sb '/1']);  ln(m, ref, [sb '/2']);
        ln(m, [sb '/1'], sprintf('%s/%d', mx, i));
        set_param([m '/' sb], 'ShowName', 'off');
    end
    set_param([m '/' mx], 'ShowName', 'off');
    scope(m, name, [mx '/1'], pos, desc);
end

function logger(m, sigs, pos)
    add_block('simulink/Signal Routing/Mux', [m '/Log_Mux'], 'Inputs', num2str(numel(sigs)), ...
        'Position', [pos(1) pos(2) pos(1) + 5 pos(2) + 120]);
    for i = 1:numel(sigs),  ln(m, sigs{i}, sprintf('Log_Mux/%d', i));  end
    add_block('simulink/Sinks/To Workspace', [m '/Log_To_Workspace'], 'VariableName', 'bms_log', ...
        'SaveFormat', 'Array', 'SampleTime', '-1', 'Position', [pos(1) + 40 pos(2) + 40 pos(1) + 120 pos(2) + 70]);
    ln(m, 'Log_Mux/1', 'Log_To_Workspace/1');
    set_param([m '/Log_Mux'], 'ShowName', 'off');
end

function note(m, txt, pos)
    try
        a = Simulink.Annotation([m '/Description']);
        a.Text = txt;  a.Position = pos;  a.FontSize = 11;
    catch
        try
            add_block('built-in/Note', [m '/Description'], 'Position', pos, 'Text', txt); %#ok<*NASGU>
        catch
        end
    end
end

%% ======================================================================
%  thin MATLAB-Function-block wrappers (algorithms live in lib/)
%% ======================================================================
function code = plant_code(variant)
    code = sprintf([ ...
    'function [True_SOC, True_Voltage, Temperature_K] = Battery_Physics_Plant(I_load, SOC0)\n' ...
    '%%#codegen\n' ...
    '%% 2-RC Arrhenius cell with capacity/resistance mismatch (see bms_params). Algorithm: lib/bms_plant_step.m\n' ...
    'P = coder.const(bms_params(''%s''));\n' ...
    'persistent x k;\n' ...
    'if isempty(x)\n' ...
    '    x = [SOC0; 0.0; 0.0];  k = 0.0;\n' ...
    'end\n' ...
    'k = k + 1.0;\n' ...
    'Temperature_K = bms_temp_profile(k);\n' ...
    '[x, True_Voltage] = bms_plant_step(x, I_load, Temperature_K, P);\n' ...
    'True_SOC = 100.0 * x(1);\n'], variant);
end

function code = sensor_code(variant)
    code = sprintf([ ...
    'function [V_meas, I_meas, T_meas] = Sensor_Measurement(True_Voltage, I_raw, Temperature_K, Fault_Mode, Seed)\n' ...
    '%%#codegen\n' ...
    '%% Noisy 12-bit-class sensors with injectable faults. Algorithm: lib/bms_sensor_step.m (seeded, reproducible)\n' ...
    'P = coder.const(bms_params(''%s''));\n' ...
    'persistent ss k;\n' ...
    'if isempty(ss)\n' ...
    '    ss = bms_seed(Seed);  k = 0.0;\n' ...
    'end\n' ...
    'k = k + 1.0;\n' ...
    '[V_meas, I_meas, T_meas, ss] = bms_sensor_step(True_Voltage, I_raw, Temperature_K, k, ss, P, Fault_Mode);\n'], variant);
end

function code = ctrl_code(variant, mode, fname)
    code = sprintf([ ...
    'function [Est_SOC, Est_Voltage, Innov_Error, P_trace, NIS, Fault_Code, R0_used, SOC_CoulombCount, R_cov, Capacity_Ratio] = %s(I_meas, V_meas, T_meas, Q_diag, R_var, SOC_guess)\n' ...
    '%%#codegen\n' ...
    '%% 3-state Joseph-form EKF + diagnostics. Algorithm: lib/bms_est_step.m (estimator mode %d)\n' ...
    'P = coder.const(bms_params(''%s''));\n' ...
    'persistent st;\n' ...
    'if isempty(st)\n' ...
    '    st = bms_est_init(P, SOC_guess, Q_diag, R_var, %d);\n' ...
    'end\n' ...
    '[st, o] = bms_est_step(st, I_meas, V_meas, T_meas, P, %d);\n' ...
    'Est_SOC = o.soc;  Est_Voltage = o.vest;  Innov_Error = o.innov;  P_trace = o.ptrace;\n' ...
    'NIS = o.nis;  Fault_Code = o.fault;  R0_used = o.r0;  SOC_CoulombCount = o.soc_cc;  R_cov = o.rcov;  Capacity_Ratio = o.cap;\n'], ...
    fname, mode, variant, mode, mode);
end
