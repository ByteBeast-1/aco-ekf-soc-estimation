function build_bms_models(which)
%BUILD_BMS_MODELS Programmatically (re)build the BMS Simulink models from the shared library.
%   build_bms_models()            builds the data dictionary and both models
%   build_bms_models('synthetic') advanced_bms_aco_ekf : simulated plant + sensors (+ fault injection) + EKF
%   build_bms_models('nasa')      sota_bms_aco_ekf     : replay of REAL NASA cell data, EKF vs adaptive EKF
%
%   All algorithm code lives in lib/*.m (real files, unit-testable, codegen-ready). The MATLAB Function
%   blocks are thin wrappers, so the model, the test-bench and the C firmware share one algorithm.
%   Signals between blocks travel on bus objects (BMS_PlantTruth, BMS_Measurement, BMS_Estimate) defined,
%   with units, in bms_data.sldd. Strict diagnostics make dangling ports, algebraic loops and bus
%   mismatches build errors.
    if nargin < 1,  which = 'all';  end
    here = fileparts(mfilename('fullpath'));
    addpath(here);  addpath(fullfile(here, 'lib'));
    bms_build_dictionary();
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

    % --- plant -> sensors -> estimator, connected by buses --------------------------
    mfb(m, 'Battery_Physics_Plant', [260 130 420 260], plant_code('synthetic'),  [0.80 0.90 1.00], ...
        {'truth', 'BMS_PlantTruth'});
    mfb(m, 'Sensor_Measurement',    [500 130 660 290], sensor_code('synthetic'), [1.00 0.90 0.75], ...
        {'truth', 'BMS_PlantTruth', 'meas', 'BMS_Measurement'});
    mfb(m, 'ACO_EKF_Controller',    [760 130 960 400], ctrl_code('synthetic', 0, 'ACO_EKF_Controller'), [0.80 1.00 0.80], ...
        {'meas', 'BMS_Measurement', 'est', 'BMS_Estimate'});

    ln(m, 'Drive_Cycle_Current/1',  'Battery_Physics_Plant/1');
    ln(m, 'Initial_SOC_True/1',     'Battery_Physics_Plant/2');
    ln(m, 'Battery_Physics_Plant/1','Sensor_Measurement/1');     % truth bus
    ln(m, 'Drive_Cycle_Current/1',  'Sensor_Measurement/2');     % raw current
    ln(m, 'Fault_Mode/1',           'Sensor_Measurement/3');
    ln(m, 'Noise_Seed/1',           'Sensor_Measurement/4');
    ln(m, 'Sensor_Measurement/1',   'ACO_EKF_Controller/1');     % measurement bus
    ln(m, 'ACO_Q_diag/1',           'ACO_EKF_Controller/2');
    ln(m, 'ACO_R/1',                'ACO_EKF_Controller/3');
    ln(m, 'Initial_SOC_Guess/1',    'ACO_EKF_Controller/4');

    % --- bus selectors: pick signals off the buses (generous width & height) -------
    TR = {'SOC_pct', 'Voltage_V', 'Temperature_K'};
    MS = {'Voltage_V'};
    ES = {'SOC_pct', 'SOC_Coulomb_pct', 'Voltage_V', 'Innovation_V', 'PTrace', 'NIS', 'FaultCode'};
    bsel(m, 'BS_Estimate', 'ACO_EKF_Controller/1',    ES, [1060  80 1065 380]);
    bsel(m, 'BS_Truth',    'Battery_Physics_Plant/1', TR, [1060 440 1065 570]);
    bsel(m, 'BS_Measured', 'Sensor_Measurement/1',    MS, [1060 620 1065 670]);
    t  = @(s) sig('BS_Truth', TR, s);
    me = @(s) sig('BS_Measured', MS, s);
    e  = @(s) sig('BS_Estimate', ES, s);

    % --- scopes (spaced generously at x0 = 1260 to avoid any label collisions) ---
    x0 = 1260;
    mux_scope(m, 'SOC_Comparison_Scope', {t('SOC_pct'), e('SOC_pct'), e('SOC_Coulomb_pct')}, ...
        [x0 50], 'True SOC | EKF SOC | Coulomb counting');
    sub_scope(m, 'SOC_Error_Scope', e('SOC_pct'), t('SOC_pct'), [x0 140], 'EKF - True [%]');
    mux_scope(m, 'Voltage_Tracking_Scope', {t('Voltage_V'), me('Voltage_V'), e('Voltage_V')}, ...
        [x0 230], 'True V | Measured V | EKF predicted V');
    scope(m, 'EKF_Covariance_Scope', e('PTrace'), [x0 320], 'trace(P)');
    scope(m, 'Innovation_NIS_Scope', e('NIS'), [x0 400], 'Normalised innovation squared (expect ~1)');
    scope(m, 'Diagnostic_Fault_Code_Scope', e('FaultCode'), [x0 480], ...
        '1 V range | 2 T range | 4 I range | 8 innovation rejected | 16 filter inconsistent');
    scope(m, 'Drive_Current_Scope', 'Drive_Cycle_Current/1', [x0 560], 'Current [A]');
    scope(m, 'Temperature_Scope', t('Temperature_K'), [x0 640], 'Cell temperature [K]');

    add_block('simulink/Sinks/Display', [m '/Live_SOC_Display'], 'Position', [x0 + 80 45 x0 + 170 80]);
    ln(m, e('SOC_pct'), 'Live_SOC_Display/1');

    % --- machine-readable log for the test bench (11 columns, order is part of the interface) -----
    names = {t('SOC_pct'), e('SOC_pct'), e('SOC_Coulomb_pct'), t('Voltage_V'), me('Voltage_V'), e('Voltage_V'), ...
             e('Innovation_V'), e('PTrace'), e('NIS'), e('FaultCode'), t('Temperature_K')};
    logger(m, names, [x0 730]);

    note(m, sprintf(['Advanced BMS: 2-RC thermal plant (with parameter mismatch) -> noisy/faultable sensors -> ACO-tuned 3-state Joseph EKF.\n' ...
                     'Algorithms: lib/bms_*.m. Signal interfaces: bus objects in bms_data.sldd.\n' ...
                     'Scenario variables are set by bms_setup (InitFcn) and can be overridden with Simulink.SimulationInput.']), [40 30]);
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
    const(m, 'True_Capacity_Ratio', 'NASA_CapRatio', [40 470 150 495]);

    % Demux: 1 current, 2 measured voltage, 3 temperature [K], 4 reference SOC -> measurement bus
    add_block('simulink/Signal Routing/Bus Creator', [m '/Measurement_Bus'], 'Inputs', '3', ...
        'UseBusObject', 'on', 'BusObject', 'BMS_Measurement', 'Position', [300 120 305 180]);
    lnn(m, 'Demux_Replay/2', 'Measurement_Bus/1', 'Voltage_V');
    lnn(m, 'Demux_Replay/1', 'Measurement_Bus/2', 'Current_A');
    lnn(m, 'Demux_Replay/3', 'Measurement_Bus/3', 'Temperature_K');

    mfb(m, 'EKF_Baseline',   [400 120 600 340], ctrl_code('nasa', 0, 'EKF_Baseline'), [0.80 1.00 0.80], ...
        {'meas', 'BMS_Measurement', 'est', 'BMS_Estimate'});
    mfb(m, 'SOTA_Adaptive_Controller', [400 380 600 600], ctrl_code('nasa', 2, 'SOTA_Adaptive_Controller'), [0.95 0.80 1.00], ...
        {'meas', 'BMS_Measurement', 'est', 'BMS_Estimate'});
    mfb(m, 'SOH_Capacity_Estimator', [400 640 600 860], ctrl_code('nasa', 10, 'SOH_Capacity_Estimator'), [1.00 0.95 0.75], ...
        {'meas', 'BMS_Measurement', 'est', 'BMS_Estimate'});
    for c = {'EKF_Baseline', 'SOTA_Adaptive_Controller', 'SOH_Capacity_Estimator'}
        ln(m, 'Measurement_Bus/1',   [c{1} '/1']);
        ln(m, 'ACO_Q_diag/1',        [c{1} '/2']);
        ln(m, 'ACO_R/1',             [c{1} '/3']);
        ln(m, 'Initial_SOC_Guess/1', [c{1} '/4']);
    end

    EK = {'SOC_pct', 'SOC_Coulomb_pct'};
    SA = {'SOC_pct', 'Voltage_V', 'Innovation_V', 'NIS', 'FaultCode', 'R0_Ohm', 'R_cov_V2'};
    SH = {'SOC_pct', 'CapacityRatio'};
    bsel(m, 'BS_Baseline', 'EKF_Baseline/1',             EK, [670 180 675 250]);
    bsel(m, 'BS_Adaptive', 'SOTA_Adaptive_Controller/1', SA, [670 410 675 580]);
    bsel(m, 'BS_Capacity', 'SOH_Capacity_Estimator/1',   SH, [670 700 675 770]);
    eb = @(s) sig('BS_Baseline', EK, s);
    ea = @(s) sig('BS_Adaptive', SA, s);
    es = @(s) sig('BS_Capacity', SH, s);

    % --- scopes (spaced generously at x0 = 940 to avoid label and subtract block collisions) ---
    x0 = 940;
    mux_scope(m, 'SOC_Comparison_Scope', {'Ref_SOC_Percent/1', eb('SOC_pct'), ea('SOC_pct'), eb('SOC_Coulomb_pct'), es('SOC_pct')}, ...
        [x0 60], 'Reference SOC | EKF | Adaptive EKF | Coulomb counting | EKF+capacity state');
    mux_sub(m, 'SOC_Error_Scope', {eb('SOC_pct'), ea('SOC_pct'), es('SOC_pct')}, 'Ref_SOC_Percent/1', [x0 160], ...
        'EKF error | Adaptive EKF error | EKF+capacity error  [%]');
    mux_scope(m, 'Voltage_Tracking_Scope', {'Demux_Replay/2', ea('Voltage_V')}, [x0 260], ...
        'Measured V (real cell) | Adaptive EKF predicted V');
    scope(m, 'R_Adaptation_Scope', ea('R_cov_V2'), [x0 350], 'Sage-Husa measurement-noise variance R');
    scope(m, 'R0_Tracking_Scope', ea('R0_Ohm'), [x0 430], 'R0 used by adaptive model [ohm]');
    scope(m, 'Innovation_NIS_Scope', ea('NIS'), [x0 510], 'NIS (expect ~1)');
    scope(m, 'Diagnostic_Fault_Code_Scope', ea('FaultCode'), [x0 590], 'Diagnostic bit-mask');
    mux_scope(m, 'Capacity_SOH_Scope', {es('CapacityRatio'), 'True_Capacity_Ratio/1'}, [x0 670], ...
        'Estimated capacity ratio (SOH) | true capacity ratio');
    add_block('simulink/Sinks/Display', [m '/Live_SOC_Display'], 'Position', [x0 + 80 50 x0 + 170 85]);
    ln(m, ea('SOC_pct'), 'Live_SOC_Display/1');

    names = {'Ref_SOC_Percent/1', eb('SOC_pct'), ea('SOC_pct'), eb('SOC_Coulomb_pct'), 'Demux_Replay/2', ...
             ea('Voltage_V'), ea('Innovation_V'), ea('NIS'), ea('FaultCode'), ea('R0_Ohm'), ea('R_cov_V2'), ...
             es('SOC_pct'), es('CapacityRatio')};
    logger(m, names, [x0 760]);

    note(m, sprintf(['Real-data BMS: replays a measured NASA Ames discharge (data_raw/*.mat).\n' ...
                     'Calibrated on B0006; evaluated out-of-sample on B0005/B0007.\n' ...
                     'Top: Baseline EKF | Middle: Adaptive EKF (Sage-Husa R) | Bottom: Joint SOH/SOC Estimator.']), [40 30]);
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
    save_system(m, f);
    set_param(m, 'DataDictionary', 'bms_data.sldd');
    set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '1', ...
        'StopTime', stop_var, 'InitFcn', init_cmd, 'SaveOutput', 'off', 'SaveTime', 'on', ...
        'ReturnWorkspaceOutputs', 'on');
    try
        set_param(m, 'EnablePacing', 'on', 'SimulationPacingRate', 20);
    catch
    end
    % strict diagnostics: structural mistakes are build errors, not warnings
    strict = {'UnconnectedInputMsg', 'error'; 'UnconnectedOutputMsg', 'error'; 'UnconnectedLineMsg', 'error'; ...
              'AlgebraicLoopMsg', 'error'; 'ArtificialAlgebraicLoopMsg', 'error'; 'BusObjectLabelMismatch', 'error'; ...
              'StrictBusMsg', 'ErrorOnBusTreatedAsVector'; 'SignalInfNanChecking', 'error'};
    for i = 1:size(strict, 1)
        try
            set_param(m, strict{i, 1}, strict{i, 2});
        catch ME
            warning('build_bms_models:diag', 'Could not set %s: %s', strict{i, 1}, ME.message);
        end
    end
    % document the intended target (32-bit ARM); real code generation needs Embedded Coder
    try,  set_param(m, 'ProdHWDeviceType', 'ARM Compatible->ARM Cortex');  catch,  end
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

function lnn(m, src, dst, name)
    h = add_line(m, src, dst, 'autorouting', 'on');
    set_param(h, 'Name', name);
end

function mfb(m, name, pos, code, rgb, bus_ports)
    p = [m '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', p, 'Position', pos);
    save_system(m);
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', '-and', 'Path', p);
    ch.Script = code;
    for i = 1:2:numel(bus_ports)          % {'argName', 'BusObjectName', ...}
        d = ch.find('-isa', 'Stateflow.Data', '-and', 'Name', bus_ports{i});
        d.DataType = ['Bus: ' bus_ports{i + 1}];
    end
    set_param(p, 'BackgroundColor', mat2str(rgb));
end

function bsel(m, name, src, signals, pos)
    add_block('simulink/Signal Routing/Bus Selector', [m '/' name], 'OutputSignals', strjoin(signals, ','), ...
        'Position', pos);
    ln(m, src, [name '/1']);
end

function p = sig(bs_name, signals, s)
    idx = find(strcmp(signals, s));
    assert(isscalar(idx), 'signal %s not selected on %s', s, bs_name);
    p = sprintf('%s/%d', bs_name, idx);
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
    'function truth = Battery_Physics_Plant(I_load, SOC0)\n' ...
    '%%#codegen\n' ...
    '%% 2-RC Arrhenius cell with capacity/resistance mismatch (see bms_params). Algorithm: lib/bms_plant_step.m\n' ...
    '%% Output: bus BMS_PlantTruth\n' ...
    'P = coder.const(bms_params(''%s''));\n' ...
    'persistent x k;\n' ...
    'if isempty(x)\n' ...
    '    x = [SOC0; 0.0; 0.0];  k = 0.0;\n' ...
    'end\n' ...
    'k = k + 1.0;\n' ...
    'T = bms_temp_profile(k);\n' ...
    '[x, Vt] = bms_plant_step(x, I_load, T, P);\n' ...
    'truth.SOC_pct = 100.0 * x(1);\n' ...
    'truth.Voltage_V = Vt;\n' ...
    'truth.Temperature_K = T;\n'], variant);
end

function code = sensor_code(variant)
    code = sprintf([ ...
    'function meas = Sensor_Measurement(truth, I_raw, Fault_Mode, Seed)\n' ...
    '%%#codegen\n' ...
    '%% Noisy 12-bit-class sensors with injectable faults. Algorithm: lib/bms_sensor_step.m (seeded, reproducible)\n' ...
    '%% Input: bus BMS_PlantTruth. Output: bus BMS_Measurement\n' ...
    'P = coder.const(bms_params(''%s''));\n' ...
    'persistent ss k;\n' ...
    'if isempty(ss)\n' ...
    '    ss = bms_seed(Seed);  k = 0.0;\n' ...
    'end\n' ...
    'k = k + 1.0;\n' ...
    '[Vm, Im, Tm, ss] = bms_sensor_step(truth.Voltage_V, I_raw, truth.Temperature_K, k, ss, P, Fault_Mode);\n' ...
    'meas.Voltage_V = Vm;\n' ...
    'meas.Current_A = Im;\n' ...
    'meas.Temperature_K = Tm;\n'], variant);
end

function code = ctrl_code(variant, mode, fname)
    code = sprintf([ ...
    'function est = %s(meas, Q_diag, R_var, SOC_guess)\n' ...
    '%%#codegen\n' ...
    '%% 3-state Joseph-form EKF + diagnostics. Algorithm: lib/bms_est_step.m (estimator mode %d)\n' ...
    '%% Input: bus BMS_Measurement. Output: bus BMS_Estimate\n' ...
    'P = coder.const(bms_params(''%s''));\n' ...
    'persistent st;\n' ...
    'if isempty(st)\n' ...
    '    st = bms_est_init(P, SOC_guess, Q_diag, R_var, %d);\n' ...
    'end\n' ...
    '[st, o] = bms_est_step(st, meas.Current_A, meas.Voltage_V, meas.Temperature_K, P, %d);\n' ...
    'est.SOC_pct = o.soc;\n' ...
    'est.Voltage_V = o.vest;\n' ...
    'est.Innovation_V = o.innov;\n' ...
    'est.PTrace = o.ptrace;\n' ...
    'est.NIS = o.nis;\n' ...
    'est.FaultCode = o.fault;\n' ...
    'est.R0_Ohm = o.r0;\n' ...
    'est.SOC_Coulomb_pct = o.soc_cc;\n' ...
    'est.R_cov_V2 = o.rcov;\n' ...
    'est.CapacityRatio = o.cap;\n'], ...
    fname, mode, variant, mode, mode);
end
