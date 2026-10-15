function build_bms_models(which)
%BUILD_BMS_MODELS Programmatically (re)build the BMS Simulink models from the shared library.
%   build_bms_models()            builds the data dictionary, referenced estimator, legacy models, and unified model
%   build_bms_models('synthetic') advanced_bms_aco_ekf : simulated plant + sensors + referenced EKF
%   build_bms_models('nasa')      sota_bms_aco_ekf     : replay of REAL NASA cell data + 3 referenced estimators
%   build_bms_models('unified')   bms_unified          : unified model with Variant Subsystem source (SIM/REPLAY)
%   build_bms_models('ref')       bms_estimator_ref    : shared referenced estimator model
%
%   All algorithm code lives in lib/*.m (real files, unit-testable, codegen-ready).
%   bms_estimator_ref.slx is a referenced model shared by both legacy models and bms_unified.slx.
%   Signals between blocks travel on bus objects (BMS_PlantTruth, BMS_Measurement, BMS_Estimate, BMS_Reference)
%   defined with units in bms_data.sldd. Strict diagnostics make dangling ports, algebraic loops and bus
%   mismatches build errors.
    if nargin < 1,  which = 'all';  end
    here = fileparts(mfilename('fullpath'));
    addpath(here);  addpath(fullfile(here, 'lib'));
    bms_build_dictionary();

    if any(strcmp(which, {'all', 'ref', 'synthetic', 'nasa', 'unified'}))
        build_estimator_ref(here);
    end
    if any(strcmp(which, {'all', 'synthetic'}))
        build_synthetic(here);
    end
    if any(strcmp(which, {'all', 'nasa'}))
        build_nasa(here);
    end
    if any(strcmp(which, {'all', 'unified'}))
        build_unified(here);
    end
end

%% ======================================================================
%  1. Shared Referenced Estimator Model
%% ======================================================================
function build_estimator_ref(here)
    m = 'bms_estimator_ref';
    fprintf('Building %s ...\n', m);
    if bdIsLoaded(m),  close_system(m, 0);  end
    f = fullfile(here, [m '.slx']);
    if isfile(f),  delete(f);  end
    new_system(m);
    save_system(m, f);
    set_param(m, 'DataDictionary', 'bms_data.sldd');
    set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '1', ...
        'ModelReferenceNumInstancesAllowed', 'Multi');
    strict = {'UnconnectedInputMsg', 'error'; 'UnconnectedOutputMsg', 'error'; 'UnconnectedLineMsg', 'error'; ...
              'AlgebraicLoopMsg', 'error'; 'ArtificialAlgebraicLoopMsg', 'error'; 'BusObjectLabelMismatch', 'error'; ...
              'StrictBusMsg', 'ErrorOnBusTreatedAsVector'; 'SignalInfNanChecking', 'error'};
    for i = 1:size(strict, 1)
        try,  set_param(m, strict{i, 1}, strict{i, 2});  catch,  end
    end

    % Inports with explicit data types and dimensions
    add_block('simulink/Sources/In1', [m '/meas'], 'Position', [40 50 70 65], ...
        'OutDataTypeStr', 'Bus: BMS_Measurement', 'PortDimensions', '1');
    add_block('simulink/Sources/In1', [m '/Q_diag'], 'Position', [40 100 70 115], ...
        'OutDataTypeStr', 'double', 'PortDimensions', '3');
    add_block('simulink/Sources/In1', [m '/R_var'], 'Position', [40 150 70 165], ...
        'OutDataTypeStr', 'double', 'PortDimensions', '1');
    add_block('simulink/Sources/In1', [m '/SOC_guess'], 'Position', [40 200 70 215], ...
        'OutDataTypeStr', 'double', 'PortDimensions', '1');
    add_block('simulink/Sources/In1', [m '/Mode'], 'Position', [40 250 70 265], ...
        'OutDataTypeStr', 'double', 'PortDimensions', '1');
    add_block('simulink/Sources/In1', [m '/Variant_Tag'], 'Position', [40 300 70 315], ...
        'OutDataTypeStr', 'double', 'PortDimensions', '1');

    p = [m '/bms_estimator_core'];
    add_block('simulink/User-Defined Functions/MATLAB Function', p, 'Position', [160 40 340 325]);
    save_system(m);
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', '-and', 'Path', p);
    code = sprintf([ ...
    'function est = bms_estimator_core(meas, Q_diag, R_var, SOC_guess, mode, variant_tag)\n' ...
    '%%#codegen\n' ...
    '%% Multi-mode 4-state Joseph-form EKF core wrapped for Model Reference.\n' ...
    'P_syn  = coder.const(bms_params(''synthetic''));\n' ...
    'P_nasa = coder.const(bms_params(''nasa''));\n' ...
    'if variant_tag > 0.5\n' ...
    '    P = P_nasa;\n' ...
    'else\n' ...
    '    P = P_syn;\n' ...
    'end\n' ...
    'persistent st;\n' ...
    'if isempty(st)\n' ...
    '    st = bms_est_init(P, SOC_guess, Q_diag, R_var, mode);\n' ...
    'end\n' ...
    '[st, o] = bms_est_step(st, meas.Current_A, meas.Voltage_V, meas.Temperature_K, P, mode);\n' ...
    'est.SOC_pct = o.soc;\n' ...
    'est.Voltage_V = o.vest;\n' ...
    'est.Innovation_V = o.innov;\n' ...
    'est.PTrace = o.ptrace;\n' ...
    'est.NIS = o.nis;\n' ...
    'est.FaultCode = o.fault;\n' ...
    'est.R0_Ohm = o.r0;\n' ...
    'est.SOC_Coulomb_pct = o.soc_cc;\n' ...
    'est.R_cov_V2 = o.rcov;\n' ...
    'est.CapacityRatio = o.cap;\n']);
    ch.Script = code;
    d_meas = ch.find('-isa', 'Stateflow.Data', '-and', 'Name', 'meas');
    d_meas.DataType = 'Bus: BMS_Measurement';
    d_est = ch.find('-isa', 'Stateflow.Data', '-and', 'Name', 'est');
    d_est.DataType = 'Bus: BMS_Estimate';

    add_block('simulink/Sinks/Out1', [m '/est'], 'Position', [420 175 450 190], ...
        'OutDataTypeStr', 'Bus: BMS_Estimate');

    ln(m, 'meas/1', 'bms_estimator_core/1');
    ln(m, 'Q_diag/1', 'bms_estimator_core/2');
    ln(m, 'R_var/1', 'bms_estimator_core/3');
    ln(m, 'SOC_guess/1', 'bms_estimator_core/4');
    ln(m, 'Mode/1', 'bms_estimator_core/5');
    ln(m, 'Variant_Tag/1', 'bms_estimator_core/6');
    ln(m, 'bms_estimator_core/1', 'est/1');

    finish(m, here);
end

%% ======================================================================
%  2. Legacy Synthetic Model (advanced_bms_aco_ekf)
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
    const(m, 'Estimator_Mode',    '0',             [680 430 730 450]);
    const(m, 'Variant_Tag',       '0',             [680 470 730 490]);

    % --- plant -> sensors -> referenced estimator --------------------------
    mfb(m, 'Battery_Physics_Plant', [260 130 420 260], plant_code('synthetic'),  [0.80 0.90 1.00], ...
        {'truth', 'BMS_PlantTruth'});
    mfb(m, 'Sensor_Measurement',    [500 130 660 290], sensor_code('synthetic'), [1.00 0.90 0.75], ...
        {'truth', 'BMS_PlantTruth', 'meas', 'BMS_Measurement'});

    add_block('simulink/Ports & Subsystems/Model', [m '/ACO_EKF_Controller'], 'ModelName', 'bms_estimator_ref', ...
        'Position', [760 130 960 400]);

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
    ln(m, 'Estimator_Mode/1',       'ACO_EKF_Controller/5');
    ln(m, 'Variant_Tag/1',          'ACO_EKF_Controller/6');

    % --- bus selectors: pick signals off the buses ------------------------
    TR = {'SOC_pct', 'Voltage_V', 'Temperature_K'};
    MS = {'Voltage_V'};
    ES = {'SOC_pct', 'SOC_Coulomb_pct', 'Voltage_V', 'Innovation_V', 'PTrace', 'NIS', 'FaultCode'};
    bsel(m, 'BS_Estimate', 'ACO_EKF_Controller/1',    ES, [1060  80 1065 420]);
    bsel(m, 'BS_Truth',    'Battery_Physics_Plant/1', TR, [1060 460 1065 600]);
    bsel(m, 'BS_Measured', 'Sensor_Measurement/1',    MS, [1060 640 1065 680]);
    t  = @(s) sig('BS_Truth', TR, s);
    me = @(s) sig('BS_Measured', MS, s);
    e  = @(s) sig('BS_Estimate', ES, s);

    % --- scopes ------------------------------------------------------------
    x0 = 1260;
    mux_scope(m, 'SOC_Comparison_Scope', {t('SOC_pct'), e('SOC_pct'), e('SOC_Coulomb_pct')}, ...
        [x0 50], 'True SOC | EKF SOC | Coulomb counting');
    sub_scope(m, 'SOC_Error_Scope', e('SOC_pct'), t('SOC_pct'), [x0 145], 'EKF - True [%]');
    mux_scope(m, 'Voltage_Tracking_Scope', {t('Voltage_V'), me('Voltage_V'), e('Voltage_V')}, ...
        [x0 240], 'True V | Measured V | EKF predicted V');
    scope(m, 'EKF_Covariance_Scope', e('PTrace'), [x0 335], 'trace(P)');
    scope(m, 'Innovation_NIS_Scope', e('NIS'), [x0 430], 'Normalised innovation squared (expect ~1)');
    scope(m, 'Diagnostic_Fault_Code_Scope', e('FaultCode'), [x0 525], ...
        '1 V range | 2 T range | 4 I range | 8 innovation rejected | 16 filter inconsistent');
    scope(m, 'Drive_Current_Scope', 'Drive_Cycle_Current/1', [x0 620], 'Current [A]');
    scope(m, 'Temperature_Scope', t('Temperature_K'), [x0 715], 'Cell temperature [K]');

    add_block('simulink/Sinks/Display', [m '/Live_SOC_Display'], 'Position', [x0 + 100 45 x0 + 190 80]);
    ln(m, e('SOC_pct'), 'Live_SOC_Display/1');

    % --- machine-readable log in clean open corridor -----------------------
    names = {t('SOC_pct'), e('SOC_pct'), e('SOC_Coulomb_pct'), t('Voltage_V'), me('Voltage_V'), e('Voltage_V'), ...
             e('Innovation_V'), e('PTrace'), e('NIS'), e('FaultCode'), t('Temperature_K')};
    logger(m, names, [1140 780]);

    note(m, sprintf(['Advanced BMS: 2-RC thermal plant -> noisy/faultable sensors -> referenced Joseph EKF.\n' ...
                     'Algorithms: lib/bms_*.m. Signal interfaces: bus objects in bms_data.sldd.\n' ...
                     'Uses referenced model bms_estimator_ref.slx.']), [40 30]);
    finish(m, here);
end

%% ======================================================================
%  3. Legacy NASA Replay Model (sota_bms_aco_ekf)
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
    const(m, 'Mode_Baseline',     '0',              [340 310 380 330]);
    const(m, 'Mode_Adaptive',     '2',              [340 570 380 590]);
    const(m, 'Mode_Capacity',     '10',             [340 830 380 850]);
    const(m, 'NASA_Variant_Tag',  '1',              [340 870 380 890]);

    % Demux: 1 current, 2 measured voltage, 3 temperature [K], 4 reference SOC -> measurement bus
    add_block('simulink/Signal Routing/Bus Creator', [m '/Measurement_Bus'], 'Inputs', '3', ...
        'UseBusObject', 'on', 'BusObject', 'BMS_Measurement', 'Position', [300 120 305 180]);
    lnn(m, 'Demux_Replay/2', 'Measurement_Bus/1', 'Voltage_V');
    lnn(m, 'Demux_Replay/1', 'Measurement_Bus/2', 'Current_A');
    lnn(m, 'Demux_Replay/3', 'Measurement_Bus/3', 'Temperature_K');

    add_block('simulink/Ports & Subsystems/Model', [m '/EKF_Baseline'], 'ModelName', 'bms_estimator_ref', ...
        'Position', [400 120 600 340]);
    add_block('simulink/Ports & Subsystems/Model', [m '/SOTA_Adaptive_Controller'], 'ModelName', 'bms_estimator_ref', ...
        'Position', [400 380 600 600]);
    add_block('simulink/Ports & Subsystems/Model', [m '/SOH_Capacity_Estimator'], 'ModelName', 'bms_estimator_ref', ...
        'Position', [400 640 600 860]);

    for c = {'EKF_Baseline', 'SOTA_Adaptive_Controller', 'SOH_Capacity_Estimator'}
        ln(m, 'Measurement_Bus/1',   [c{1} '/1']);
        ln(m, 'ACO_Q_diag/1',        [c{1} '/2']);
        ln(m, 'ACO_R/1',             [c{1} '/3']);
        ln(m, 'Initial_SOC_Guess/1', [c{1} '/4']);
        ln(m, 'NASA_Variant_Tag/1',  [c{1} '/6']);
    end
    ln(m, 'Mode_Baseline/1', 'EKF_Baseline/5');
    ln(m, 'Mode_Adaptive/1', 'SOTA_Adaptive_Controller/5');
    ln(m, 'Mode_Capacity/1', 'SOH_Capacity_Estimator/5');

    EK = {'SOC_pct', 'SOC_Coulomb_pct'};
    SA = {'SOC_pct', 'Voltage_V', 'Innovation_V', 'NIS', 'FaultCode', 'R0_Ohm', 'R_cov_V2'};
    SH = {'SOC_pct', 'CapacityRatio'};
    bsel(m, 'BS_Baseline', 'EKF_Baseline/1',             EK, [670 160 675 240]);
    bsel(m, 'BS_Adaptive', 'SOTA_Adaptive_Controller/1', SA, [670 300 675 620]);
    bsel(m, 'BS_Capacity', 'SOH_Capacity_Estimator/1',   SH, [670 680 675 760]);
    eb = @(s) sig('BS_Baseline', EK, s);
    ea = @(s) sig('BS_Adaptive', SA, s);
    es = @(s) sig('BS_Capacity', SH, s);

    % --- scopes ordered to match signal output ports -----------------------
    x0 = 960;
    mux_scope(m, 'SOC_Comparison_Scope', {'Ref_SOC_Percent/1', eb('SOC_pct'), ea('SOC_pct'), eb('SOC_Coulomb_pct'), es('SOC_pct')}, ...
        [x0 50], 'Reference SOC | EKF | Adaptive EKF | Coulomb counting | EKF+capacity state');
    mux_sub(m, 'SOC_Error_Scope', {eb('SOC_pct'), ea('SOC_pct'), es('SOC_pct')}, 'Ref_SOC_Percent/1', [x0 155], ...
        'EKF error | Adaptive EKF error | EKF+capacity error [%]');
    mux_scope(m, 'Voltage_Tracking_Scope', {'Demux_Replay/2', ea('Voltage_V')}, [x0 260], ...
        'Measured V (real cell) | Adaptive EKF predicted V');
    scope(m, 'Innovation_NIS_Scope', ea('NIS'), [x0 370], 'NIS (expect ~1)');
    scope(m, 'Diagnostic_Fault_Code_Scope', ea('FaultCode'), [x0 480], 'Diagnostic bit-mask');
    scope(m, 'R0_Tracking_Scope', ea('R0_Ohm'), [x0 590], 'R0 used by adaptive model [ohm]');
    scope(m, 'R_Adaptation_Scope', ea('R_cov_V2'), [x0 700], 'Sage-Husa measurement-noise variance R');
    mux_scope(m, 'Capacity_SOH_Scope', {es('CapacityRatio'), 'True_Capacity_Ratio/1'}, [x0 805], ...
        'Estimated capacity ratio (SOH) | true capacity ratio');
    add_block('simulink/Sinks/Display', [m '/Live_SOC_Display'], 'Position', [x0 + 100 45 x0 + 190 80]);
    ln(m, ea('SOC_pct'), 'Live_SOC_Display/1');

    % Logger placed cleanly below bus selectors
    names = {'Ref_SOC_Percent/1', eb('SOC_pct'), ea('SOC_pct'), eb('SOC_Coulomb_pct'), 'Demux_Replay/2', ...
             ea('Voltage_V'), ea('Innovation_V'), ea('NIS'), ea('FaultCode'), ea('R0_Ohm'), ea('R_cov_V2'), ...
             es('SOC_pct'), es('CapacityRatio')};
    logger(m, names, [750 860]);

    note(m, sprintf(['Real-data BMS: replays measured NASA Ames discharge.\n' ...
                     'Estimators use referenced model bms_estimator_ref.slx.\n' ...
                     'Top: Baseline EKF | Middle: Adaptive EKF | Bottom: Joint SOH/SOC Estimator.']), [40 30]);
    finish(m, here);
end

%% ======================================================================
%  4. Unified Model (bms_unified.slx)
%% ======================================================================
function build_unified(here)
    m = 'bms_unified';
    fprintf('Building %s ...\n', m);
    bms_setup('unified', true);
    new_model(m, here, 'BMS_StopTime', 'bms_setup(''unified'')');

    % --- Sources / Constants -----------------------------------------------
    add_block('simulink/Sources/From Workspace', [m '/Drive_Cycle_Current'], 'VariableName', 'Drive_Profile', ...
        'SampleTime', '1', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [40 80 150 110]);
    add_block('simulink/Sources/From Workspace', [m '/NASA_Cell_Replay'], 'VariableName', 'NASA_Replay', ...
        'SampleTime', '1', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [40 130 150 160]);
    const(m, 'Initial_SOC_True',           'BMS_SOC0',                 [40 180 150 205]);
    const(m, 'Fault_Mode',                 'BMS_FaultMode',            [40 220 150 245]);
    const(m, 'Noise_Seed',                 'BMS_Seed',                 [40 260 150 285]);
    const(m, 'Inject_Faults_On_Replay',    'BMS_InjectFaultsOnReplay', [40 300 150 325]);

    % Parameter routing based on BMS_Source (1 = SIM, 2 = REPLAY)
    const(m, 'BMS_Source_Param',           'BMS_Source',               [40 350 150 375]);
    const(m, 'SIM_Q_diag',                 'BMS_Q',                    [40 400 150 425]);
    const(m, 'NASA_Q_diag',                'NASA_Q',                   [40 440 150 465]);
    const(m, 'SIM_R_var',                  'BMS_R',                    [40 480 150 505]);
    const(m, 'NASA_R_var',                 'NASA_R',                   [40 520 150 545]);
    const(m, 'SIM_SOC_Guess',              'BMS_SOC_Guess',            [40 560 150 585]);
    const(m, 'NASA_SOC_Guess',             'NASA_SOC_Guess',           [40 600 150 625]);
    const(m, 'SIM_Variant_Tag',            '0',                        [40 640 150 665]);
    const(m, 'NASA_Variant_Tag',           '1',                        [40 680 150 705]);

    % Switches for parameter routing (Threshold = 1.5, port 1 when > 1.5 [NASA], port 3 when <= 1.5 [SIM])
    add_block('simulink/Signal Routing/Switch', [m '/Switch_Q'], 'Threshold', '1.5', 'Position', [220 410 250 450]);
    ln(m, 'NASA_Q_diag/1',      'Switch_Q/1');
    ln(m, 'BMS_Source_Param/1', 'Switch_Q/2');
    ln(m, 'SIM_Q_diag/1',       'Switch_Q/3');

    add_block('simulink/Signal Routing/Switch', [m '/Switch_R'], 'Threshold', '1.5', 'Position', [220 490 250 530]);
    ln(m, 'NASA_R_var/1',       'Switch_R/1');
    ln(m, 'BMS_Source_Param/1', 'Switch_R/2');
    ln(m, 'SIM_R_var/1',        'Switch_R/3');

    add_block('simulink/Signal Routing/Switch', [m '/Switch_Guess'], 'Threshold', '1.5', 'Position', [220 570 250 610]);
    ln(m, 'NASA_SOC_Guess/1',   'Switch_Guess/1');
    ln(m, 'BMS_Source_Param/1', 'Switch_Guess/2');
    ln(m, 'SIM_SOC_Guess/1',    'Switch_Guess/3');

    add_block('simulink/Signal Routing/Switch', [m '/Switch_Tag'], 'Threshold', '1.5', 'Position', [220 650 250 690]);
    ln(m, 'NASA_Variant_Tag/1', 'Switch_Tag/1');
    ln(m, 'BMS_Source_Param/1', 'Switch_Tag/2');
    ln(m, 'SIM_Variant_Tag/1',  'Switch_Tag/3');

    % --- Input Source Variant Subsystem ------------------------------------
    p_vs = [m '/Input_Source_Variant'];
    add_block('simulink/Ports & Subsystems/Variant Subsystem', p_vs, 'Position', [200 80 440 330]);

    % VS already has In1 and Out1. Add In2..In6 and Out2 to VS:
    for i = 2:6
        add_block('simulink/Sources/In1', sprintf('%s/In%d', p_vs, i), 'Position', [40, 40 * i, 70, 40 * i + 15]);
    end
    add_block('simulink/Sinks/Out1', [p_vs '/Out2'], 'Position', [600, 150, 630, 165]);
    set_param([p_vs '/Out1'], 'UseBusObject', 'on', 'BusObject', 'BMS_Measurement');
    set_param([p_vs '/Out2'], 'UseBusObject', 'on', 'BusObject', 'BMS_Reference');

    % Choice 1: SIM (renamed from default Subsystem)
    p_sim = [p_vs '/Subsystem'];
    set_param(p_sim, 'Name', 'Choice_SIM');
    p_sim = [p_vs '/Choice_SIM'];
    set_param(p_sim, 'VariantControl', 'BMS_Source == 1');
    build_choice_sim(p_sim);

    % Choice 2: REPLAY (renamed from default Subsystem1)
    p_rep = [p_vs '/Subsystem1'];
    set_param(p_rep, 'Name', 'Choice_REPLAY');
    p_rep = [p_vs '/Choice_REPLAY'];
    set_param(p_rep, 'VariantControl', 'BMS_Source == 2');
    build_choice_replay(p_rep);

    % Connect top-level inputs to Input_Source_Variant
    ln(m, 'Drive_Cycle_Current/1',        'Input_Source_Variant/1');
    ln(m, 'NASA_Cell_Replay/1',          'Input_Source_Variant/2');
    ln(m, 'Initial_SOC_True/1',          'Input_Source_Variant/3');
    ln(m, 'Fault_Mode/1',                'Input_Source_Variant/4');
    ln(m, 'Noise_Seed/1',                'Input_Source_Variant/5');
    ln(m, 'Inject_Faults_On_Replay/1',   'Input_Source_Variant/6');

    % --- Estimator Bank (3 Referenced Models) -------------------------------
    const(m, 'Mode_Baseline', '0',  [500 185 540 205]);
    const(m, 'Mode_Adaptive', '2',  [500 405 540 425]);
    const(m, 'Mode_Capacity', '10', [500 625 540 645]);

    add_block('simulink/Ports & Subsystems/Model', [m '/EKF_Baseline'], 'ModelName', 'bms_estimator_ref', ...
        'Position', [580 80 780 240]);
    add_block('simulink/Ports & Subsystems/Model', [m '/SOTA_Adaptive_Controller'], 'ModelName', 'bms_estimator_ref', ...
        'Position', [580 300 780 460]);
    add_block('simulink/Ports & Subsystems/Model', [m '/SOH_Capacity_Estimator'], 'ModelName', 'bms_estimator_ref', ...
        'Position', [580 520 780 680]);

    for c = {'EKF_Baseline', 'SOTA_Adaptive_Controller', 'SOH_Capacity_Estimator'}
        ln(m, 'Input_Source_Variant/1', [c{1} '/1']);  % BMS_Measurement bus
        ln(m, 'Switch_Q/1',             [c{1} '/2']);
        ln(m, 'Switch_R/1',             [c{1} '/3']);
        ln(m, 'Switch_Guess/1',         [c{1} '/4']);
        ln(m, 'Switch_Tag/1',           [c{1} '/6']);
    end
    ln(m, 'Mode_Baseline/1', 'EKF_Baseline/5');
    ln(m, 'Mode_Adaptive/1', 'SOTA_Adaptive_Controller/5');
    ln(m, 'Mode_Capacity/1', 'SOH_Capacity_Estimator/5');

    % --- Bus Selectors & Scopes --------------------------------------------
    REF = {'SOC_pct', 'Source_Tag'};
    EK  = {'SOC_pct', 'SOC_Coulomb_pct'};
    SA  = {'SOC_pct', 'Voltage_V', 'Innovation_V', 'NIS', 'FaultCode', 'R0_Ohm', 'R_cov_V2'};
    SH  = {'SOC_pct', 'CapacityRatio'};
    MS  = {'Voltage_V'};

    add_block('simulink/Signal Routing/Bus Selector', [m '/BS_Reference'], 'OutputSignals', strjoin(REF, ','), ...
        'Position', [840  40 845 120]);
    ln(m, 'Input_Source_Variant/2', 'BS_Reference/1');

    bsel(m, 'BS_Baseline',  'EKF_Baseline/1',           EK,  [840 150 845 230]);
    bsel(m, 'BS_Adaptive',  'SOTA_Adaptive_Controller/1', SA, [840 280 845 620]);
    bsel(m, 'BS_Capacity',  'SOH_Capacity_Estimator/1',   SH, [840 670 845 750]);
    bsel(m, 'BS_Measured',  'Input_Source_Variant/1',   MS,  [840 790 845 830]);

    er = @(s) sig('BS_Reference', REF, s);
    eb = @(s) sig('BS_Baseline', EK, s);
    ea = @(s) sig('BS_Adaptive', SA, s);
    es = @(s) sig('BS_Capacity', SH, s);
    em = @(s) sig('BS_Measured', MS, s);

    % Scopes vertically aligned in matching signal order with generous spacing
    x0 = 1180;
    mux_scope(m, 'SOC_Comparison_Scope', {er('SOC_pct'), eb('SOC_pct'), ea('SOC_pct'), eb('SOC_Coulomb_pct'), es('SOC_pct')}, ...
        [x0 50], 'Reference SOC | EKF | Adaptive EKF | Coulomb Counting | EKF+capacity');
    mux_sub(m, 'SOC_Error_Scope', {eb('SOC_pct'), ea('SOC_pct'), es('SOC_pct')}, er('SOC_pct'), [x0 165], ...
        'EKF error | Adaptive EKF error | EKF+capacity error [%]');
    mux_scope(m, 'Voltage_Tracking_Scope', {em('Voltage_V'), ea('Voltage_V')}, [x0 280], ...
        'Measured V | Adaptive EKF predicted V');
    scope(m, 'Innovation_NIS_Scope', ea('NIS'), [x0 395], 'NIS (expect ~1)');
    scope(m, 'Diagnostic_Fault_Code_Scope', ea('FaultCode'), [x0 505], 'Diagnostic bit-mask');
    scope(m, 'R0_Tracking_Scope', ea('R0_Ohm'), [x0 615], 'R0 used by adaptive model [ohm]');
    scope(m, 'R_Adaptation_Scope', ea('R_cov_V2'), [x0 725], 'Sage-Husa measurement-noise variance R');
    scope(m, 'Capacity_SOH_Scope', es('CapacityRatio'), [x0 835], 'Estimated capacity ratio (SOH)');

    add_block('simulink/Sinks/Display', [m '/Live_SOC_Display'], 'Position', [x0 + 80 45 x0 + 170 80]);
    ln(m, er('SOC_pct'), 'Live_SOC_Display/1');

    % Logger (14 machine-readable columns) placed in clean corridor below bus selectors
    names = {er('SOC_pct'), eb('SOC_pct'), ea('SOC_pct'), eb('SOC_Coulomb_pct'), em('Voltage_V'), ...
             ea('Voltage_V'), ea('Innovation_V'), ea('NIS'), ea('FaultCode'), ea('R0_Ohm'), ea('R_cov_V2'), ...
             es('SOC_pct'), es('CapacityRatio'), er('Source_Tag')};
    logger(m, names, [920 880]);

    note(m, sprintf(['Unified BMS Model: connects synthetic simulation and real NASA replay via Variant Subsystem.\n' ...
                     'Input: BMS_Source (1: SIM, 2: REPLAY). Sensors: inject faults on replay via BMS_InjectFaultsOnReplay.\n' ...
                     'Estimators: 3 referenced models (bms_estimator_ref.slx).']), [40 30]);
    finish(m, here);
end

%% ======================================================================
%  Sub-helpers for bms_unified Variant Choices
%% ======================================================================
function build_choice_sim(p)
    % Choice SIM: Plant -> Sensors -> BMS_Measurement & BMS_Reference
    lines = get_param(p, 'Lines');
    for i = 1:numel(lines), if lines(i).Handle ~= -1, delete_line(lines(i).Handle); end; end

    % In1 and Out1 already exist in Subsystem
    set_param([p '/In1'],  'Position', [40  60 70  75]);
    set_param([p '/Out1'], 'Position', [640 115 670 135], 'UseBusObject', 'on', 'BusObject', 'BMS_Measurement');

    for i = 2:6
        add_block('simulink/Sources/In1', sprintf('%s/In%d', p, i), 'Position', [40, 20 + 40 * i, 70, 35 + 40 * i]);
    end
    add_block('simulink/Sinks/Out1', [p '/Out2'], 'Position', [640 265 670 285]);
    set_param([p '/Out2'], 'UseBusObject', 'on', 'BusObject', 'BMS_Reference');

    add_block('simulink/Sinks/Terminator', [p '/Term1'], 'Position', [100 100 120 115]);
    add_block('simulink/Sinks/Terminator', [p '/Term2'], 'Position', [100 260 120 275]);
    add_line(p, 'In2/1',   'Term1/1');  % NASA_Replay
    add_line(p, 'In6/1',   'Term2/1');  % Inject_Fault

    mfb_sub(p, 'Battery_Physics_Plant', [160 50 320 170], plant_code('synthetic'), [0.80 0.90 1.00], ...
        {'truth', 'BMS_PlantTruth'});
    mfb_sub(p, 'Sensor_Measurement',    [390 50 550 200], sensor_code('synthetic'), [1.00 0.90 0.75], ...
        {'truth', 'BMS_PlantTruth', 'meas', 'BMS_Measurement'});

    add_line(p, 'In1/1',                   'Battery_Physics_Plant/1'); % Drive current
    add_line(p, 'In3/1',                   'Battery_Physics_Plant/2'); % SOC0
    add_line(p, 'Battery_Physics_Plant/1', 'Sensor_Measurement/1');
    add_line(p, 'In1/1',                   'Sensor_Measurement/2', 'autorouting', 'on');
    add_line(p, 'In4/1',                   'Sensor_Measurement/3');    % Fault_Mode
    add_line(p, 'In5/1',                   'Sensor_Measurement/4');    % Seed
    add_line(p, 'Sensor_Measurement/1',    'Out1/1');

    % Reference bus builder
    add_block('simulink/Signal Routing/Bus Selector', [p '/BS_Truth'], 'OutputSignals', 'SOC_pct', ...
        'Position', [390 230 395 270]);
    add_line(p, 'Battery_Physics_Plant/1', 'BS_Truth/1');

    add_block('simulink/Sources/Constant', [p '/Source_Tag_0'], 'Value', '0.0', 'Position', [420 300 460 320], 'ShowName', 'off');
    add_block('simulink/Signal Routing/Bus Creator', [p '/Bus_Reference'], 'Inputs', '2', ...
        'UseBusObject', 'on', 'BusObject', 'BMS_Reference', 'Position', [520 230 525 320]);
    add_line(p, 'BS_Truth/1',      'Bus_Reference/1');
    h2 = add_line(p, 'Source_Tag_0/1',  'Bus_Reference/2');
    set_param(h2, 'Name', 'Source_Tag');
    add_line(p, 'Bus_Reference/1', 'Out2/1');
end

function build_choice_replay(p)
    % Choice REPLAY: NASA data -> Sensor Fault Injection -> BMS_Measurement & BMS_Reference
    lines = get_param(p, 'Lines');
    for i = 1:numel(lines), if lines(i).Handle ~= -1, delete_line(lines(i).Handle); end; end

    set_param([p '/In1'],  'Position', [40  60 70  75]);
    set_param([p '/Out1'], 'Position', [480 130 510 150], 'UseBusObject', 'on', 'BusObject', 'BMS_Measurement');

    for i = 2:6
        add_block('simulink/Sources/In1', sprintf('%s/In%d', p, i), 'Position', [40, 20 + 40 * i, 70, 35 + 40 * i]);
    end
    add_block('simulink/Sinks/Out1', [p '/Out2'], 'Position', [480 265 510 285]);
    set_param([p '/Out2'], 'UseBusObject', 'on', 'BusObject', 'BMS_Reference');

    add_block('simulink/Sinks/Terminator', [p '/Term1'], 'Position', [100 60 120 75]);
    add_block('simulink/Sinks/Terminator', [p '/Term2'], 'Position', [100 140 120 155]);
    add_block('simulink/Sinks/Terminator', [p '/Term3'], 'Position', [100 220 120 235]);
    add_line(p, 'In1/1', 'Term1/1'); % Drive current
    add_line(p, 'In3/1', 'Term2/1'); % SOC0
    add_line(p, 'In5/1', 'Term3/1'); % Seed

    add_block('simulink/Signal Routing/Demux', [p '/Demux_Replay'], 'Outputs', '4', 'Position', [140 70 145 170]);
    add_line(p, 'In2/1', 'Demux_Replay/1'); % NASA_Replay

    mfb_sub(p, 'Sensor_Replay_Fault', [240 70 400 210], replay_sensor_code(), [1.00 0.90 0.75], ...
        {'meas', 'BMS_Measurement'});
    add_line(p, 'Demux_Replay/1', 'Sensor_Replay_Fault/1');  % I
    add_line(p, 'Demux_Replay/2', 'Sensor_Replay_Fault/2');  % V
    add_line(p, 'Demux_Replay/3', 'Sensor_Replay_Fault/3');  % T
    add_line(p, 'In4/1',          'Sensor_Replay_Fault/4');  % fault mode
    add_line(p, 'In6/1',          'Sensor_Replay_Fault/5', 'autorouting', 'on');
    add_line(p, 'Sensor_Replay_Fault/1', 'Out1/1');

    % Reference bus builder
    add_block('simulink/Math Operations/Gain', [p '/Ref_SOC_Percent'], 'Gain', '100', 'Position', [240 245 275 275]);
    add_line(p, 'Demux_Replay/4', 'Ref_SOC_Percent/1');

    add_block('simulink/Sources/Constant', [p '/Source_Tag_1'], 'Value', '1.0', 'Position', [240 305 275 325], 'ShowName', 'off');
    add_block('simulink/Signal Routing/Bus Creator', [p '/Bus_Reference'], 'Inputs', '2', ...
        'UseBusObject', 'on', 'BusObject', 'BMS_Reference', 'Position', [350 240 355 330]);
    h1 = add_line(p, 'Ref_SOC_Percent/1', 'Bus_Reference/1');
    set_param(h1, 'Name', 'SOC_pct');
    h2 = add_line(p, 'Source_Tag_1/1',    'Bus_Reference/2');
    set_param(h2, 'Name', 'Source_Tag');
    add_line(p, 'Bus_Reference/1',   'Out2/1');
end

function code = replay_sensor_code()
    code = sprintf([ ...
    'function meas = Sensor_Replay_Fault(I_raw, V_raw, T_raw, Fault_Mode, Inject_Fault)\n' ...
    '%%#codegen\n' ...
    '%% Common sensor interface for REPLAY: passes real NASA signals cleanly unless\n' ...
    '%% BMS_InjectFaultsOnReplay is active.\n' ...
    'P = coder.const(bms_params(''nasa''));\n' ...
    'persistent k;\n' ...
    'if isempty(k),  k = 0.0;  end\n' ...
    'k = k + 1.0;\n' ...
    'Vm = V_raw;  Im = I_raw;  Tm = T_raw;\n' ...
    'if Inject_Fault > 0.5 && k >= P.fault_k0 && k <= P.fault_k1\n' ...
    '    if Fault_Mode == 1.0\n' ...
    '        Im = Im + P.fault_I_offset;\n' ...
    '    elseif Fault_Mode == 2.0\n' ...
    '        if mod(k, 25.0) == 0.0\n' ...
    '            Vm = Vm + P.fault_V_spike * (1.0 - 2.0 * mod(k / 25.0, 2.0));\n' ...
    '        end\n' ...
    '    elseif Fault_Mode == 3.0\n' ...
    '        Tm = P.fault_T_stuck;\n' ...
    '    end\n' ...
    'end\n' ...
    'meas.Voltage_V = Vm;\n' ...
    'meas.Current_A = Im;\n' ...
    'meas.Temperature_K = Tm;\n']);
end

%% ======================================================================
%  General Helpers
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
    for i = 1:2:numel(bus_ports)
        d = ch.find('-isa', 'Stateflow.Data', '-and', 'Name', bus_ports{i});
        d.DataType = ['Bus: ' bus_ports{i + 1}];
    end
    set_param(p, 'BackgroundColor', mat2str(rgb));
end

function mfb_sub(p_parent, name, pos, code, rgb, bus_ports)
    p = [p_parent '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', p, 'Position', pos);
    m = bdroot(p_parent);
    save_system(m);
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', '-and', 'Path', p);
    ch.Script = code;
    for i = 1:2:numel(bus_ports)
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
        'Position', [pos(1) - 60 pos(2) pos(1) - 55 pos(2) + 20 + 10 * numel(srcs)]);
    for i = 1:numel(srcs),  ln(m, srcs{i}, sprintf('%s/%d', mx, i));  end
    set_param([m '/' mx], 'ShowName', 'off');
    scope(m, name, [mx '/1'], [pos(1) + 20 pos(2)], desc);
end

function sub_scope(m, name, a, b, pos, desc)
    sb = ['Sub_' name];
    add_block('simulink/Math Operations/Subtract', [m '/' sb], 'Inputs', '+-', ...
        'Position', [pos(1) - 90 pos(2) pos(1) - 65 pos(2) + 30]);
    ln(m, a, [sb '/1']);  ln(m, b, [sb '/2']);
    set_param([m '/' sb], 'ShowName', 'off');
    scope(m, name, [sb '/1'], [pos(1) + 20 pos(2)], desc);
end

function mux_sub(m, name, ests, ref, pos, desc)
    mx = ['Mux_' name];
    add_block('simulink/Signal Routing/Mux', [m '/' mx], 'Inputs', num2str(numel(ests)), ...
        'Position', [pos(1) - 45 pos(2) pos(1) - 40 pos(2) + 30]);
    for i = 1:numel(ests)
        sb = sprintf('Sub_%s_%d', name, i);
        add_block('simulink/Math Operations/Subtract', [m '/' sb], 'Inputs', '+-', ...
            'Position', [pos(1) - 120 pos(2) + 35 * (i - 1) pos(1) - 95 pos(2) + 25 + 35 * (i - 1)]);
        ln(m, ests{i}, [sb '/1']);  ln(m, ref, [sb '/2']);
        ln(m, [sb '/1'], sprintf('%s/%d', mx, i));
        set_param([m '/' sb], 'ShowName', 'off');
    end
    set_param([m '/' mx], 'ShowName', 'off');
    scope(m, name, [mx '/1'], [pos(1) + 20 pos(2)], desc);
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
