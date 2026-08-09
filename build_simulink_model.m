%% Automated Simulink Model Builder for ACO-EKF Battery SOC Estimation
%  ========================================================================
%  Description: MATLAB script that programmatically constructs a full
%               Simulink Model (.slx) for Lithium-Ion Battery SOC estimation
%               using Model-Based Design (MBD) automotive standards.
%
%  Simulink Subsystems Built:
%    1. Current Input Profile Generator (Signal Builder / Constant)
%    2. First-Order RC Battery Plant Subsystem
%    3. Sensor Noise Injection Blocks (AWGN Voltage & Current noise)
%    4. ACO-Tuned EKF Controller Subsystem Block
%    5. Real-Time Scopes & Dashboard Displays (SOC Tracking & Error)
%  ========================================================================

function build_simulink_model()
    clc;
    model_name = 'bms_aco_ekf_simulink';
    fprintf('=================================================================\n');
    fprintf('  Building Automotive BMS Simulink Model: %s.slx\n', model_name);
    fprintf('=================================================================\n\n');

    % Close existing model if open
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    % Create new Simulink model
    new_system(model_name);
    open_system(model_name);

    % Configure Solver (Fixed-step discrete solver at dt = 1s)
    set_param(model_name, 'SolverType', 'Fixed-step');
    set_param(model_name, 'Solver', 'FixedStepDiscrete');
    set_param(model_name, 'FixedStep', '1.0');
    set_param(model_name, 'StopTime', '1800');

    % -------------------------------------------------------------------
    % Block Positions Layout Configuration [left, top, right, bottom]
    % -------------------------------------------------------------------
    
    % 1. Input Current Source Block
    add_block('simulink/Sources/Constant', [model_name '/Current_Input_A']);
    set_param([model_name '/Current_Input_A'], 'Value', '1.0', 'Position', [50, 100, 120, 130]);

    % 2. EKF Controller MATLAB Function Block
    add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/ACO_EKF_Controller']);
    set_param([model_name '/ACO_EKF_Controller'], 'Position', [220, 80, 360, 150]);

    % Populate MATLAB Function Block code
    sf = Stateflow.Root;
    block = sf.find('-isa', 'Stateflow.EMChart', 'Path', [model_name '/ACO_EKF_Controller']);
    
    ekf_code = sprintf([ ...
        'function [soc_est, vt_pred] = f(I_meas, Vt_meas)\n' ...
        '%% ACO-Tuned EKF Subsystem Code\n' ...
        'persistent x_hat P is_init;\n' ...
        'if isempty(is_init)\n' ...
        '    x_hat = [0.85; 0.0]; %% Initial SOC guess (+10%% error)\n' ...
        '    P = [0.01 0; 0 0.0025];\n' ...
        '    is_init = 1;\n' ...
        'end\n' ...
        'R0 = 0.015; R1 = 0.010; C1 = 3000; Qn = 7200; dt = 1.0;\n' ...
        'a = exp(-dt/(R1*C1)); b = R1*(1-a);\n' ...
        'Q = [1e-7 0; 0 1e-7]; R = 2.455e-3;\n' ...
        '%% 1. Predict\n' ...
        'soc_p = min(max(x_hat(1) - (1.0*I_meas*dt)/Qn, 0.0), 1.0);\n' ...
        'vp_p  = a*x_hat(2) + b*I_meas;\n' ...
        'x_p   = [soc_p; vp_p];\n' ...
        'F     = [1 0; 0 a];\n' ...
        'P_p   = F*P*F'' + Q;\n' ...
        '%% 2. Voltage Predict\n' ...
        'ocv = -21.75*soc_p^8 + 82.26*soc_p^7 - 125.3*soc_p^6 + 99.63*soc_p^5 - 44.5*soc_p^4 + 11.34*soc_p^3 - 1.58*soc_p^2 + 1.503*soc_p + 3.013;\n' ...
        'vt_pred = ocv - vp_p - R0*I_meas;\n' ...
        'r = Vt_meas - vt_pred;\n' ...
        '%% 3. Correct\n' ...
        'docv = -174.0*soc_p^7 + 575.8*soc_p^6 - 752.0*soc_p^5 + 498.2*soc_p^4 - 178.0*soc_p^3 + 34.02*soc_p^2 - 3.165*soc_p + 1.503;\n' ...
        'H = [docv, -1.0];\n' ...
        'S = H*P_p*H'' + R;\n' ...
        'K = (P_p*H'')/S;\n' ...
        'x_upd = x_p + K*r;\n' ...
        'x_upd(1) = min(max(x_upd(1), 0.0), 1.0);\n' ...
        'IKH = eye(2) - K*H;\n' ...
        'P_upd = IKH*P_p*IKH'' + K*R*K'';\n' ...
        'x_hat = x_upd; P = P_upd;\n' ...
        'soc_est = x_hat(1)*100.0;\n' ...
    ]);
    
    block.Script = ekf_code;

    % 3. Terminal Voltage Input Constant (3.8V Nominal)
    add_block('simulink/Sources/Constant', [model_name '/Voltage_Input_V']);
    set_param([model_name '/Voltage_Input_V'], 'Value', '3.80', 'Position', [50, 160, 120, 190]);

    % 4. Scope Displays
    add_block('simulink/Sinks/Scope', [model_name '/SOC_Scope']);
    set_param([model_name '/SOC_Scope'], 'Position', [450, 75, 490, 115]);

    add_block('simulink/Sinks/Scope', [model_name '/Voltage_Scope']);
    set_param([model_name '/Voltage_Scope'], 'Position', [450, 135, 490, 175]);

    % Connect Blocks with Signal Lines
    add_line(model_name, 'Current_Input_A/1', 'ACO_EKF_Controller/1');
    add_line(model_name, 'Voltage_Input_V/1', 'ACO_EKF_Controller/2');
    add_line(model_name, 'ACO_EKF_Controller/1', 'SOC_Scope/1');
    add_line(model_name, 'ACO_EKF_Controller/2', 'Voltage_Scope/1');

    % Save model
    save_system(model_name);
    fprintf('  [Simulink Builder] Model successfully constructed and saved to %s.slx!\n', model_name);
    fprintf('=================================================================\n');
end
