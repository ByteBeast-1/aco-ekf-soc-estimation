function P = bms_params(variant, tuning)
%BMS_PARAMS Single source of truth for every cell/filter/sensor parameter.
%   P = BMS_PARAMS('synthetic')  2-RC cell used by the simulated plant
%   P = BMS_PARAMS('nasa')       cell calibrated on real NASA data (BMS_NASA_CAL_DATA)
%   P = BMS_PARAMS('nasa', 'cv_best') uses cross-validation optimal Q and R
%   Codegen-compatible (no file I/O); safe inside MATLAB Function blocks.
    if nargin < 1
        variant = 'synthetic';
    end
    if nargin < 2
        tuning = 'aco_shipped';
    end
    tuned = bms_tuned_data();

    % --- constants shared by both variants -------------------------------
    P.dt   = 1.0;      % [s]  fixed solver step
    P.eta  = 1.0;      % coulombic efficiency
    P.Rg   = 8.314;    % [J/mol/K]
    P.Ea   = 18000.0;  % [J/mol] Arrhenius activation energy
    P.Tref = 298.15;   % [K]

    if strcmp(variant, 'nasa')
        cal  = bms_nasa_cal_data();
        P.R0 = cal.R0;  P.R1 = cal.R1;  P.R2 = cal.R2;
        P.C1 = cal.C1;  P.C2 = cal.C2;  P.Qn = cal.Qn;
        P.ocv = cal.ocv;
        if strcmp(tuning, 'cv_best')
            % Cross-validation optimal parameters from leave-one-cell-out benchmark (ACO_R_Val_Pick)
            P.Q = [1.0622e-06; 3.2606e-05; 5.2942e-05];
            P.R = 2.1843e-03;
        else
            P.Q = tuned.nasa.Q;  P.R = tuned.nasa.R;
        end
        P.P0 = diag([0.0225, 0.0025, 0.0025]);   % 15 % SOC 1-sigma prior
    else
        P.R0 = 0.012;  P.R1 = 0.008;  P.R2 = 0.015;
        P.C1 = 1500.0; P.C2 = 12000.0; P.Qn = 7200.0;
        P.ocv = [-21.7497, 82.2625, -125.3296, 99.6313, -44.5010, ...
                  11.3416,  -1.5825,   1.5032,   3.0130];
        P.Q = tuned.synthetic.Q;  P.R = tuned.synthetic.R;
        P.P0 = diag([0.01, 0.0025, 0.0025]);
    end

    % --- plant/model mismatch (so the filter is NOT tested on its own model)
    P.plant_Qn_scale = 0.98;   % true capacity is 2 % below nominal
    P.plant_R_scale  = 1.10;   % true resistances are 10 % above nominal

    % --- sensors ----------------------------------------------------------
    P.sv = 0.008;  P.si = 0.010;  P.sT = 0.5;   % noise 1-sigma [V, A, K]
    P.adc_lsb = 0.001;                          % 1 mV voltage ADC

    % --- fault injection (window in samples) ------------------------------
    P.fault_k0 = 900;  P.fault_k1 = 1200;
    P.fault_I_offset = 0.25;     % [A]  mode 1: current-sensor offset
    P.fault_V_spike  = 0.5;      % [V]  mode 2: voltage glitches every 25 s
    P.fault_T_stuck  = 200.0;    % [K]  mode 3: thermistor open circuit

    % --- diagnostics / robustness -----------------------------------------
    P.V_valid = [2.0, 4.8];      % [V]  plausibility window
    P.T_valid = [250.0, 345.0];  % [K]
    P.I_max   = 10.0;            % [A]
    P.gate_k0  = 30;             % samples before innovation gating arms
    P.gate_nis = 20.0;           % chi2(1 dof) ~ 99.99 %
    P.max_rej  = 10;             % force-accept after N consecutive rejects
    P.nis_alpha = 0.02;          % consistency monitor EMA factor
    P.nis_limit = 5.0;           % expected mean NIS ~ 1
    P.ibias_k0  = 150;           % samples before current-bias monitor arms
    P.ibias_soc_min = 0.08;      % monitor valid window (SOC >= 8%)
    P.ibias_alpha = 0.02;        % innovation smoothing factor
    P.ibias_thresh = 0.0055;     % innovation threshold [V] (5.5 mV)
    P.ibias_count_limit = 20;    % persistence samples before flagging fault

    % --- adaptation (SOTA estimator) --------------------------------------
    P.sota_mode = 2;             % bit mask, see bms_est_step (ablation: only R-adaptation helps on real data)
    P.sh_d     = 0.02;           % Sage-Husa forgetting
    P.rls_lambda = 0.999;        % scalar-RLS forgetting for R0 scale
    P.rls_I_min  = 0.3;          % [A]  persistent-excitation gate
    P.rls_bounds = [0.5, 3.0];   % allowed R0 scale
    P.cap_P0 = 0.01;             % capacity-ratio prior variance (1-sigma 0.1; 0.04 was worse on every cycle)
    P.cap_Q  = 1e-8;             % capacity random-walk variance per step
    P.cap_bounds = [0.5, 1.2];   % allowed capacity ratio
    P.R_bounds = [1e-4, 5e-2];
    P.Q_bounds = [1e-9, 1e-4];

    P.variant = variant;
end
