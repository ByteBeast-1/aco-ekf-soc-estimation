function sc = bms_scenario(name, varargin)
%BMS_SCENARIO Unified scenario generator for synthetic and NASA replay runs.
%   sc = bms_scenario(name, ...) returns a scenario struct with fields:
%     name           : string identifier
%     source         : 'synthetic' | 'nasa'
%     I              : 1xN current vector [A, discharge positive]
%     V_ref          : 1xN reference/measured terminal voltage [V]
%     T              : 1xN temperature vector [K]
%     soc_ref        : 1xN reference state of charge [0..1 fraction]
%     soc0           : scalar true initial state of charge [0..1 fraction]
%     init_err       : scalar initial estimator offset (e.g. -0.10)
%     seed           : scalar random seed for sensor noise
%     fault_mode     : scalar fault mode (0: none, 1: I-offset, 2: V-spike, 3: T-open)
%     params_variant : 'synthetic' | 'nasa'
%
%   sc_list = bms_scenario('list') returns cell array of standard scenario structs.
%
%   Optional Name-Value overrides:
%     'init_err',   val
%     'seed',       val
%     'fault_mode', val

    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, 'lib'));

    if nargin < 1 || isempty(name)
        name = 'udds';
    end

    if strcmpi(name, 'list')
        sc = default_scenario_list();
        return;
    end

    % Parse optional overrides
    p = inputParser;
    addParameter(p, 'init_err',   []);
    addParameter(p, 'seed',       1);
    addParameter(p, 'fault_mode', 0);
    parse(p, varargin{:});
    opts = p.Results;

    % Detect if synthetic or NASA replay
    name_clean = lower(strrep(name, 'synthetic_', ''));
    if any(strcmp(name_clean, {'udds', 'cc', 'pulse', 'cccv'})) || startsWith(name_clean, 'fault')
        sc = build_synthetic_scenario(name_clean, opts);
    else
        sc = build_nasa_scenario(name, opts);
    end
end

%% ======================================================================
function sc = build_synthetic_scenario(name_clean, opts)
    if startsWith(name_clean, 'fault1')
        prof = 'udds';  fault = 1;
    elseif startsWith(name_clean, 'fault2')
        prof = 'udds';  fault = 2;
    elseif startsWith(name_clean, 'fault3')
        prof = 'udds';  fault = 3;
    else
        prof = name_clean;
        fault = opts.fault_mode;
    end

    [I, soc0] = bms_profile(prof);
    N = numel(I);
    P = bms_params('synthetic');

    % Reference trajectories from clean plant simulation
    x = [soc0; 0.0; 0.0];
    soc_ref = zeros(1, N);
    V_ref   = zeros(1, N);
    T       = zeros(1, N);
    for k = 1:N
        Tk = bms_temp_profile(k);
        [x, Vt] = bms_plant_step(x, I(k), Tk, P);
        soc_ref(k) = x(1);
        V_ref(k)   = Vt;
        T(k)       = Tk;
    end

    if isempty(opts.init_err)
        init_err = -0.10;
    else
        init_err = opts.init_err;
    end

    sc.name           = sprintf('synthetic_%s', name_clean);
    sc.source         = 'synthetic';
    sc.I              = I(:)';
    sc.V_ref          = V_ref;
    sc.T              = T;
    sc.soc_ref        = soc_ref;
    sc.soc0           = soc0;
    sc.init_err       = init_err;
    sc.seed           = opts.seed;
    sc.fault_mode     = fault;
    sc.params_variant = 'synthetic';
end

%% ======================================================================
function sc = build_nasa_scenario(name, opts)
    % Parse e.g. 'B0005_1', 'B0005 #1', 'B0005_charge_1', 'B0007_120'
    kind = 'discharge';
    if contains(lower(name), 'charge')
        kind = 'charge';
    end

    tokens = regexp(name, '(B000[567])[_#\s]*([0-9]+)', 'tokens');
    if isempty(tokens)
        cellname = 'B0005';
        dis_idx = 1;
    else
        cellname = tokens{1}{1};
        dis_idx  = str2double(tokens{1}{2});
    end

    if isempty(opts.init_err)
        init_err = -0.15;
    else
        init_err = opts.init_err;
    end

    sc_rep = bms_replay_scenario(cellname, dis_idx, init_err, kind);

    sc.name           = sprintf('nasa_%s_%s_%d', cellname, kind, dis_idx);
    sc.source         = 'nasa';
    sc.I              = sc_rep.I(:)';
    sc.V_ref          = sc_rep.V(:)';
    sc.T              = sc_rep.T(:)';
    sc.soc_ref        = sc_rep.soc_ref(:)';
    sc.soc0           = sc_rep.soc_ref(1);
    sc.init_err       = init_err;
    sc.seed           = opts.seed;
    sc.fault_mode     = opts.fault_mode;
    sc.params_variant = 'nasa';
end

%% ======================================================================
function list = default_scenario_list()
    names = { ...
        'synthetic_udds', ...
        'synthetic_cc', ...
        'synthetic_pulse', ...
        'synthetic_cccv', ...
        'synthetic_fault1', ...
        'synthetic_fault2', ...
        'synthetic_fault3', ...
        'B0005_1', ...
        'B0005_60', ...
        'B0005_120', ...
        'B0007_1', ...
        'B0007_60', ...
        'B0007_120', ...
        'B0006_120', ...
        'B0005_charge_1', ...
        'B0007_charge_1' ...
    };
    list = cell(1, numel(names));
    for i = 1:numel(names)
        list{i} = bms_scenario(names{i});
    end
end
