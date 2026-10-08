function out = run_simulink(profile, fault_mode, init_err, seed)
%RUN_SIMULINK Run advanced_bms_aco_ekf (simulated 2-RC plant + sensors + ACO-tuned EKF) and plot the result.
%   out = run_simulink()                         UDDS cycle, no fault, -10 % initial SOC error, seed 1
%   out = run_simulink('pulse', 2, 0.15, 42)     pulse cycle with voltage glitches, +15 % start error
%
%   profile    : 'udds' | 'cc' | 'pulse' | 'cccv'
%   fault_mode : 0 none | 1 current-sensor offset | 2 voltage glitches | 3 thermistor open-circuit
%   init_err   : filter initial SOC minus true initial SOC (fraction)
    if nargin < 1,  profile    = 'udds';  end
    if nargin < 2,  fault_mode = 0;       end
    if nargin < 3,  init_err   = -0.10;   end
    if nargin < 4,  seed       = 1;       end
    here = fileparts(mfilename('fullpath'));
    addpath(here);  addpath(fullfile(here, 'lib'));
    model = 'advanced_bms_aco_ekf';
    if ~isfile(fullfile(here, [model '.slx'])),  build_bms_models('synthetic');  end

    [I, soc0] = bms_profile(profile);
    assignin('base', 'Drive_Profile', [(0:numel(I) - 1)', I(:)]);
    assignin('base', 'BMS_SOC0',      soc0);
    assignin('base', 'BMS_SOC_Guess', min(max(soc0 + init_err, 0), 1));
    assignin('base', 'BMS_Seed',      seed);
    assignin('base', 'BMS_FaultMode', fault_mode);
    assignin('base', 'BMS_StopTime',  numel(I) - 1);
    bms_setup('synthetic', false);          % fills Q/R from the ACO-tuned data if not yet defined

    fprintf('Simulating %s | profile=%s fault=%d init_err=%+.0f%% seed=%d\n', model, profile, fault_mode, 100 * init_err, seed);
    out = sim(model);
    plot_simulink_results(out, 'synthetic');
end
