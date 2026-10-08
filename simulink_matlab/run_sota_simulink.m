function out = run_sota_simulink(cellname, cycle, init_err)
%RUN_SOTA_SIMULINK Replay a REAL NASA Ames discharge through sota_bms_aco_ekf and plot the result.
%   out = run_sota_simulink()                      B0005, discharge #1, filter starts 15 % low
%   out = run_sota_simulink('B0007', 60, -0.20)    aged cell, 20 % low start
%
%   The OCV curve and ACO covariances were calibrated on B0006 (discharges #1 and #60), so B0005 and
%   B0007 are out-of-sample. Data come from data_raw/B000x.mat (not the synthetic data/NASA_B0005.csv).
    if nargin < 1,  cellname = 'B0005';  end
    if nargin < 2,  cycle    = 1;        end
    if nargin < 3,  init_err = -0.15;    end
    here = fileparts(mfilename('fullpath'));
    addpath(here);  addpath(fullfile(here, 'lib'));
    model = 'sota_bms_aco_ekf';
    if ~isfile(fullfile(here, [model '.slx'])),  build_bms_models('nasa');  end

    sc = bms_replay_scenario(cellname, cycle, init_err);
    assignin('base', 'NASA_Replay',    [sc.t(:), sc.I(:), sc.V(:), sc.T(:), sc.soc_ref(:)]);
    assignin('base', 'NASA_SOC_Guess', min(max(sc.soc_ref(1) + init_err, 0), 1));
    assignin('base', 'NASA_StopTime',  sc.t(end));
    assignin('base', 'NASA_CapRatio',  sc.cap_Ah / 2.0);
    bms_setup('nasa', false);

    fprintf('Replaying %s through %s (%d s)\n', sc.name, model, sc.t(end));
    out = sim(model);
    plot_simulink_results(out, 'nasa', sc.name);
end
