function bms_setup(variant, force)
%BMS_SETUP Put the library on the path and define the base-workspace variables the models read.
%   Called automatically by the models' InitFcn. Existing variables are kept unless FORCE is true,
%   so a Simulink.SimulationInput / Monte-Carlo script can override any of them.
%   The tuned noise settings (BMS_Q, BMS_R, NASA_Q, NASA_R) and the bus objects live in bms_data.sldd.
%
%   'synthetic' (advanced_bms_aco_ekf)           'nasa' (sota_bms_aco_ekf)
%     Drive_Profile   [t I] drive cycle            NASA_Replay     [t I V T SOC_ref] real cycle
%     BMS_SOC0        true initial SOC             NASA_SOC_Guess  filter initial SOC
%     BMS_SOC_Guess   filter initial SOC           NASA_StopTime   [s]
%     BMS_Seed        sensor-noise seed            NASA_CapRatio   true capacity / 2.0 Ah (for the SOH scope)
%     BMS_FaultMode   0 none|1 I-offset|2 V-glitch|3 T-open
%     BMS_StopTime    [s]
    if nargin < 1,  variant = 'synthetic';  end
    if nargin < 2,  force   = false;        end
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, 'lib'));

    function put(name, value)
        if force || ~evalin('base', sprintf('exist(''%s'',''var'')', name))
            assignin('base', name, value);
        end
    end

    if strcmp(variant, 'nasa')
        sc = bms_replay_scenario('B0005', 1, -0.15);
        put('NASA_Replay',    [sc.t(:), sc.I(:), sc.V(:), sc.T(:), sc.soc_ref(:)]);
        put('NASA_SOC_Guess', sc.soc_ref(1) + sc.init_err);
        put('NASA_StopTime',  sc.t(end));
        put('NASA_CapRatio',  sc.cap_Ah / 2.0);
    else
        [I, soc0] = bms_profile('udds');
        put('Drive_Profile', [(0:numel(I) - 1)', I(:)]);
        put('BMS_SOC0',      soc0);
        put('BMS_SOC_Guess', soc0 - 0.10);
        put('BMS_Seed',      1);
        put('BMS_FaultMode', 0);
        put('BMS_StopTime',  numel(I) - 1);
    end
end
