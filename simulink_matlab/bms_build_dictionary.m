function bms_build_dictionary()
%BMS_BUILD_DICTIONARY (Re)create bms_data.sldd: the Simulink data dictionary shared by both BMS models.
%   Contents
%     Bus objects (every element has a name, unit, data type and description)
%       BMS_PlantTruth   what the simulated cell really did
%       BMS_Measurement  what the sensors report (also built from real NASA data in the replay model)
%       BMS_Estimate     everything the estimator outputs
%     Parameters (Simulink.Parameter, values taken from lib/bms_params.m and lib/bms_tuned_data.m)
%       BMS_Q, BMS_R     ACO-tuned noise settings, simulated cell
%       NASA_Q, NASA_R   ACO-tuned noise settings, NASA-calibrated cell
%   Run automatically by BUILD_BMS_MODELS. Re-run it (or build_bms_models) after re-tuning with BMS_ACO_TUNE.
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, 'lib'));
    f = fullfile(here, 'bms_data.sldd');
    Simulink.data.dictionary.closeAll;
    if isfile(f),  delete(f);  end
    dd  = Simulink.data.dictionary.create(f);
    sec = getSection(dd, 'Design Data');

    addEntry(sec, 'BMS_PlantTruth', make_bus({ ...
        'SOC_pct',       'percent', 'True state of charge'; ...
        'Voltage_V',     'V',       'True terminal voltage'; ...
        'Temperature_K', 'K',       'True cell temperature'}));

    addEntry(sec, 'BMS_Measurement', make_bus({ ...
        'Voltage_V',     'V',       'Measured terminal voltage'; ...
        'Current_A',     'A',       'Measured current, positive when discharging'; ...
        'Temperature_K', 'K',       'Measured cell temperature'}));

    addEntry(sec, 'BMS_Estimate', make_bus({ ...
        'SOC_pct',          'percent', 'Estimated state of charge'; ...
        'Voltage_V',        'V',       'Predicted terminal voltage (before the correction)'; ...
        'Innovation_V',     'V',       'Measured minus predicted voltage'; ...
        'PTrace',           '',        'Trace of the state covariance matrix'; ...
        'NIS',              '',        'Normalised innovation squared, expected near 1'; ...
        'FaultCode',        '',        'Diagnostic bit mask: 1 V range, 2 T range, 4 I range, 8 rejected, 16 inconsistent'; ...
        'R0_Ohm',           'ohm',     'Ohmic resistance used by the model'; ...
        'SOC_Coulomb_pct',  'percent', 'Coulomb-counting baseline SOC'; ...
        'R_cov_V2',         '',        'Measurement-noise variance used by the filter'; ...
        'CapacityRatio',    '',        'Estimated capacity divided by nominal capacity'}));

    addEntry(sec, 'BMS_Reference', make_bus({ ...
        'SOC_pct',       'percent', 'Reference state of charge'; ...
        'Source_Tag',    '',        'Source tag (0: SIM, 1: REPLAY)'}));

    Ps = bms_params('synthetic');  Pn = bms_params('nasa');
    addEntry(sec, 'BMS_Q',  make_param(Ps.Q(:), 'Process-noise variances [SOC; V1; V2], ACO-tuned, simulated cell'));
    addEntry(sec, 'BMS_R',  make_param(Ps.R,    'Voltage measurement-noise variance, ACO-tuned, simulated cell'));
    addEntry(sec, 'NASA_Q', make_param(Pn.Q(:), 'Process-noise variances [SOC; V1; V2], ACO-tuned, NASA-calibrated cell'));
    addEntry(sec, 'NASA_R', make_param(Pn.R,    'Voltage measurement-noise variance, ACO-tuned, NASA-calibrated cell'));

    saveChanges(dd);
    close(dd);
end

function b = make_bus(rows)
    n = size(rows, 1);
    el = Simulink.BusElement.empty(0, n);
    for i = 1:n
        e = Simulink.BusElement;
        e.Name = rows{i, 1};  e.DataType = 'double';  e.Dimensions = 1;
        e.Unit = rows{i, 2};  e.Description = rows{i, 3};
        el(i) = e;
    end
    b = Simulink.Bus;
    b.Elements = el;
end

function p = make_param(value, desc)
    p = Simulink.Parameter(value);
    p.DataType = 'double';
    p.Description = desc;
end
