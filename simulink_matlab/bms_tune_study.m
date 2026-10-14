function study = bms_tune_study(varargin)
%BMS_TUNE_STUDY Wrapper calling lib/bms_tune_study.m
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, 'lib'));
    study = bms_tune_study(varargin{:});
end
