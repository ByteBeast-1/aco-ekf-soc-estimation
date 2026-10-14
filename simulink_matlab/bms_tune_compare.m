function tbl = bms_tune_compare(varargin)
%BMS_TUNE_COMPARE Wrapper calling lib/bms_tune_compare.m
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, 'lib'));
    tbl = bms_tune_compare(varargin{:});
end
