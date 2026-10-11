function S = bms_cast(S, cls)
%BMS_CAST Recursively convert every floating-point field of a struct to class CLS ('single' or 'double').
%   Used to run the estimator in 32-bit float, the arithmetic of the embedded firmware.
    f = fieldnames(S);
    for i = 1:numel(f)
        v = S.(f{i});
        if isstruct(v)
            S.(f{i}) = bms_cast(v, cls);
        elseif isnumeric(v) && ~isinteger(v)
            S.(f{i}) = cast(v, cls);
        end
    end
end
