function m = bms_metrics(est, ref, skip)
%BMS_METRICS SOC error metrics in percent. SKIP samples are ignored (filter convergence window).
    if nargin < 3,  skip = 0;  end
    e = est(:) - ref(:);   e = e(skip+1:end);
    m.mae  = mean(abs(e));
    m.rmse = sqrt(mean(e.^2));
    m.max  = max(abs(e));
    m.p95  = prctile(abs(e), 95);
end
