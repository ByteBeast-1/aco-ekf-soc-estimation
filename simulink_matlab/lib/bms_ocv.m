function [v, dv] = bms_ocv(soc, P)
%BMS_OCV Open-circuit voltage and its slope dV/dSOC (slope floored for a safe Jacobian).
    s  = min(max(soc, 0.0), 1.0);
    v  = polyval(P.ocv, s);
    dv = max(polyval(polyder(P.ocv), s), 0.02);
end
