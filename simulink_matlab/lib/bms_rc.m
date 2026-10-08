function [R0, R1, R2, a1, b1, a2, b2] = bms_rc(T, P, rscale)
%BMS_RC Arrhenius-scaled resistances and discretised RC coefficients at temperature T [K].
    if nargin < 3
        rscale = 1.0;
    end
    T  = min(max(T, 250.0), 350.0);
    tf = exp((P.Ea / P.Rg) * (1.0 / T - 1.0 / P.Tref)) * rscale;
    R0 = P.R0 * tf;  R1 = P.R1 * tf;  R2 = P.R2 * tf;
    a1 = exp(-P.dt / (R1 * P.C1));  b1 = R1 * (1.0 - a1);
    a2 = exp(-P.dt / (R2 * P.C2));  b2 = R2 * (1.0 - a2);
end
