function [x, Vt] = bms_plant_step(x, I, T, P)
%BMS_PLANT_STEP One step of the "true" 2-RC cell (with parameter mismatch). x = [SOC; V1; V2].
    [R0, ~, ~, a1, b1, a2, b2] = bms_rc(T, P, P.plant_R_scale);
    Qn  = P.Qn * P.plant_Qn_scale;
    soc = min(max(x(1) - P.eta * I * P.dt / Qn, 0.0), 1.0);
    v1  = a1 * x(2) + b1 * I;
    v2  = a2 * x(3) + b2 * I;
    x   = [soc; v1; v2];
    Vt  = bms_ocv(soc, P) - v1 - v2 - R0 * I;
end
