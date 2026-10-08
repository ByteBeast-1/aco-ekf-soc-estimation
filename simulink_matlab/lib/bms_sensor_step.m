function [Vm, Im, Tm, ss] = bms_sensor_step(Vt, I, T, k, ss, P, fault_mode)
%BMS_SENSOR_STEP Sensor models: Gaussian noise, 1 mV ADC, optional fault injection.
%   fault_mode: 0 none | 1 current offset | 2 voltage glitches | 3 thermistor open
    [zv, ss] = bms_randn(ss);
    [zi, ss] = bms_randn(ss);
    [zt, ss] = bms_randn(ss);
    Vm = Vt + P.sv * zv;
    Im = I  + P.si * zi;
    Tm = T  + P.sT * zt;
    if k >= P.fault_k0 && k <= P.fault_k1
        if fault_mode == 1
            Im = Im + P.fault_I_offset;
        elseif fault_mode == 2
            if mod(k, 25) == 0
                Vm = Vm + P.fault_V_spike * (1.0 - 2.0 * mod(k / 25, 2));
            end
        elseif fault_mode == 3
            Tm = P.fault_T_stuck;
        end
    end
    Vm = round(Vm / P.adc_lsb) * P.adc_lsb;
end
