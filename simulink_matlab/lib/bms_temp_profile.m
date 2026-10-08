function T = bms_temp_profile(k)
%BMS_TEMP_PROFILE Cell self-heating used by the simulated plant: 24 C -> 39 C over 1 h.
    T = 297.15 + 15.0 * min(k / 3600.0, 1.0);
end
