function [z, s] = bms_randn(s)
%BMS_RANDN Reproducible N(0,1) generator (xorshift32 + Box-Muller). State s is uint32.
    [u1, s] = bms_rand_u(s);
    [u2, s] = bms_rand_u(s);
    z = sqrt(-2.0 * log(u1)) * cos(2.0 * pi * u2);
end

function [u, s] = bms_rand_u(s)
    s = bitxor(s, bitshift(s, 13));
    s = bitxor(s, bitshift(s, -17));
    s = bitxor(s, bitshift(s, 5));
    u = (double(s) + 0.5) / 4294967296.0;
end
