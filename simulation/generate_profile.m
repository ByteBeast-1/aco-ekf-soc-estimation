function [t, I] = generate_profile(profile_name, dt, duration)
% GENERATE_PROFILE  Synthetic discharge current profiles (Fig. 7, base paper).
%
%   [t, I] = generate_profile(profile_name, dt, duration)
%
%   INPUT
%       profile_name - one of:
%           'constant'   : constant 7.2 A discharge
%           'pulse'      : 6.5 A ON (25 s) / 0 A OFF (5 s), repeating
%           'dual_pulse' : 6.25 A ON (25 s) / 1.7 A OFF (5 s), repeating
%           'motor'      : 5 A ON (25 s) / 0 A (open-circuit) OFF (5 s)
%       dt       - sample time in seconds (paper uses 1 s)
%       duration - total simulation time in seconds
%
%   OUTPUT
%       t - time vector (s), column vector, 0:dt:duration
%       I - discharge current (A), same size as t.
%           Convention: positive I = discharging.

    t = (0:dt:duration)';
    n = length(t);
    I = zeros(n, 1);

    switch profile_name
        case 'constant'
            I(:) = 7.2;

        case 'pulse'
            I = make_cycle(t, 25, 5, 6.5, 0);

        case 'dual_pulse'
            I = make_cycle(t, 25, 5, 6.25, 1.7);

        case 'motor'
            I = make_cycle(t, 25, 5, 5.0, 0);

        otherwise
            error('generate_profile:unknownProfile', ...
                  'Unknown profile "%s". Use constant, pulse, dual_pulse, or motor.', ...
                  profile_name);
    end
end

function I = make_cycle(t, on_time, off_time, on_amp, off_amp)
% Repeating ON/OFF rectangular current cycle.
    cycle_len = on_time + off_time;
    phase = mod(t, cycle_len);
    I = zeros(size(t));
    I(phase < on_time) = on_amp;
    I(phase >= on_time) = off_amp;
end
