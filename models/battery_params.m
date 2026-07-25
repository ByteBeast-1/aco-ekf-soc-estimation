function [OCV, Ri, Rp, Rc, Cp, Cc] = battery_params(SoC)
% BATTERY_PARAMS  SoC-dependent R-2RC equivalent-circuit parameters.
%
%   [OCV, Ri, Rp, Rc, Cp, Cc] = battery_params(SoC)
%
%   Implements equations (5)-(10) from the base paper:
%   "Ant Colony Optimized Extended Kalman Filter for State of Charge
%   Estimation of Lithium-Ion Batteries" (IEEE TIM, 2025).
%
%   INPUT
%       SoC  - State of Charge, scalar or vector, expected in [0, 1]
%
%   OUTPUT
%       OCV  - Open circuit voltage (V)
%       Ri   - Internal resistance (Ohm)
%       Rp   - Fast-transient branch resistance (Ohm)
%       Rc   - Slow-transient branch resistance (Ohm)
%       Cp   - Fast-transient branch capacitance (F)
%       Cc   - Slow-transient branch capacitance (F)
%
%   NOTE: these polynomial coefficients are taken directly from the base
%   paper. Values outside SoC = [0, 1] are not physically meaningful, so
%   SoC is clamped into range before evaluating the polynomials.

    SoC = min(max(SoC, 0), 1);   % clamp to valid range

    OCV = 0.5214*SoC.^3 - 0.9611*SoC.^2 + 0.66*SoC + 3.13;         % Eq. (5)

    Ri  = -0.03*SoC.^3 + 0.05*SoC.^2 - 0.03*SoC + 0.031;           % Eq. (6)

    Rp  = 1.14*SoC.^4 - 2.91*SoC.^3 + 2.68*SoC.^2 - 1.06*SoC + 0.18; % Eq. (7)

    Rc  = -0.04*SoC.^3 + 0.08*SoC.^2 - 0.06*SoC + 0.02;            % Eq. (8)

    Cp  = 2.32e4*SoC.^5 - 6.03e4*SoC.^4 + 5.80e4*SoC.^3 ...
          - 2.53e4*SoC.^2 + 5.17e3*SoC + 439.2;                    % Eq. (9)

    Cc  = -3.62e3*SoC.^5 + 9.08e3*SoC.^4 - 9.02e3*SoC.^3 ...
          + 4.82e3*SoC.^2 - 1.09e3*SoC + 283.8;                    % Eq. (10)
end
