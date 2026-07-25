function out = simulate_battery(t, I, SoC0, Qn_Ah, add_noise)
% SIMULATE_BATTERY  Forward-simulate the true R-2RC battery plant.
%
%   out = simulate_battery(t, I, SoC0, Qn_Ah, add_noise)
%
%   This represents the REAL battery (the "ground truth" plant), not the
%   EKF. It propagates SoC, Vp, Vc, and terminal voltage VT forward in
%   time using the nonlinear battery model directly -- Coulomb counting
%   for SoC, and the exact R-2RC transient equations (3)-(4) and the
%   terminal voltage equation (2) from the base paper.
%
%   Week 2's EKF will later try to RE-ESTIMATE SoC from noisy I and VT
%   generated here, without being told the true SoC directly.
%
%   INPUT
%       t         - time vector (s), from generate_profile
%       I         - current vector (A), positive = discharging
%       SoC0      - initial SoC, in [0, 1] (e.g. 1.0 = fully charged)
%       Qn_Ah     - nominal capacity in Amp-hours (paper: 12 Ah)
%       add_noise - true/false, whether to add realistic sensor noise
%                   to the OUTPUT current/voltage (does not affect the
%                   true SoC trajectory itself)
%
%   OUTPUT (struct)
%       out.t         - time vector
%       out.SoC_true  - true SoC trajectory (ground truth, no noise)
%       out.Vp_true   - fast-transient branch voltage
%       out.Vc_true   - slow-transient branch voltage
%       out.VT_true   - true terminal voltage (no noise)
%       out.I_meas    - current as "measured" by a sensor (noisy if requested)
%       out.VT_meas   - terminal voltage as "measured" by a sensor (noisy if requested)

    if nargin < 5
        add_noise = false;
    end

    dt = t(2) - t(1);
    n  = length(t);
    Qn = Qn_Ah * 3600;   % Amp-hours -> Amp-seconds, matches Eq. (11)'s units

    SoC = zeros(n, 1);
    Vp  = zeros(n, 1);
    Vc  = zeros(n, 1);
    VT  = zeros(n, 1);

    SoC(1) = SoC0;
    Vp(1)  = 0;   % assume battery starts at rest, no transient voltage yet
    Vc(1)  = 0;

    for k = 1:n
        [OCV_k, Ri_k, Rp_k, Rc_k, Cp_k, Cc_k] = battery_params(SoC(k));

        % Terminal voltage, Eq. (2)
        VT(k) = OCV_k - Vp(k) - Vc(k) - I(k) * Ri_k;

        if k < n
            % Coulomb counting state update, Eq. (11)
            SoC(k+1) = SoC(k) - I(k) * dt / Qn;

            % RC branch updates, Eq. (3)-(4)
            aP = exp(-dt / (Rp_k * Cp_k));
            aC = exp(-dt / (Rc_k * Cc_k));
            Vp(k+1) = aP * Vp(k) + Rp_k * (1 - aP) * I(k);
            Vc(k+1) = aC * Vc(k) + Rc_k * (1 - aC) * I(k);
        end
    end

    I_meas  = I;
    VT_meas = VT;
    if add_noise
        % Illustrative sensor noise levels; refine once real hardware is used.
        I_meas  = I  + 0.02  * randn(n, 1);   % ~20 mA current-sensor noise
        VT_meas = VT + 0.005 * randn(n, 1);   % ~5 mV voltage-sensor noise
    end

    out.t        = t;
    out.SoC_true = SoC;
    out.Vp_true  = Vp;
    out.Vc_true  = Vc;
    out.VT_true  = VT;
    out.I_meas   = I_meas;
    out.VT_meas  = VT_meas;
end
