function [st, out] = bms_est_step(st, I, V, T, P, mode)
%BMS_EST_STEP 4-state Joseph-form EKF (SOC, V1, V2, optional online capacity) with diagnostics.
%   mode is a bit mask (0 = conventional EKF with fixed, ACO-tuned Q and R):
%       1 = scalar-RLS tracking of the R0 scale   2 = Sage-Husa adaptation of R
%       4 = Sage-Husa adaptation of Q             8 = online capacity (SOH) state
%
%   Diagnostic code (sum of): 1 voltage out of range | 2 temperature out of range |
%   4 current out of range | 8 innovation rejected by gate | 16 filter inconsistent (NIS) |
%   32 current sensor offset fault (persistent innovation bias)
    st.k = st.k + 1.0;
    flags = 0.0;

    % ---- 1. sensor plausibility ---------------------------------------
    vbad = ~(V > P.V_valid(1) && V < P.V_valid(2));
    if vbad
        flags = flags + 1.0;
    end
    if ~(T > P.T_valid(1) && T < P.T_valid(2))
        flags = flags + 2.0;
        T = st.T_last;                       % hold last good temperature
    else
        st.T_last = T;
    end
    if abs(I) > P.I_max
        flags = flags + 4.0;
    end

    % ---- 2. time update -------------------------------------------------
    cap_mode = bitand(mode, 8) > 0;
    if cap_mode && st.cap_active < 0.5
        st.x(4) = 1.0;
        st.Pc(4, :) = 0.0;
        st.Pc(:, 4) = 0.0;
    end

    [R0n, ~, ~, a1, b1, a2, b2] = bms_rc(T, P);
    R0  = R0n * st.rls_th;
    theta = st.x(4);                                  % capacity ratio Q_true / Q_nominal
    soc_p = min(max(st.x(1) - P.eta * I * P.dt / (P.Qn * theta), 0.0), 1.0);
    xp  = [soc_p; a1 * st.x(2) + b1 * I; a2 * st.x(3) + b2 * I; theta];
    F   = [1.0, 0.0, 0.0, P.eta * I * P.dt / (P.Qn * theta * theta); ...
           0.0, a1,  0.0, 0.0; ...
           0.0, 0.0, a2,  0.0; ...
           0.0, 0.0, 0.0, 1.0];
    Pp  = F * st.Pc * F' + st.Q;
    if cap_mode && st.cap_active < 0.5
        Pp(4, :) = 0.0;
        Pp(:, 4) = 0.0;
    end

    % ---- 3. innovation ---------------------------------------------------
    [vocv, dv] = bms_ocv(soc_p, P);
    Vest = vocv - xp(2) - xp(3) - R0 * I;
    r    = V - Vest;
    H    = [dv, -1.0, -1.0, 0.0];
    S    = H * Pp * H' + st.R;
    nis  = r * r / S;

    % ---- 4. outlier gate ---------------------------------------------------
    accept = ~vbad;
    if accept && st.k > P.gate_k0 && nis > P.gate_nis && st.rej < P.max_rej
        accept = false;
        flags  = flags + 8.0;
        st.rej = st.rej + 1.0;
    elseif accept
        st.rej = 0.0;
    end

    % ---- 5. measurement update (Joseph form) --------------------------------
    if accept
        K  = (Pp * H') / S;
        xu = xp + K * r;
        xu(1) = min(max(xu(1), 0.0), 1.0);
        IKH = eye(4) - K * H;
        Pu  = IKH * Pp * IKH' + K * st.R * K';

        % Evidence gating, lock-out, rate limit, and bounds on capacity
        if cap_mode
            if st.cap_active < 0.5
                xu(4) = 1.0;
                Pu(4, :) = 0.0;
                Pu(:, 4) = 0.0;
                if st.k > P.cap_k_lock && V > P.cap_V_min
                    st.cap_r_smooth = 0.95 * st.cap_r_smooth + 0.05 * r;
                    if st.cap_r_smooth < P.cap_r_thresh
                        st.cap_ev_cnt = st.cap_ev_cnt + 1.0;
                        if st.cap_ev_cnt >= P.cap_count_limit
                            st.cap_active = 1.0;
                            Pu(4, 4) = P.cap_P0;
                        end
                    else
                        st.cap_ev_cnt = max(0.0, st.cap_ev_cnt - 0.5);
                    end
                end
            else
                d_th = xu(4) - st.x(4);
                if abs(d_th) > P.cap_slew_max
                    xu(4) = st.x(4) + sign(d_th) * P.cap_slew_max;
                end
                xu(4) = min(max(xu(4), P.cap_bounds(1)), P.cap_bounds(2));
            end
        end

        d = P.sh_d;
        if bitand(mode, 2) > 0                       % Sage-Husa on R
            if ~cap_mode || st.cap_active >= 0.5
                Rn = (1.0 - d) * st.R + d * (r * r - H * Pp * H');
                st.R = min(max(Rn, P.R_bounds(1)), P.R_bounds(2));
            else
                st.R = P.R;
            end
        end
        if bitand(mode, 4) > 0                       % Sage-Husa on Q
            Qn_ = (1.0 - d) * st.Q + d * ((K * r) * (K * r)' + Pu - F * st.Pc * F');
            dq  = diag(Qn_);
            st.Q = diag([min(max(dq(1:3), P.Q_bounds(1)), P.Q_bounds(2)); st.Q(4, 4)]);
        end
        if bitand(mode, 1) > 0
            % scalar RLS on R0: (OCV - V1 - V2 - V) = R0 * I, gated by excitation
            if abs(I) > P.rls_I_min
                z   = vocv - xp(2) - xp(3) - V;
                phi = R0n * I;
                g   = st.rls_P * phi / (P.rls_lambda + phi * st.rls_P * phi);
                st.rls_th = st.rls_th + g * (z - phi * st.rls_th);
                st.rls_th = min(max(st.rls_th, P.rls_bounds(1)), P.rls_bounds(2));
                st.rls_P  = min((st.rls_P - g * phi * st.rls_P) / P.rls_lambda, 100.0);
            end
        end
    else
        xu = xp;
        Pu = Pp;
        if cap_mode && st.cap_active < 0.5
            xu(4) = 1.0;
            Pu(4, :) = 0.0;
            Pu(:, 4) = 0.0;
        end
    end

    % ---- 6. consistency monitor --------------------------------------------
    st.nis_avg = (1.0 - P.nis_alpha) * st.nis_avg + P.nis_alpha * min(nis, 100.0);
    if st.k > 100 && st.nis_avg > P.nis_limit
        flags = flags + 16.0;
    end

    % ---- 7. current-bias monitor (bit 32) ----------------------------------
    if st.k > P.ibias_k0 && xu(1) >= P.ibias_soc_min && xu(1) <= 0.95
        if I >= 0.0
            st.innov_avg = (1.0 - P.ibias_alpha) * st.innov_avg + P.ibias_alpha * r;
            if abs(st.innov_avg) > P.ibias_thresh
                st.ibias_cnt = st.ibias_cnt + 1.0;
            else
                st.ibias_cnt = max(0.0, st.ibias_cnt - 1.0);
            end
        else
            st.ibias_cnt = max(0.0, st.ibias_cnt - 1.0);
        end
        if st.ibias_cnt >= P.ibias_count_limit
            flags = flags + 32.0;
        end
    end

    st.x      = xu;
    st.Pc     = Pu;
    st.soc_cc = min(max(st.soc_cc - P.eta * I * P.dt / P.Qn, 0.0), 1.0);

    out.soc    = 100.0 * xu(1);
    out.vest   = Vest;
    out.innov  = r;
    out.ptrace = trace(Pu);
    out.nis    = nis;
    out.fault  = flags;
    out.r0     = R0;
    out.soc_cc = 100.0 * st.soc_cc;
    out.rcov   = st.R;
    out.cap    = xu(4);
end
