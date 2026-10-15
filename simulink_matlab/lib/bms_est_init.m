function st = bms_est_init(P, soc_guess, Qd, R, mode)
%BMS_EST_INIT Initial estimator state. Qd is a 3x1 process-noise diagonal, R the voltage-noise variance.
%   State x = [SOC; V1; V2; capacity ratio]. The capacity state is frozen (variance 0) unless bit 8 of
%   MODE is set, in which case the filter also learns Q_true / Q_nominal (SOH) online.
    if nargin < 5
        mode = 0;
    end
    p44 = 0.0;  q44 = 0.0;
    if bitand(mode, 8) > 0
        p44 = 0.0;  q44 = P.cap_Q;
    end
    st.x       = [soc_guess; 0.0; 0.0; 1.0];
    st.Pc      = [P.P0, zeros(3, 1); zeros(1, 3), p44];
    st.Q       = diag([Qd(:); q44]);
    st.R       = R;
    st.soc_cc  = soc_guess;     % Coulomb-counting baseline
    st.rls_th  = 1.0;           % R0 scale factor (SOTA only)
    st.rls_P   = 10.0;
    st.rej     = 0.0;
    st.k       = 0.0;
    st.T_last    = 298.15;
    st.nis_avg   = 1.0;
    st.innov_avg = 0.0;
    st.ibias_cnt = 0.0;
    st.cap_active   = 0.0;
    st.cap_r_smooth = 0.0;
    st.cap_ev_cnt   = 0.0;
end
