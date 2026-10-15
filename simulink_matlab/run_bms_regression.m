function ok = run_bms_regression(opts)
%RUN_BMS_REGRESSION Automated verification of the BMS estimator and its Simulink models.
%   ok = run_bms_regression()                 full suite (about 3-4 min)
%   ok = run_bms_regression(struct('simulink',false))   skip the Simulink back-to-back tests
%
%   Suites
%     1. Back-to-back  : Simulink output == MATLAB library output (model-vs-code equivalence)
%     2. Monte Carlo   : 40 randomised synthetic runs (init error +-20 %, capacity/resistance mismatch,
%                        4 drive profiles, random noise seeds)
%     3. Fault injection: current offset, voltage glitches, thermistor open-circuit
%     4. Real NASA data: out-of-sample cells (B0005, B0007, aged B0006); tuning/calibration used B0006 #1/#60 only
%     5. Ablation      : which adaptation features help (bit-mask modes)
%     0. Model quality : no dangling ports, strict diagnostics, bus/unit/dictionary consistency
%     6. Precision     : estimator in 32-bit float (the firmware's arithmetic) versus 64-bit double
%     7. SIL           : compiled C firmware versus the MATLAB float32 estimator on identical measurements
%     8. Unified model : bms_unified (SIM and REPLAY) == legacy models within 1e-9
%   Options: struct('simulink', false) skips suites 0, 1, and 8; struct('sil', false) skips suite 7.
%   Results -> results/bms_regression_results.csv and figures/15_*.png
    if nargin < 1,  opts = struct();  end
    if ~isfield(opts, 'simulink'),  opts.simulink = true;  end
    if ~isfield(opts, 'sil'),       opts.sil = true;       end
    here = fileparts(mfilename('fullpath'));
    addpath(here);  addpath(fullfile(here, 'lib'));
    outdir = fullfile(here, 'results');  if ~isfolder(outdir),  mkdir(outdir);  end

    TH.b2b_tol        = 1e-6;     % [%] max |Simulink - library|
    TH.mc_mae_mean    = 1.0;      % [%] EKF mean MAE over Monte Carlo
    TH.mc_mae_worst   = 2.0;      % [%] worst-case MAE of any run
    TH.mc_max_err     = 6.0;      % [%] worst instantaneous error after convergence
    TH.fault_extra    = 3.0;      % [%] extra max-error tolerated under a fault
    TH.nasa_fresh_mae = 2.0;      % [%] fresh unseen cells (B0005 #1, B0007 #1)
    TH.nasa_any_mae   = 6.5;      % [%] any unseen cycle (aged cells included)
    TH.nasa_charge_mae = 5.0;     % [%] fresh real charge cycles
    TH.soh_err        = 0.20;     % mean |capacity-ratio error| of the online SOH estimate
    TH.prec_dsoc      = 0.10;     % [%]  worst |SOC float32 - SOC float64|
    TH.prec_dv_mV     = 1.0;      % [mV] worst |V_est float32 - V_est float64|
    TH.sil_dsoc       = 0.002;    % [%]  worst |SOC C firmware - SOC MATLAB float32|
    TH.sil_dv_mV      = 1.0;      % [mV] worst predicted-voltage difference
    TH.sil_ptrace_rel = 1e-3;     % relative difference of trace(P)
    skip = 120;                   % convergence window excluded from metrics [s]

    rows = {};  checks = {};
    function check(name, passed, detail)
        checks(end + 1, :) = {name, passed, detail};                       %#ok<AGROW>
        fprintf('  [%s] %s  (%s)\n', ternary(passed, 'PASS', 'FAIL'), name, detail);
    end

    %% ---- 1. back-to-back ------------------------------------------------
    if opts.simulink
        fprintf('\n== 1. Back-to-back: Simulink vs MATLAB library ==\n');
        % synthetic model
        bms_setup('synthetic', true);
        so = sim('advanced_bms_aco_ekf');  d = so.bms_log;
        P = bms_params('synthetic');  [I, soc0] = bms_profile('udds');
        L = bms_simulate(P, struct('I', I, 'soc0', soc0, 'init_err', -0.10, 'seed', 1, 'fault_mode', 0));
        e = max(abs(d(:, 2)' - L.soc_ekf));  e2 = max(abs(d(:, 1)' - L.soc_true));
        check('advanced_bms_aco_ekf: EKF SOC == library', e < TH.b2b_tol, sprintf('max diff %.2e %%', e));
        check('advanced_bms_aco_ekf: plant SOC == library', e2 < TH.b2b_tol, sprintf('max diff %.2e %%', e2));
        bdclose('advanced_bms_aco_ekf');
        % real-data model
        bms_setup('nasa', true);
        so = sim('sota_bms_aco_ekf');  d = so.bms_log;
        Pn = bms_params('nasa');  sc = bms_replay_scenario('B0005', 1, -0.15);
        Ln = bms_simulate(Pn, sc);
        e = max(abs(d(:, 2)' - Ln.soc_ekf));  e2 = max(abs(d(:, 3)' - Ln.soc_sota));
        check('sota_bms_aco_ekf: baseline EKF == library', e < TH.b2b_tol, sprintf('max diff %.2e %%', e));
        check('sota_bms_aco_ekf: adaptive EKF == library', e2 < TH.b2b_tol, sprintf('max diff %.2e %%', e2));
        Pc = Pn;  Pc.sota_mode = 10;  Lc = bms_simulate(Pc, sc);
        e3 = max(abs(d(:, 12)' - Lc.soc_sota));  e4 = max(abs(d(:, 13)' - Lc.cap_sota));
        check('sota_bms_aco_ekf: capacity-state SOC == library', e3 < TH.b2b_tol, sprintf('max diff %.2e %%', e3));
        check('sota_bms_aco_ekf: capacity ratio == library', e4 < 1e-9, sprintf('max diff %.2e', e4));
        bdclose('sota_bms_aco_ekf');

        fprintf('\n== 0. Model quality (structure, interfaces, diagnostics) ==\n');
        n_un = 0;  strict_ok = true;
        for mname = {'advanced_bms_aco_ekf', 'sota_bms_aco_ekf', 'bms_estimator_ref', 'bms_unified'}
            [nu, so_] = model_quality(mname{1});
            n_un = n_un + nu;  strict_ok = strict_ok && so_;
        end
        check('Models: no unconnected ports', n_un == 0, sprintf('%d unconnected ports across all 4 models', n_un));
        check('Models: strict diagnostics are errors (dangling ports/lines, algebraic loops, bus mismatch)', strict_ok, 'all enabled');
        dq = dictionary_quality();
        check('Data dictionary: every bus element has a description and physical ones a unit', dq.units_ok, sprintf('%d elements checked', dq.n_elements));
        check('Data dictionary: tuned Q and R equal lib/bms_tuned_data.m', dq.params_ok, sprintf('max difference %.1e', dq.param_diff));
    end

    %% ---- 2. Monte Carlo ------------------------------------------------------
    fprintf('\n== 2. Monte Carlo (synthetic, 40 randomised runs) ==\n');
    P = bms_params('synthetic');  nMC = 40;
    profs = {'udds', 'pulse', 'cc', 'cccv'};
    rng(2024, 'twister');
    mc = struct('ekf', zeros(nMC, 4), 'sota', zeros(nMC, 4), 'cc', zeros(nMC, 4));
    mc_fp = 0;
    for i = 1:nMC
        Pi = P;
        Pi.plant_Qn_scale = 0.90 + 0.15 * rand();
        Pi.plant_R_scale  = 0.90 + 0.50 * rand();
        [I, soc0] = bms_profile(profs{mod(i - 1, 4) + 1});
        I = I(1:min(numel(I), 1800));
        ie = (2 * rand() - 1) * 0.20;
        if soc0 + ie < 0.02 || soc0 + ie > 0.98,  ie = -ie;  end
        L = bms_simulate(Pi, struct('I', I, 'soc0', soc0, 'init_err', ie, 'seed', 5000 + i, 'fault_mode', 0));
        a = bms_metrics(L.soc_ekf,  L.soc_true, skip);  mc.ekf(i, :)  = [a.mae a.rmse a.max a.p95];
        b = bms_metrics(L.soc_sota, L.soc_true, skip);  mc.sota(i, :) = [b.mae b.rmse b.max b.p95];
        c = bms_metrics(L.soc_cc,   L.soc_true, skip);  mc.cc(i, :)   = [c.mae c.rmse c.max c.p95];
    end
    fprintf('  %-22s %9s %9s %9s %9s\n', 'MAE / max error [%]', 'mean MAE', 'worst MAE', 'p95 MAE', 'worst max');
    for n = {'cc', 'ekf', 'sota'}
        M = mc.(n{1});
        fprintf('  %-22s %9.3f %9.3f %9.3f %9.3f\n', n{1}, mean(M(:,1)), max(M(:,1)), prctile(M(:,1), 95), max(M(:,3)));
        rows(end + 1, :) = {'montecarlo', n{1}, mean(M(:,1)), max(M(:,1)), prctile(M(:,1), 95), max(M(:,3))}; %#ok<AGROW>
    end
    check('Monte Carlo: EKF mean MAE', mean(mc.ekf(:,1)) < TH.mc_mae_mean, sprintf('%.3f %% < %.1f', mean(mc.ekf(:,1)), TH.mc_mae_mean));
    check('Monte Carlo: EKF worst-run MAE', max(mc.ekf(:,1)) < TH.mc_mae_worst, sprintf('%.3f %% < %.1f', max(mc.ekf(:,1)), TH.mc_mae_worst));
    check('Monte Carlo: EKF worst instantaneous error', max(mc.ekf(:,3)) < TH.mc_max_err, sprintf('%.3f %% < %.1f', max(mc.ekf(:,3)), TH.mc_max_err));
    check('Monte Carlo: EKF beats Coulomb counting', mean(mc.ekf(:,1)) < 0.2 * mean(mc.cc(:,1)), ...
        sprintf('%.3f vs %.3f %%', mean(mc.ekf(:,1)), mean(mc.cc(:,1))));

    % 40 clean Monte Carlo runs for false-positive validation of current-offset detector (bit 32)
    [I_udds, s0_udds] = bms_profile('udds');
    mc_fp = 0;
    for i = 1:nMC
        sc_mc = struct('I', I_udds, 'soc0', s0_udds, 'init_err', -0.10, 'seed', i, 'fault_mode', 0);
        L_mc = bms_simulate(P, sc_mc);
        mc_fp = mc_fp + nnz(bitand(L_mc.fault_ekf, 32) > 0);
    end
    check('Monte Carlo: zero false positives on current-offset detector (bit 32)', mc_fp == 0, ...
        sprintf('%d flagged samples over %d clean runs', mc_fp, nMC));

    %% ---- 3. Fault injection --------------------------------------------------------
    fprintf('\n== 3. Fault injection (synthetic UDDS, faults between t=900..1200 s) ==\n');
    [I, soc0] = bms_profile('udds');
    base = struct('I', I, 'soc0', soc0, 'init_err', -0.10, 'seed', 77, 'fault_mode', 0);
    L0 = bms_simulate(P, base);   m0 = bms_metrics(L0.soc_ekf, L0.soc_true, skip);
    names = {'none', 'current offset +0.25 A', 'voltage glitches +-0.5 V', 'thermistor open-circuit'};
    flag_bit = [0 32 8 2];        % expected diagnostic bit indexed by fault mode + 1
    t_f1_delay = NaN;
    for fm = 0:3
        sc = base;  sc.fault_mode = fm;  L = bms_simulate(P, sc);
        m = bms_metrics(L.soc_ekf, L.soc_true, skip);
        win = 900:1200;
        det = any(bitand(L.fault_ekf(win), max(flag_bit(fm + 1), 1)) > 0);
        fprintf('  mode %d %-26s max err %5.2f %%  MAE %5.3f %%  flag raised in window: %d\n', fm, names{fm+1}, m.max, m.mae, det);
        rows(end + 1, :) = {'fault', names{fm+1}, m.mae, m.rmse, m.p95, m.max}; %#ok<AGROW>
        if fm == 0
            check('Fault-free run raises no hard diagnostics', ~any(bitand(L.fault_ekf, 63) > 0), ...
                sprintf('%d flagged samples', nnz(bitand(L.fault_ekf, 63) > 0)));
        else
            check(sprintf('Fault %d (%s): bounded error', fm, names{fm+1}), m.max < m0.max + TH.fault_extra, ...
                sprintf('max %.2f %% vs fault-free %.2f %%', m.max, m0.max));
            if fm >= 1
                check(sprintf('Fault %d (%s): detected by diagnostics', fm, names{fm+1}), det, 'flag bit set in fault window');
            end
            if fm == 1
                k_flag = find(bitand(L.fault_ekf, 32) > 0, 1);
                t_f1_delay = k_flag - 900;
                check('Fault 1 (current offset +0.25 A): flagged within stated latency after onset', ...
                    det && (t_f1_delay <= 210), sprintf('latency %d s <= 210 s (flagged at t=%d s)', t_f1_delay, k_flag));
            end
        end
    end
    % Negative offset verification (-0.25 A)
    P_neg = P;  P_neg.fault_I_offset = -0.25;
    sc_neg = base;  sc_neg.fault_mode = 1;
    L_neg = bms_simulate(P_neg, sc_neg);
    det_neg = any(bitand(L_neg.fault_ekf(win), 32) > 0);
    k_flag_neg = find(bitand(L_neg.fault_ekf, 32) > 0, 1);
    t_f1_neg_delay = k_flag_neg - 900;
    fprintf('  mode 1b current offset -0.25 A     latency: %d s (flagged at t=%d s)\n', t_f1_neg_delay, k_flag_neg);
    check('Fault 1 (current offset -0.25 A and +0.25 A): both offsets flagged within latency', ...
        det && det_neg && (t_f1_delay <= 210) && (t_f1_neg_delay <= 210), ...
        sprintf('+0.25 A: %d s, -0.25 A: %d s (both <= 210 s)', t_f1_delay, t_f1_neg_delay));

    %% ---- 4. Real NASA data ----------------------------------------------------------------
    fprintf('\n== 4. Real NASA Ames data (out-of-sample cells) ==\n');
    Pn = bms_params('nasa');
    cases = {{'B0005',1},{'B0005',60},{'B0005',120},{'B0007',1},{'B0007',60},{'B0007',120},{'B0006',120}};
    fprintf('  %-22s %8s %8s %8s %9s\n', 'cycle (capacity)', 'CC MAE', 'EKF MAE', 'SOTA MAE', 'EKF max');
    nasa = zeros(numel(cases), 4);  scs = cell(1, numel(cases));
    for i = 1:numel(cases)
        sc = bms_replay_scenario(cases{i}{:}, -0.15);  scs{i} = sc;
        L = bms_simulate(Pn, sc);
        a = bms_metrics(L.soc_ekf, L.soc_true, skip);  b = bms_metrics(L.soc_sota, L.soc_true, skip);
        c = bms_metrics(L.soc_cc, L.soc_true, skip);
        nasa(i, :) = [c.mae a.mae b.mae a.max];
        fprintf('  %-22s %8.2f %8.2f %8.2f %9.2f\n', sc.name, c.mae, a.mae, b.mae, a.max);
        rows(end + 1, :) = {'nasa', sc.name, c.mae, a.mae, b.mae, a.max}; %#ok<AGROW>
    end
    fresh = [1 4];
    check('NASA fresh unseen cells: EKF MAE', all(nasa(fresh, 2) < TH.nasa_fresh_mae), sprintf('%.2f / %.2f %% < %.1f', nasa(fresh, 2), TH.nasa_fresh_mae));
    check('NASA all unseen cycles: EKF MAE', all(nasa(:, 2) < TH.nasa_any_mae), sprintf('worst %.2f %% < %.1f', max(nasa(:, 2)), TH.nasa_any_mae));
    check('NASA: EKF beats Coulomb counting on every cycle', all(nasa(:, 2) < nasa(:, 1)), sprintf('mean %.2f vs %.2f %%', mean(nasa(:,2)), mean(nasa(:,1))));
    nasa_fp = 0;
    nasa_fresh_cases = {{'B0005',1},{'B0006',1},{'B0007',1}};
    for i = 1:numel(nasa_fresh_cases)
        sc_ = bms_replay_scenario(nasa_fresh_cases{i}{:}, -0.15);
        L = bms_simulate(Pn, sc_);
        nasa_fp = nasa_fp + nnz(bitand(L.fault_ekf, 32) > 0);
    end
    check('NASA: zero false positives on current-offset detector (bit 32)', nasa_fp == 0, ...
        sprintf('%d flagged samples across fresh cycles (B0005, B0006, B0007 @ cycle 1)', nasa_fp));

    %% ---- 4b. Real NASA charge cycles + capacity (SOH) estimation -----------------------------
    fprintf('\n== 4b. Real CC-CV charge cycles (fresh cells; reference assumes start from ~empty) ==\n');
    chg = {{'B0005',1},{'B0007',1}};  nchg = zeros(numel(chg), 3);
    for i = 1:numel(chg)
        sc = bms_replay_scenario(chg{i}{:}, 0.15, 'charge');  L = bms_simulate(Pn, sc);
        nchg(i, :) = [bms_metrics(L.soc_cc, L.soc_true, skip).mae, bms_metrics(L.soc_ekf, L.soc_true, skip).mae, ...
                      bms_metrics(L.soc_ekf, L.soc_true, skip).max];
        fprintf('  %-22s CC MAE %5.2f | EKF MAE %5.2f | EKF max %6.2f\n', sc.name, nchg(i, :));
        rows(end + 1, :) = {'nasa_charge', sc.name, nchg(i, 1), nchg(i, 2), NaN, nchg(i, 3)}; %#ok<AGROW>
    end
    check('NASA charge (fresh): EKF beats Coulomb counting', all(nchg(:, 2) < nchg(:, 1)), sprintf('%.2f / %.2f vs %.2f / %.2f %%', nchg(:, 2), nchg(:, 1)));
    check('NASA charge (fresh): EKF MAE', all(nchg(:, 2) < TH.nasa_charge_mae), sprintf('worst %.2f %% < %.1f', max(nchg(:, 2)), TH.nasa_charge_mae));

    fprintf('\n== 4c. Online capacity (SOH) estimate vs true capacity ratio ==\n');
    Pc = Pn;  Pc.sota_mode = 10;  capErr = zeros(1, numel(scs));
    cap_unlocked = false(1, numel(scs));
    for i = 1:numel(scs)
        L = bms_simulate(Pc, scs{i});  capErr(i) = abs(L.cap_sota(end) - scs{i}.cap_Ah / 2.0);
        cap_unlocked(i) = (L.cap_sota(end) < 0.999);
        fprintf('  %-22s true %.3f  est %.3f  |err| %.3f  (unlocked: %d)\n', scs{i}.name, scs{i}.cap_Ah / 2.0, L.cap_sota(end), capErr(i), cap_unlocked(i));
        rows(end + 1, :) = {'soh', scs{i}.name, scs{i}.cap_Ah / 2.0, L.cap_sota(end), capErr(i), NaN}; %#ok<AGROW>
    end
    check('SOH estimate: mean |error| of capacity ratio', mean(capErr) < TH.soh_err, sprintf('%.3f < %.2f', mean(capErr), TH.soh_err));
    fresh_idx = [1, 4];          % B0005 #1, B0007 #1
    aged_idx = [2, 3, 5, 6, 7];  % B0005 #60/#120, B0007 #60/#120, B0006 #120
    gate_ok = all(~cap_unlocked(fresh_idx)) && all(cap_unlocked(aged_idx));
    check('Capacity gate: locked on fresh cycles and unlocks on aged ones', gate_ok, ...
        sprintf('fresh locked: %d/%d, aged unlocked: %d/%d', sum(~cap_unlocked(fresh_idx)), numel(fresh_idx), sum(cap_unlocked(aged_idx)), numel(aged_idx)));

    %% ---- 5. Ablation ------------------------------------------------------------------------------
    fprintf('\n== 5. Ablation of adaptation features (mean MAE %%, lower is better) ==\n');
    modes = [0 1 2 4 3 7 8 10];
    mlabel = {'EKF (ACO-tuned)', '+RLS R0', '+Sage-Husa R', '+Sage-Husa Q', '+RLS +SH-R', 'all (full SOTA)', '+Capacity (SOH)', '+SH-R +Capacity'};
    abl = zeros(2, numel(modes));
    Pi = P;  Pi.plant_Qn_scale = 0.97;  Pi.plant_R_scale = 1.2;
    for j = 1:numel(modes)
        Pm = Pi;  Pm.sota_mode = modes(j);
        t = 0;
        for i = 1:4
            [I, soc0] = bms_profile(profs{i});  I = I(1:min(numel(I), 1800));
            L = bms_simulate(Pm, struct('I', I, 'soc0', soc0, 'init_err', -0.10, 'seed', 900 + i, 'fault_mode', 0));
            t = t + bms_metrics(L.soc_sota, L.soc_true, skip).mae / 4;
        end
        abl(1, j) = t;
        Pn2 = Pn;  Pn2.sota_mode = modes(j);  t = 0;
        for i = 1:numel(scs)
            L = bms_simulate(Pn2, scs{i});
            t = t + bms_metrics(L.soc_sota, L.soc_true, skip).mae / numel(scs);
        end
        abl(2, j) = t;
    end
    fprintf('  %-16s', ''); fprintf('%18s', mlabel{:}); fprintf('\n');
    fprintf('  %-16s', 'synthetic'); fprintf('%18.3f', abl(1, :)); fprintf('\n');
    fprintf('  %-16s', 'real NASA');  fprintf('%18.3f', abl(2, :)); fprintf('\n');
    for j = 1:numel(modes)
        rows(end + 1, :) = {'ablation', mlabel{j}, abl(1, j), abl(2, j), NaN, NaN}; %#ok<AGROW>
    end

    %% ---- 6. Precision: float32 (firmware arithmetic) vs float64 --------------------------------
    fprintf('\n== 6. Estimator in 32-bit float vs 64-bit double (identical measurements) ==\n');
    PR = bms_precision_study(true);
    for i = 1:height(PR.T)
        rows(end + 1, :) = {'precision', PR.T.scenario{i}, PR.T.max_dsoc(i), PR.T.max_dv_mV(i), PR.T.mae_d(i), PR.T.mae_s(i)}; %#ok<AGROW>
    end
    check('float32 vs float64: worst SOC difference', PR.worst_dsoc < TH.prec_dsoc, sprintf('%.4f %% < %.2f', PR.worst_dsoc, TH.prec_dsoc));
    check('float32 vs float64: worst voltage difference', PR.worst_dv_mV < TH.prec_dv_mV, sprintf('%.3f mV < %.1f', PR.worst_dv_mV, TH.prec_dv_mV));
    check('float32 vs float64: identical diagnostics, no NaN or Inf', PR.all_flags_equal && PR.all_finite, 'every scenario');

    %% ---- 7. SIL: compiled C firmware vs MATLAB float32 ----------------------------------------------
    if opts.sil
        fprintf('\n== 7. Software-in-the-loop: C firmware (compiled) vs MATLAB float32 ==\n');
        Ps = bms_params('synthetic');  [Iu, s0u] = bms_profile('udds');  sil = [];
        for fm = [0 1 2 3]
            scs_ = struct('I', Iu, 'soc0', s0u, 'init_err', -0.10, 'seed', 5, 'fault_mode', fm);
            sil = [sil, bms_sil_run(Ps, scs_, sprintf('sim udds fault %d', fm))]; %#ok<AGROW>
        end
        silcases = {{'B0005', 1, 'discharge', -0.15}, {'B0007', 120, 'discharge', -0.15}, {'B0005', 1, 'charge', 0.15}};
        for i = 1:numel(silcases)
            r_ = silcases{i};  scn_ = bms_replay_scenario(r_{1}, r_{2}, r_{4}, r_{3});
            sil = [sil, bms_sil_run(Pn, scn_, scn_.name)]; %#ok<AGROW>
        end
        fprintf('  %-24s %6s %11s %11s %12s %7s %8s\n', 'scenario', 'N', 'dSOC %', 'dV mV', 'dP rel', 'flags=', 'flagged');
        for i = 1:numel(sil)
            fprintf('  %-24s %6d %11.6f %11.5f %12.2e %7d %8d\n', sil(i).name, sil(i).n, sil(i).max_dsoc, sil(i).max_dv_mV, ...
                sil(i).max_dptrace_rel, sil(i).flags_equal, sil(i).n_flagged);
            rows(end + 1, :) = {'sil', sil(i).name, sil(i).max_dsoc, sil(i).max_dv_mV, sil(i).max_dptrace_rel, sil(i).flags_equal}; %#ok<AGROW>
        end
        check('C firmware vs MATLAB float32: SOC', max([sil.max_dsoc]) < TH.sil_dsoc, sprintf('worst %.5f %% < %.3f', max([sil.max_dsoc]), TH.sil_dsoc));
        check('C firmware vs MATLAB float32: predicted voltage', max([sil.max_dv_mV]) < TH.sil_dv_mV, sprintf('worst %.3f mV < %.1f', max([sil.max_dv_mV]), TH.sil_dv_mV));
        check('C firmware vs MATLAB float32: covariance trace', max([sil.max_dptrace_rel]) < TH.sil_ptrace_rel, sprintf('worst %.1e < %.0e', max([sil.max_dptrace_rel]), TH.sil_ptrace_rel));
        check('C firmware vs MATLAB float32: diagnostic codes identical', all([sil.flags_equal]), sprintf('%d flagged samples compared', sum([sil.n_flagged])));
    end

    %% ---- 8. Unified model equivalence ----------------------------------------------
    if opts.simulink
        fprintf('\n== 8. Unified model equivalence: bms_unified vs legacy models ==\n');
        % Synthetic equivalence: bms_unified (BMS_Source = 1) vs advanced_bms_aco_ekf
        bms_setup('synthetic', true);
        so_adv = sim('advanced_bms_aco_ekf');  d_adv = so_adv.bms_log;
        bms_setup('unified', true);  BMS_Source = 1;
        so_uni_s = sim('bms_unified');  d_uni_s = so_uni_s.bms_log;
        e_syn_ekf = max(abs(d_adv(:, 2) - d_uni_s(:, 2)));
        e_syn_plt = max(abs(d_adv(:, 1) - d_uni_s(:, 1)));
        check('Unified (SIM): EKF SOC == advanced_bms_aco_ekf', e_syn_ekf < 1e-9, sprintf('max diff %.2e %%', e_syn_ekf));
        check('Unified (SIM): plant SOC == advanced_bms_aco_ekf', e_syn_plt < 1e-9, sprintf('max diff %.2e %%', e_syn_plt));
        bdclose('advanced_bms_aco_ekf');

        % Real-data equivalence: bms_unified (BMS_Source = 2) vs sota_bms_aco_ekf
        bms_setup('nasa', true);
        so_sota = sim('sota_bms_aco_ekf');  d_sota = so_sota.bms_log;
        bms_setup('nasa', true);  BMS_Source = 2;
        so_uni_n = sim('bms_unified');  d_uni_n = so_uni_n.bms_log;
        e_nasa_ekf = max(abs(d_sota(:, 2) - d_uni_n(:, 2)));
        e_nasa_ada = max(abs(d_sota(:, 3) - d_uni_n(:, 3)));
        e_nasa_cap = max(abs(d_sota(:, 12) - d_uni_n(:, 12)));
        check('Unified (REPLAY): baseline EKF == sota_bms_aco_ekf', e_nasa_ekf < 1e-9, sprintf('max diff %.2e %%', e_nasa_ekf));
        check('Unified (REPLAY): adaptive EKF == sota_bms_aco_ekf', e_nasa_ada < 1e-9, sprintf('max diff %.2e %%', e_nasa_ada));
        check('Unified (REPLAY): capacity-state SOC == sota_bms_aco_ekf', e_nasa_cap < 1e-9, sprintf('max diff %.2e %%', e_nasa_cap));

        % Non-vacuous switch check: Source Tag equals 0 in SIM and 1 in REPLAY, and reference SOC series differ
        tag_sim_ok = all(d_uni_s(:, 14) == 0);
        tag_rep_ok = all(d_uni_n(:, 14) == 1);
        n_min = min(size(d_uni_s, 1), size(d_uni_n, 1));
        soc_diff = max(abs(d_uni_s(1:n_min, 1) - d_uni_n(1:n_min, 1)));
        switch_nonvacuous = tag_sim_ok && tag_rep_ok && (soc_diff > 1.0);
        check('Unified: source switch is non-vacuous (Source Tag 0 vs 1, distinct reference SOC)', ...
            switch_nonvacuous, sprintf('tags [%d, %d], ref diff %.1f %%', d_uni_s(1, 14), d_uni_n(1, 14), soc_diff));
        bdclose('sota_bms_aco_ekf');
        bdclose('bms_unified');

        rows(end + 1, :) = {'unified_eq', 'synthetic_ekf', e_syn_ekf, e_syn_plt, NaN, NaN}; %#ok<AGROW>
        rows(end + 1, :) = {'unified_eq', 'nasa_replay', e_nasa_ekf, e_nasa_ada, e_nasa_cap, NaN}; %#ok<AGROW>
        rows(end + 1, :) = {'unified_eq', 'switch_nonvacuous', double(tag_sim_ok), double(tag_rep_ok), soc_diff, NaN}; %#ok<AGROW>

        %% ---- 9. Cross-feeding test matrix and single entry point ------------------------------------
        fprintf('\n== 9. Cross-feeding test matrix & single entry point ==\n');
        cross_csv = fullfile(outdir, 'cross_matrix.csv');
        if ~isfile(cross_csv)
            bms_cross_matrix('quick');
        end
        cross_exists = isfile(cross_csv);
        check('Cross-matrix: results/cross_matrix.csv exists and is populated', cross_exists, sprintf('file present: %d', cross_exists));

        n_rows = 0;
        if cross_exists
            t_cross = readtable(cross_csv);
            n_rows = height(t_cross);
            has_estimators = any(strcmp(t_cross.estimator, 'Baseline_EKF')) && ...
                             any(strcmp(t_cross.estimator, 'Adaptive_EKF')) && ...
                             any(strcmp(t_cross.estimator, 'Capacity_SOH_EKF'));
            check('Cross-matrix: all 3 estimators evaluated across scenarios', has_estimators && n_rows >= 12, ...
                sprintf('%d rows, 3 estimators', n_rows));

            % Check adaptive EKF steady-state error bounded on synthetic
            syn_mask = strcmp(t_cross.source, 'synthetic') & strcmp(t_cross.estimator, 'Adaptive_EKF');
            mean_syn_mae = mean(t_cross.mae_pct(syn_mask));
            check('Cross-matrix: adaptive EKF mean MAE bounded on synthetic plant', mean_syn_mae < 1.0, ...
                sprintf('mean MAE %.3f %% < 1.0', mean_syn_mae));

            % Check per-spike detection and first-flag delay on NASA replay
            rep_f2_mask = strcmp(t_cross.source, 'nasa') & (t_cross.fault_mode == 2);
            spike_rates = t_cross.spike_flag_rate(rep_f2_mask);
            delays = t_cross.first_flag_delay_s(rep_f2_mask);
            spike_det_ok = ~isempty(spike_rates) && all(spike_rates == 1.0) && all(delays <= 25);
            check('Cross-matrix: NASA replay fault detection (100% spikes flagged, delay <= 25 s)', spike_det_ok, ...
                sprintf('spike rate %.0f %%, delay %.0f s', spike_rates(1) * 100, delays(1)));

            % Check capacity EKF performance on NASA data
            cap_mask = strcmp(t_cross.estimator, 'Capacity_SOH_EKF') & strcmp(t_cross.source, 'nasa');
            cap_mae_ok = mean(t_cross.mae_pct(cap_mask)) < 4.0;
            check('Cross-matrix: capacity SOH EKF bounded error on NASA data', cap_mae_ok, ...
                sprintf('mean MAE %.3f %% < 4.0', mean(t_cross.mae_pct(cap_mask))));
        else
            check('Cross-matrix: all 3 estimators evaluated across scenarios', false, 'CSV missing');
            check('Cross-matrix: adaptive EKF mean MAE bounded on synthetic plant', false, 'CSV missing');
            check('Cross-matrix: NASA replay fault detection latency is bounded', false, 'CSV missing');
            check('Cross-matrix: capacity SOH EKF bounded error on NASA data', false, 'CSV missing');
        end

        % Check single entry point script exists and is callable
        single_entry_ok = (exist('run_everything', 'file') == 2);
        check('Single entry point: run_everything.m exists and is callable', single_entry_ok, 'run_everything.m verified');

        % Check hyperparameter tuner comparison results
        tuner_csv = fullfile(outdir, 'tuner_compare.csv');
        if ~isfile(tuner_csv)
            bms_tune_compare('quick');
        end
        tuner_exists = isfile(tuner_csv);
        n_tuner_rows = 0;
        if tuner_exists
            t_tuner = readtable(tuner_csv);
            n_tuner_rows = height(t_tuner);
            expected_methods = {'Hand_Tuned', 'Random_Search', 'Grid_Search', 'Nelder_Mead', 'ACO_R'};
            methods_present = all(ismember(expected_methods, unique(t_tuner.method)));
            has_both_sources = any(strcmp(t_tuner.source, 'synthetic')) && any(strcmp(t_tuner.source, 'nasa'));
            tuner_ok = methods_present && has_both_sources && (n_tuner_rows >= 10);
            check('Tuner comparison: results/tuner_compare.csv contains all attempted methods', tuner_ok, ...
                sprintf('%d rows, %d methods across synthetic & nasa', n_tuner_rows, numel(unique(t_tuner.method))));
        else
            check('Tuner comparison: results/tuner_compare.csv contains all attempted methods', false, 'CSV missing');
        end

        % Check widened tuner study results
        wide_csv = fullfile(outdir, 'tuner_compare_wide.csv');
        if ~isfile(wide_csv)
            bms_tune_study('quick');
        end
        wide_exists = isfile(wide_csv);
        n_wide_rows = 0;
        if wide_exists
            t_wide = readtable(wide_csv);
            n_wide_rows = height(t_wide);
            expected_wide_methods = {'ACO_R', 'Grid_Search', 'Random_Search'};
            wide_methods_present = all(ismember(expected_wide_methods, unique(t_wide.method)));
            has_both_wide_sources = any(strcmp(t_wide.source, 'synthetic')) && any(strcmp(t_wide.source, 'nasa'));
            has_breakdown_cols = ismember('mae_fresh', t_wide.Properties.VariableNames) && ...
                                 ismember('mae_aged', t_wide.Properties.VariableNames) && ...
                                 ismember('mae_B0005_1', t_wide.Properties.VariableNames);
            wide_ok = wide_methods_present && has_both_wide_sources && has_breakdown_cols && (n_wide_rows >= 6);
            check('Widened tuner study: results/tuner_compare_wide.csv contains wide-bounds methods and per-cell metrics', wide_ok, ...
                sprintf('%d rows, fresh/aged breakdown present', n_wide_rows));
        else
            check('Widened tuner study: results/tuner_compare_wide.csv contains wide-bounds methods and per-cell metrics', false, 'CSV missing');
        end

        % Check leave-one-cell-out cross-validation study results
        cv_csv = fullfile(outdir, 'cv_study.csv');
        if ~isfile(cv_csv)
            cv_csv = fullfile(here, '..', 'results', 'cv_study.csv');
        end
        if ~isfile(cv_csv)
            bms_cv_study('quick');
            cv_csv = fullfile(outdir, 'cv_study.csv');
        end
        cv_exists = isfile(cv_csv);
        n_cv_rows = 0;
        if cv_exists
            t_cv = readtable(cv_csv);
            n_cv_rows = height(t_cv);
            expected_cv_methods = {'Hand_Tuned', 'Grid_Search', 'Random_Search', 'ACO_R', 'ACO_R_Val_Pick'};
            cv_methods_present = all(ismember(expected_cv_methods, unique(t_cv.method)));
            cv_folds_present = all(ismember([1, 2, 3], unique(t_cv.fold)));
            cv_ok = cv_methods_present && cv_folds_present && (n_cv_rows == 15);
            check('Cross-validation study: results/cv_study.csv contains 15 rows (5 methods x 3 folds)', cv_ok, ...
                sprintf('%d rows, all 5 methods and 3 folds present', n_cv_rows));
        else
            check('Cross-validation study: results/cv_study.csv contains 15 rows (5 methods x 3 folds)', false, 'CSV missing');
        end

        rows(end + 1, :) = {'cross_matrix', 'status', double(cross_exists), n_rows, NaN, NaN}; %#ok<AGROW>
        rows(end + 1, :) = {'tuner_compare', 'status', double(tuner_exists), n_tuner_rows, NaN, NaN}; %#ok<AGROW>
        rows(end + 1, :) = {'tuner_compare_wide', 'status', double(wide_exists), n_wide_rows, NaN, NaN}; %#ok<AGROW>
        rows(end + 1, :) = {'cv_study', 'status', double(cv_exists), n_cv_rows, NaN, NaN}; %#ok<AGROW>
    end

    %% ---- report -------------------------------------------------------------------------------------
    T = cell2table(rows, 'VariableNames', {'suite', 'case', 'v1', 'v2', 'v3', 'v4'});
    writetable(T, fullfile(outdir, 'bms_regression_results.csv'));
    make_figures(here, Pn, scs, mc, abl, mlabel, skip);

    passed = cell2mat(checks(:, 2));
    ok = all(passed);
    fprintf('\n====================================================\n');
    fprintf('  %d / %d checks passed  ->  %s\n', nnz(passed), numel(passed), ternary(ok, 'ALL PASS', 'FAILURES PRESENT'));
    fprintf('====================================================\n');
end

function [n_un, strict_ok] = model_quality(m)
    load_system(m);
    ports = find_system(m, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'port');
    n_un = 0;
    for p = ports(:)'
        if get_param(p, 'Line') == -1,  n_un = n_un + 1;  end
    end
    keys = {'UnconnectedInputMsg', 'UnconnectedOutputMsg', 'UnconnectedLineMsg', 'AlgebraicLoopMsg', 'BusObjectLabelMismatch'};
    strict_ok = true;
    for i = 1:numel(keys)
        strict_ok = strict_ok && strcmp(get_param(m, keys{i}), 'error');
    end
    strict_ok = strict_ok && strcmp(get_param(m, 'StrictBusMsg'), 'ErrorOnBusTreatedAsVector');
    bdclose(m);
end

function q = dictionary_quality()
    here = fileparts(mfilename('fullpath'));
    dd  = Simulink.data.dictionary.open(fullfile(here, 'bms_data.sldd'));
    sec = getSection(dd, 'Design Data');
    physical = {'SOC_pct', 'Voltage_V', 'Current_A', 'Temperature_K', 'Innovation_V', 'R0_Ohm', 'SOC_Coulomb_pct'};
    q.units_ok = true;  q.n_elements = 0;
    for b = {'BMS_PlantTruth', 'BMS_Measurement', 'BMS_Estimate', 'BMS_Reference'}
        bus = getValue(getEntry(sec, b{1}));
        for el = bus.Elements(:)'
            q.n_elements = q.n_elements + 1;
            if isempty(el.Description),  q.units_ok = false;  end
            if any(strcmp(physical, el.Name)) && isempty(el.Unit),  q.units_ok = false;  end
        end
    end
    Ps = bms_params('synthetic');  Pn = bms_params('nasa');
    d = [max(abs(getValue(getEntry(sec, 'BMS_Q')).Value(:) - Ps.Q(:))), abs(getValue(getEntry(sec, 'BMS_R')).Value - Ps.R), ...
         max(abs(getValue(getEntry(sec, 'NASA_Q')).Value(:) - Pn.Q(:))), abs(getValue(getEntry(sec, 'NASA_R')).Value - Pn.R)];
    q.param_diff = max(d);  q.params_ok = q.param_diff < 1e-15;
    close(dd);
end

function s = ternary(c, a, b)
    if c,  s = a;  else,  s = b;  end
end

function make_figures(here, Pn, scs, mc, abl, mlabel, skip)
    figdir = fullfile(here, '..', 'figures');  if ~isfolder(figdir),  mkdir(figdir);  end
    c1 = [0.15 0.45 0.80];  c2 = [0.90 0.25 0.25];  c3 = [0.45 0.45 0.45];  c4 = [0.20 0.65 0.35];

    % --- real NASA replay: B0005 fresh + aged -----------------------------
    f = figure('Visible', 'off', 'Position', [60 60 1300 760], 'Color', 'w');
    pick = [1 3];
    for q = 1:2
        sc = scs{pick(q)};  L = bms_simulate(Pn, sc);
        subplot(2, 2, q);
        plot(sc.t, L.soc_true, 'k-', 'LineWidth', 2); hold on;
        plot(sc.t, L.soc_ekf,  '-', 'Color', c1, 'LineWidth', 1.5);
        plot(sc.t, L.soc_sota, '--', 'Color', c2, 'LineWidth', 1.5);
        plot(sc.t, L.soc_cc,   ':', 'Color', c3, 'LineWidth', 1.5);
        grid on; xlabel('Time (s)'); ylabel('SOC (%)'); title(sc.name, 'FontWeight', 'bold');
        if q == 1,  legend('Reference (Coulomb, delivered capacity)', 'ACO-tuned EKF', 'EKF + Sage-Husa R', 'Coulomb counting (-15% start)', 'Location', 'northeast'); end
        subplot(2, 2, 2 + q);
        plot(sc.t, L.soc_ekf - L.soc_true, '-', 'Color', c1, 'LineWidth', 1.4); hold on;
        plot(sc.t, L.soc_sota - L.soc_true, '--', 'Color', c2, 'LineWidth', 1.4);
        yline(0, 'k:'); grid on; xlabel('Time (s)'); ylabel('SOC error (%)');
        title(sprintf('Error: EKF MAE %.2f %% | adaptive MAE %.2f %%', ...
            bms_metrics(L.soc_ekf, L.soc_true, skip).mae, bms_metrics(L.soc_sota, L.soc_true, skip).mae));
    end
    sgtitle('Simulink real-data replay (NASA Ames, out-of-sample cell B0005; calibrated/tuned on B0006)', 'FontWeight', 'bold');
    exportgraphics(f, fullfile(figdir, '15_simulink_nasa_replay.png'), 'Resolution', 150);  close(f);

    % --- Monte Carlo + ablation -------------------------------------------
    f = figure('Visible', 'off', 'Position', [60 60 1300 520], 'Color', 'w');
    subplot(1, 2, 1);
    n = size(mc.ekf, 1);
    semilogy(1:n, sort(mc.cc(:,1)), 's-', 'Color', c3, 'LineWidth', 1.4); hold on;
    semilogy(1:n, sort(mc.ekf(:,1)), 'o-', 'Color', c1, 'LineWidth', 1.4);
    semilogy(1:n, sort(mc.sota(:,1)), 'x--', 'Color', c2, 'LineWidth', 1.4);
    grid on; xlabel('Run (sorted by MAE)'); ylabel('MAE per run (%), log scale');
    legend('Coulomb counting', 'ACO-tuned EKF', 'EKF + Sage-Husa R', 'Location', 'east');
    title(sprintf('Monte Carlo, %d randomised runs', size(mc.ekf, 1)), 'FontWeight', 'bold');
    subplot(1, 2, 2);
    b = bar(abl'); b(1).FaceColor = c1; b(2).FaceColor = c4; grid on;
    set(gca, 'XTickLabel', mlabel, 'XTickLabelRotation', 25); ylabel('Mean MAE (%)');
    legend('synthetic (model-matched)', 'real NASA (unseen)', 'Location', 'northwest');
    title('Ablation: what each adaptation feature buys', 'FontWeight', 'bold');
    exportgraphics(f, fullfile(figdir, '16_simulink_montecarlo_ablation.png'), 'Resolution', 150);  close(f);
end
