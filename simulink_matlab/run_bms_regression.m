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
%   Results -> results/bms_regression_results.csv and figures/15_*.png
    if nargin < 1,  opts = struct();  end
    if ~isfield(opts, 'simulink'),  opts.simulink = true;  end
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
    TH.soh_err        = 0.10;     % mean |capacity-ratio error| of the online SOH estimate
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
    end

    %% ---- 2. Monte Carlo ------------------------------------------------------
    fprintf('\n== 2. Monte Carlo (synthetic, 40 randomised runs) ==\n');
    P = bms_params('synthetic');  nMC = 40;
    profs = {'udds', 'pulse', 'cc', 'cccv'};
    rng(2024, 'twister');
    mc = struct('ekf', zeros(nMC, 4), 'sota', zeros(nMC, 4), 'cc', zeros(nMC, 4));
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

    %% ---- 3. Fault injection --------------------------------------------------------
    fprintf('\n== 3. Fault injection (synthetic UDDS, faults between t=900..1200 s) ==\n');
    [I, soc0] = bms_profile('udds');
    base = struct('I', I, 'soc0', soc0, 'init_err', -0.10, 'seed', 77, 'fault_mode', 0);
    L0 = bms_simulate(P, base);   m0 = bms_metrics(L0.soc_ekf, L0.soc_true, skip);
    names = {'none', 'current offset +0.25 A', 'voltage glitches +-0.5 V', 'thermistor open-circuit'};
    flag_bit = [0 0 8 2];         % expected diagnostic bit indexed by fault mode + 1 (mode 1 = silent drift, no flag)
    for fm = 0:3
        sc = base;  sc.fault_mode = fm;  L = bms_simulate(P, sc);
        m = bms_metrics(L.soc_ekf, L.soc_true, skip);
        win = 900:1200;
        det = any(bitand(L.fault_ekf(win), max(flag_bit(fm + 1), 1)) > 0);
        fprintf('  mode %d %-26s max err %5.2f %%  MAE %5.3f %%  flag raised in window: %d\n', fm, names{fm+1}, m.max, m.mae, det);
        rows(end + 1, :) = {'fault', names{fm+1}, m.mae, m.rmse, m.p95, m.max}; %#ok<AGROW>
        if fm == 0
            check('Fault-free run raises no hard diagnostics', ~any(bitand(L.fault_ekf, 15) > 0), ...
                sprintf('%d flagged samples', nnz(bitand(L.fault_ekf, 15) > 0)));
        else
            check(sprintf('Fault %d (%s): bounded error', fm, names{fm+1}), m.max < m0.max + TH.fault_extra, ...
                sprintf('max %.2f %% vs fault-free %.2f %%', m.max, m0.max));
            if fm >= 2
                check(sprintf('Fault %d (%s): detected by diagnostics', fm, names{fm+1}), det, 'flag bit set in fault window');
            end
        end
    end

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
    for i = 1:numel(scs)
        L = bms_simulate(Pc, scs{i});  capErr(i) = abs(L.cap_sota(end) - scs{i}.cap_Ah / 2.0);
        fprintf('  %-22s true %.3f  est %.3f  |err| %.3f\n', scs{i}.name, scs{i}.cap_Ah / 2.0, L.cap_sota(end), capErr(i));
        rows(end + 1, :) = {'soh', scs{i}.name, scs{i}.cap_Ah / 2.0, L.cap_sota(end), capErr(i), NaN}; %#ok<AGROW>
    end
    check('SOH estimate: mean |error| of capacity ratio', mean(capErr) < TH.soh_err, sprintf('%.3f < %.2f', mean(capErr), TH.soh_err));

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
