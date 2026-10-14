function tbl = bms_tune_compare(mode, opts)
%BMS_TUNE_COMPARE Compare hyperparameter tuning methods for EKF noise covariances.
%   tbl = bms_tune_compare() runs full comparison (200 evals, 5 seeds).
%   tbl = bms_tune_compare('quick') runs quick comparison (50 evals, 3 seeds).
%   tbl = bms_tune_compare(mode, opts) allows custom options.
%
%   Methods compared:
%     1. Hand-tuned (documented reasonable default)
%     2. Random Search
%     3. Grid Search
%     4. Nelder-Mead (fminsearch)
%     5. ACO_R (continuous Ant Colony Optimization from bms_aco_tune.m)
%     6. GA / Particle Swarm (skipped if Global Optimization Toolbox is missing)
%
%   Outputs:
%     - simulink_matlab/results/tuner_compare.csv
%     - simulink_matlab/results/fig_tuner_compare.png
%     - figures/18_simulink_tuner_compare.png

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    matlab_dir = fullfile(here, '..');
    addpath(matlab_dir);
    resdir = fullfile(matlab_dir, 'results');
    if ~isfolder(resdir),  mkdir(resdir);  end
    figdir = fullfile(matlab_dir, '..', 'figures');
    if ~isfolder(figdir),  mkdir(figdir);  end

    if nargin < 1 || isempty(mode)
        mode = 'full';
    end
    if nargin < 2 || isempty(opts)
        opts = struct();
    end

    % Set default budgets
    if strcmpi(mode, 'quick')
        default_evals = 50;
        default_seeds = [7, 42, 101];
    else
        default_evals = 200;
        default_seeds = [7, 42, 101, 2024, 9999];
    end

    if ~isfield(opts, 'max_evals'),  opts.max_evals = default_evals;  end
    if ~isfield(opts, 'seeds'),      opts.seeds     = default_seeds;  end
    if ~isfield(opts, 'sources'),    opts.sources   = {'synthetic', 'nasa'};  end

    % Check toolboxes
    fprintf('=========================================================================\n');
    fprintf(' BMS EKF Hyperparameter Tuning Method Comparison\n');
    fprintf(' Mode: %s | Budget: %d evaluations | Seeds: %d\n', upper(mode), opts.max_evals, numel(opts.seeds));
    fprintf('=========================================================================\n');

    tb = ver;
    installed_tb = {tb.Name};
    has_global_optim = any(contains(installed_tb, 'Global Optimization Toolbox'));
    has_optim = any(contains(installed_tb, 'Optimization Toolbox'));
    has_fminsearch = (exist('fminsearch', 'file') == 2);
    has_ga = (exist('ga', 'file') == 2);
    has_pso = (exist('particleswarm', 'file') == 2);

    fprintf('Toolbox Check:\n');
    fprintf('  - Core MATLAB (fminsearch)       : %s\n', ternary(has_fminsearch, 'AVAILABLE', 'MISSING'));
    fprintf('  - Optimization Toolbox           : %s\n', ternary(has_optim, 'AVAILABLE', 'NOT INSTALLED'));
    fprintf('  - Global Optimization Toolbox    : %s\n', ternary(has_global_optim, 'AVAILABLE', 'NOT INSTALLED'));
    if ~has_ga
        fprintf('  [SKIP] ga: Global Optimization Toolbox not installed.\n');
    end
    if ~has_pso
        fprintf('  [SKIP] particleswarm: Global Optimization Toolbox not installed.\n');
    end
    fprintf('-------------------------------------------------------------------------\n\n');

    % Search bounds (log10 of [Q_soc, Q_v1, Q_v2, R])
    % bnd = [-9 -4; -9 -4; -9 -4; -4 -1]
    bnd = [-9 -4; -9 -4; -9 -4; -4 -1];

    rows = {};

    for s_idx = 1:numel(opts.sources)
        src = opts.sources{s_idx};
        fprintf('\n>>> Running Benchmark for Source: %s <<<\n', upper(src));
        pre = prepare_scenarios(src);
        fitness_fn = @(lp) eval_fitness(lp, pre.train, pre.P);

        % 1. Hand-Tuned (deterministic baseline)
        fprintf('  [1/5] Evaluating Hand-Tuned Baseline... ');
        t_start = tic;
        [p_hand, f_hand, ev_hand] = run_hand_tuned(fitness_fn);
        rt_hand = toc(t_start);
        mae_hand = eval_held_out(p_hand, pre.test, pre.P);
        fprintf('Done. (train fit: %.3e | test MAE: %.3f%% | rt: %.3fs)\n', f_hand, mae_hand, rt_hand);
        rows(end + 1, :) = {'Hand_Tuned', src, NaN, f_hand, mae_hand, ev_hand, rt_hand}; %#ok<AGROW>

        % 2. Random Search (stochastic, runs per seed)
        fprintf('  [2/5] Running Random Search (%d seeds x %d evals)...\n', numel(opts.seeds), opts.max_evals);
        for s = opts.seeds
            t_start = tic;
            [p_rs, f_rs, ev_rs] = run_random_search(fitness_fn, bnd, opts.max_evals, s);
            rt_rs = toc(t_start);
            mae_rs = eval_held_out(p_rs, pre.test, pre.P);
            fprintf('        Seed %5d -> train fit: %.3e | test MAE: %.3f%% | rt: %.3fs\n', s, f_rs, mae_rs, rt_rs);
            rows(end + 1, :) = {'Random_Search', src, s, f_rs, mae_rs, ev_rs, rt_rs}; %#ok<AGROW>
        end

        % 3. Grid Search (deterministic)
        fprintf('  [3/5] Running Grid Search (budget: %d)... ', opts.max_evals);
        t_start = tic;
        [p_grid, f_grid, ev_grid] = run_grid_search(fitness_fn, bnd, opts.max_evals);
        rt_grid = toc(t_start);
        mae_grid = eval_held_out(p_grid, pre.test, pre.P);
        fprintf('Done. (%d pts | train fit: %.3e | test MAE: %.3f%% | rt: %.3fs)\n', ev_grid, f_grid, mae_grid, rt_grid);
        rows(end + 1, :) = {'Grid_Search', src, NaN, f_grid, mae_grid, ev_grid, rt_grid}; %#ok<AGROW>

        % 4. Nelder-Mead fminsearch (deterministic from bounds center)
        fprintf('  [4/5] Running Nelder-Mead (fminsearch, max evals: %d)... ', opts.max_evals);
        t_start = tic;
        [p_nm, f_nm, ev_nm] = run_nelder_mead(fitness_fn, bnd, opts.max_evals);
        rt_nm = toc(t_start);
        mae_nm = eval_held_out(p_nm, pre.test, pre.P);
        fprintf('Done. (%d evals | train fit: %.3e | test MAE: %.3f%% | rt: %.3fs)\n', ev_nm, f_nm, mae_nm, rt_nm);
        rows(end + 1, :) = {'Nelder_Mead', src, NaN, f_nm, mae_nm, ev_nm, rt_nm}; %#ok<AGROW>

        % 5. ACO_R (stochastic, runs per seed)
        fprintf('  [5/5] Running ACO_R Continuous Optimization (%d seeds x %d evals)...\n', numel(opts.seeds), opts.max_evals);
        for s = opts.seeds
            t_start = tic;
            [p_aco, f_aco, ev_aco] = run_aco_r(fitness_fn, bnd, opts.max_evals, s);
            rt_aco = toc(t_start);
            mae_aco = eval_held_out(p_aco, pre.test, pre.P);
            fprintf('        Seed %5d -> train fit: %.3e | test MAE: %.3f%% | rt: %.3fs\n', s, f_aco, mae_aco, rt_aco);
            rows(end + 1, :) = {'ACO_R', src, s, f_aco, mae_aco, ev_aco, rt_aco}; %#ok<AGROW>
        end
    end

    % Construct Table
    tbl = cell2table(rows, 'VariableNames', ...
        {'method', 'source', 'seed', 'best_training_fitness', 'held_out_mae', 'evaluations_used', 'runtime_s'});

    % Save CSV to results/
    csv_file = fullfile(resdir, 'tuner_compare.csv');
    writetable(tbl, csv_file);
    fprintf('\n[OK] Wrote results to: %s\n\n', csv_file);

    % Top-level results directory copy if exists or created
    top_resdir = fullfile(matlab_dir, '..', 'results');
    if isfolder(top_resdir) || ~isfolder(top_resdir)
        try
            if ~isfolder(top_resdir), mkdir(top_resdir); end
            writetable(tbl, fullfile(top_resdir, 'tuner_compare.csv'));
        catch
        end
    end

    % Print Summary Table
    print_summary(tbl);

    % Generate Figure
    make_comparison_figure(tbl, figdir, resdir);
end

%% ==================== TUNER IMPLEMENTATIONS ====================

function [best_p, best_f, n_evals] = run_hand_tuned(fitness_fn)
    % Documented reasonable engineering baseline from literature/Python model
    % Q = [1e-6, 1e-6, 1e-6], R = 1e-2
    best_p = [-6.0, -6.0, -6.0, -2.0];
    best_f = fitness_fn(best_p);
    n_evals = 1;
end

function [best_p, best_f, n_evals] = run_random_search(fitness_fn, bnd, max_evals, seed)
    rng(seed, 'twister');
    nd = size(bnd, 1);
    pts = bnd(:,1)' + rand(max_evals, nd) .* (bnd(:,2) - bnd(:,1))';
    best_f = Inf;
    best_p = pts(1, :);
    for i = 1:max_evals
        f = fitness_fn(pts(i, :));
        if f < best_f
            best_f = f;
            best_p = pts(i, :);
        end
    end
    n_evals = max_evals;
end

function [best_p, best_f, n_evals] = run_grid_search(fitness_fn, bnd, max_evals)
    if max_evals >= 192
        n_pts = [4, 4, 4, 3]; % 192 grid points
    elseif max_evals >= 81
        n_pts = [3, 3, 3, 3]; % 81 grid points
    elseif max_evals >= 48
        n_pts = [3, 4, 2, 2]; % 48 grid points
    elseif max_evals >= 36
        n_pts = [3, 3, 2, 2]; % 36 grid points
    else
        n_pts = [2, 2, 2, 2]; % 16 grid points
    end

    g1 = linspace(bnd(1,1), bnd(1,2), n_pts(1));
    g2 = linspace(bnd(2,1), bnd(2,2), n_pts(2));
    g3 = linspace(bnd(3,1), bnd(3,2), n_pts(3));
    g4 = linspace(bnd(4,1), bnd(4,2), n_pts(4));

    [G1, G2, G3, G4] = ndgrid(g1, g2, g3, g4);
    pts = [G1(:), G2(:), G3(:), G4(:)];
    n_evals = min(size(pts, 1), max_evals);
    pts = pts(1:n_evals, :);

    best_f = Inf;
    best_p = pts(1, :);
    for i = 1:n_evals
        f = fitness_fn(pts(i, :));
        if f < best_f
            best_f = f;
            best_p = pts(i, :);
        end
    end
end

function [best_p, best_f, n_evals] = run_nelder_mead(fitness_fn, bnd, max_evals)
    lp0 = mean(bnd, 2)'; % Center of bounds
    eval_count = 0;
    best_f = Inf;
    best_p = lp0;

    function val = penalized_obj(p)
        eval_count = eval_count + 1;
        p_proj = min(max(p, bnd(:,1)'), bnd(:,2)');
        penalty = 100 * sum(max(0, bnd(:,1)' - p).^2 + max(0, p - bnd(:,2)').^2);
        raw_f = fitness_fn(p_proj);
        val = raw_f + penalty;
        if raw_f < best_f
            best_f = raw_f;
            best_p = p_proj;
        end
    end

    options = optimset('MaxFunEvals', max_evals, 'MaxIter', max_evals, 'Display', 'off');
    try
        fminsearch(@penalized_obj, lp0, options);
    catch
    end
    n_evals = eval_count;
end

function [best_p, best_f, n_evals] = run_aco_r(fitness_fn, bnd, max_evals, seed)
    rng(seed, 'twister');
    nd = size(bnd, 1);
    K = min(20, max(5, round(0.10 * max_evals)));
    q = 0.1;
    xi = 0.85;

    % Initial archive
    arc = bnd(:,1)' + rand(K, nd) .* (bnd(:,2) - bnd(:,1))';
    afit = zeros(K, 1);
    for i = 1:K
        afit(i) = fitness_fn(arc(i, :));
    end
    [afit, ix] = sort(afit);
    arc = arc(ix, :);
    eval_count = K;

    rem = max_evals - K;
    n_ants = max(2, round(K / 2));
    n_iter = floor(rem / n_ants);
    leftover = rem - (n_iter * n_ants);

    w = exp(-((1:K)' - 1).^2 / (2 * (q * K)^2));
    w = w / sum(w);
    cp = cumsum(w);

    for it = 1:n_iter
        ants_this_iter = n_ants + (it == n_iter) * leftover;
        ns = zeros(ants_this_iter, nd);
        nf = zeros(ants_this_iter, 1);
        for a = 1:ants_this_iter
            ki = find(cp >= rand(), 1);
            if isempty(ki), ki = K; end
            s = zeros(1, nd);
            for d = 1:nd
                sg = xi * sum(abs(arc(:,d) - arc(ki,d))) / (K - 1);
                s(d) = min(max(arc(ki,d) + sg * randn(), bnd(d,1)), bnd(d,2));
            end
            ns(a, :) = s;
            nf(a) = fitness_fn(s);
        end
        eval_count = eval_count + ants_this_iter;
        [af, ix] = sort([afit; nf]);
        al = [arc; ns];
        arc = al(ix(1:K), :);
        afit = af(1:K);
    end

    best_p = arc(1, :);
    best_f = afit(1);
    n_evals = eval_count;
end

%% ==================== SCENARIO PREPARATION & EVAL ====================

function pre = prepare_scenarios(src)
    P = bms_params(src);
    if strcmp(src, 'nasa')
        % Training scenarios: B0006 #1 and #60
        train_raw = {bms_replay_scenario('B0006', 1, -0.15), bms_replay_scenario('B0006', 60, -0.15)};
        % Test scenarios: unseen B0005 (#1, #60, #120) and B0007 (#1, #60, #120)
        test_raw = { ...
            bms_replay_scenario('B0005', 1, -0.15), ...
            bms_replay_scenario('B0005', 60, -0.15), ...
            bms_replay_scenario('B0005', 120, -0.15), ...
            bms_replay_scenario('B0007', 1, -0.15), ...
            bms_replay_scenario('B0007', 60, -0.15), ...
            bms_replay_scenario('B0007', 120, -0.15) ...
        };
    else
        % Training scenarios: UDDS (seed 1001) and CC (seed 1002)
        [I1, s1] = bms_profile('udds');
        [I2, s2] = bms_profile('cc');
        train_raw = { ...
            struct('I', I1, 'soc0', s1, 'init_err', -0.10, 'seed', 1001, 'fault_mode', 0), ...
            struct('I', I2(1:1800), 'soc0', s2, 'init_err', 0.10, 'seed', 1002, 'fault_mode', 0) ...
        };
        % Test scenarios: unseen seeds across all 4 profiles
        [I_u, su] = bms_profile('udds');
        [I_p, sp] = bms_profile('pulse');
        [I_c, sc_c] = bms_profile('cc');
        [I_cv, sc_cv] = bms_profile('cccv');
        test_raw = { ...
            struct('name', 'test_udds',  'I', I_u(1:min(numel(I_u), 1800)),   'soc0', su,    'init_err', -0.10, 'seed', 3001, 'fault_mode', 0), ...
            struct('name', 'test_pulse', 'I', I_p(1:min(numel(I_p), 1800)),   'soc0', sp,    'init_err',  0.10, 'seed', 3002, 'fault_mode', 0), ...
            struct('name', 'test_cc',    'I', I_c(1:min(numel(I_c), 1800)),   'soc0', sc_c,  'init_err', -0.10, 'seed', 3003, 'fault_mode', 0), ...
            struct('name', 'test_cccv',  'I', I_cv(1:min(numel(I_cv), 1800)), 'soc0', sc_cv, 'init_err',  0.10, 'seed', 3004, 'fault_mode', 0) ...
        };
    end

    pre.train = cache_scenarios(P, train_raw);
    pre.test  = cache_scenarios(P, test_raw);
    pre.P = P;
    pre.src = src;
end

function cached = cache_scenarios(P, raw_scs)
    cached = cell(1, numel(raw_scs));
    for i = 1:numel(raw_scs)
        sc = raw_scs{i};
        sc.skip_sota = true;
        L = bms_simulate(P, sc);
        if isfield(sc, 'replay') && sc.replay
            s0 = sc.soc_ref(1);
        else
            s0 = sc.soc0;
        end
        guess = min(max(s0 + sc.init_err, 0), 1);
        c.Im = L.I_meas;
        c.Vm = L.V_meas;
        c.Tm = L.T_meas;
        c.soc_true = L.soc_true;
        c.guess = guess;
        if isfield(sc, 'name'),  c.name = sc.name;  else,  c.name = sprintf('sc_%d', i);  end
        cached{i} = c;
    end
end

function f = eval_fitness(lp, train_scs, P, lam)
    if nargin < 4 || isempty(lam), lam = 0.5; end
    Q = (10.^lp(1:3))';  R = 10^lp(4);
    n_sc = numel(train_scs);
    f = 0;
    for i = 1:n_sc
        cs = train_scs{i};
        N = numel(cs.Im);
        st = bms_est_init(P, cs.guess, Q, R, 0);
        soc_ekf = zeros(1, N);
        for k = 1:N
            [st, oe] = bms_est_step(st, cs.Im(k), cs.Vm(k), cs.Tm(k), P, 0);
            soc_ekf(k) = oe.soc;
        end
        e = (soc_ekf - cs.soc_true) / 100;
        kc = find(abs(e) < 0.03, 1);
        if isempty(kc), kc = N; end
        f = f + mean(e(60:end).^2) + lam * kc * P.dt / 3600;
    end
    f = f / n_sc;
end

function mae = eval_held_out(lp, test_scs, P, skip)
    if nargin < 4 || isempty(skip), skip = 120; end
    Q = (10.^lp(1:3))';  R = 10^lp(4);
    n_sc = numel(test_scs);
    maes = zeros(1, n_sc);
    for i = 1:n_sc
        cs = test_scs{i};
        N = numel(cs.Im);
        st = bms_est_init(P, cs.guess, Q, R, 0);
        soc_ekf = zeros(1, N);
        for k = 1:N
            [st, oe] = bms_est_step(st, cs.Im(k), cs.Vm(k), cs.Tm(k), P, 0);
            soc_ekf(k) = oe.soc;
        end
        if N > skip
            maes(i) = mean(abs(soc_ekf(skip+1:end) - cs.soc_true(skip+1:end)));
        else
            maes(i) = mean(abs(soc_ekf - cs.soc_true));
        end
    end
    mae = mean(maes);
end

%% ==================== SUMMARY & PLOTTING ====================

function print_summary(tbl)
    fprintf('========================================================================================\n');
    fprintf('  HYPERPARAMETER TUNER BENCHMARK SUMMARY (Held-out Test MAE %%, Excl. First 120 s)\n');
    fprintf('========================================================================================\n');
    fprintf('  %-12s %-16s %8s %8s %8s %8s %10s %10s\n', ...
        'Source', 'Method', 'Mean MAE', 'Std MAE', 'Min MAE', 'Max MAE', 'Mean Fit', 'Mean Time');
    fprintf('  --------------------------------------------------------------------------------------\n');

    sources = unique(tbl.source);
    for s_idx = 1:numel(sources)
        src = sources{s_idx};
        sub = tbl(strcmp(tbl.source, src), :);
        methods = unique(sub.method, 'stable');
        best_mae = Inf;
        best_meth = '';
        aco_mae = NaN;

        for m_idx = 1:numel(methods)
            meth = methods{m_idx};
            m_rows = sub(strcmp(sub.method, meth), :);
            m_mae = mean(m_rows.held_out_mae);
            s_mae = std(m_rows.held_out_mae);
            min_mae = min(m_rows.held_out_mae);
            max_mae = max(m_rows.held_out_mae);
            m_fit = mean(m_rows.best_training_fitness);
            m_time = mean(m_rows.runtime_s);

            if m_mae < best_mae
                best_mae = m_mae;
                best_meth = meth;
            end
            if strcmp(meth, 'ACO_R')
                aco_mae = m_mae;
            end

            fprintf('  %-12s %-16s %7.3f%% %7.3f%% %7.3f%% %7.3f%% %10.2e %9.2fs\n', ...
                src, meth, m_mae, s_mae, min_mae, max_mae, m_fit, m_time);
        end

        % Honest Winner Declaration (Requirement 6)
        if strcmp(best_meth, 'ACO_R')
            fprintf('  >> [%s] ACO_R achieved lowest held-out MAE: %.3f%% <<\n', src, best_mae);
        else
            fprintf('  >> [%s] ACO_R did not win. Best performer was %s (%.3f%% vs ACO_R: %.3f%%) <<\n', ...
                src, best_meth, best_mae, aco_mae);
        end
        fprintf('  --------------------------------------------------------------------------------------\n');
    end
    fprintf('\n');
end

function make_comparison_figure(tbl, figdir, resdir)
    f = figure('Visible', 'off', 'Position', [80 80 1200 520], 'Color', 'w');
    sources = {'synthetic', 'nasa'};
    titles = {'Synthetic Plant (Unseen Seeds & Profiles)', 'Real NASA Replay (Unseen Cells B0005 & B0007)'};
    col = [0.20 0.45 0.75; 0.85 0.35 0.25; 0.30 0.65 0.40; 0.90 0.65 0.15; 0.55 0.30 0.70];

    for s_idx = 1:2
        src = sources{s_idx};
        subplot(1, 2, s_idx);
        sub = tbl(strcmp(tbl.source, src), :);
        methods = unique(sub.method, 'stable');
        nM = numel(methods);
        means = zeros(1, nM);
        stds = zeros(1, nM);

        for m = 1:nM
            m_rows = sub(strcmp(sub.method, methods{m}), :);
            means(m) = mean(m_rows.held_out_mae);
            stds(m) = std(m_rows.held_out_mae);
        end

        b = bar(1:nM, means, 0.65, 'FaceColor', 'flat');
        for m = 1:nM
            b.CData(m, :) = col(mod(m - 1, size(col, 1)) + 1, :);
        end
        hold on;
        errorbar(1:nM, means, stds, 'k.', 'LineWidth', 1.3, 'CapSize', 8);

        % Scatter individual seed points
        for m = 1:nM
            m_rows = sub(strcmp(sub.method, methods{m}), :);
            if height(m_rows) > 1
                scatter(m * ones(height(m_rows), 1) + 0.15 * (rand(height(m_rows), 1) - 0.5), ...
                    m_rows.held_out_mae, 25, 'k', 'filled', 'MarkerFaceAlpha', 0.6);
            end
        end

        set(gca, 'XTick', 1:nM, 'XTickLabel', strrep(methods, '_', ' '), 'XTickLabelRotation', 20);
        ylabel('Held-Out MAE (%) [Lower is Better]');
        title(titles{s_idx}, 'FontWeight', 'bold');
        grid on;
    end

    sgtitle('Hyperparameter Tuning Method Comparison: Held-Out Generalization Error', 'FontWeight', 'bold', 'FontSize', 13);

    exportgraphics(f, fullfile(resdir, 'fig_tuner_compare.png'), 'Resolution', 150);
    exportgraphics(f, fullfile(figdir, '18_simulink_tuner_compare.png'), 'Resolution', 150);
    close(f);
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end
