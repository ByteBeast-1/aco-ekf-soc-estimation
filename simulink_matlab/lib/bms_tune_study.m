function study = bms_tune_study(mode, opts)
%BMS_TUNE_STUDY Deep investigation of EKF noise covariance tuning.
%   1. Bounds Position Analysis: reports where best Q & R sit relative to search bounds.
%   2. Widened Bounds: re-runs ACO_R, Grid_Search, Random_Search with bounds widened
%      by one decade on each side ([-10 -3; -10 -3; -10 -3; -5 0]), 10 seeds, 200 evals.
%      Outputs results/tuner_compare_wide.csv.
%   3. Per-cell Held-Out Analysis: reports MAE for B0005 & B0007 at cycles 1, 60, 120,
%      and fresh (#1) vs aged (#120) breakdown.
%   4. Overfitting Analysis: compares training fitness against held-out MAE.
%   5. Training Diversity Variant: trains ACO on B0006 cycles 1, 60, and 120,
%      evaluates on B0005 & B0007, and tests whether diversity improves generalization.
%      Does NOT overwrite bms_tuned_data.m.

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    matlab_dir = fullfile(here, '..');
    addpath(matlab_dir);
    resdir = fullfile(matlab_dir, 'results');
    if ~isfolder(resdir),  mkdir(resdir);  end
    top_resdir = fullfile(matlab_dir, '..', 'results');
    if ~isfolder(top_resdir),  mkdir(top_resdir);  end
    figdir = fullfile(matlab_dir, '..', 'figures');
    if ~isfolder(figdir),  mkdir(figdir);  end

    if nargin < 1 || isempty(mode)
        mode = 'full';
    end
    if nargin < 2 || isempty(opts)
        opts = struct();
    end

    if strcmpi(mode, 'quick')
        default_evals = 50;
        default_seeds = [7, 42, 101];
    else
        default_evals = 200;
        default_seeds = [7, 42, 101, 2024, 9999, 13, 29, 53, 79, 137]; % 10 seeds
    end

    if ~isfield(opts, 'max_evals'), opts.max_evals = default_evals; end
    if ~isfield(opts, 'seeds'),     opts.seeds     = default_seeds; end

    fprintf('========================================================================================\n');
    fprintf(' BMS EKF Hyperparameter Tuning In-Depth Study (phase3-tuners-b)\n');
    fprintf(' Mode: %s | Budget: %d evaluations | Seeds: %d\n', upper(mode), opts.max_evals, numel(opts.seeds));
    fprintf('========================================================================================\n\n');

    study = struct();

    %% -------------------------------------------------------------------------
    %% 1 & 4. Original Bounds Analysis (Bounds saturation & Overfitting check)
    %% -------------------------------------------------------------------------
    fprintf('>>> Stage 1: Evaluating Original Bounds Saturation & Overfitting <<<\n');
    bnd_orig = [-9 -4; -9 -4; -9 -4; -4 -1];
    pre_nasa_orig = prepare_scenarios('nasa', [1, 60]);
    pre_syn_orig  = prepare_scenarios('synthetic', []);

    study.orig_nasa = run_methods_suite(pre_nasa_orig, bnd_orig, opts.max_evals, opts.seeds(1:min(5, numel(opts.seeds))), false);
    study.orig_syn  = run_methods_suite(pre_syn_orig,  bnd_orig, opts.max_evals, opts.seeds(1:min(5, numel(opts.seeds))), false);

    report_bounds_position(study.orig_nasa, bnd_orig, 'NASA (Original Bounds: Q in [-9, -4], R in [-4, -1])');
    report_bounds_position(study.orig_syn,  bnd_orig, 'Synthetic (Original Bounds: Q in [-9, -4], R in [-4, -1])');

    %% -------------------------------------------------------------------------
    %% 2 & 3. Widened Bounds Benchmark (results/tuner_compare_wide.csv)
    %% -------------------------------------------------------------------------
    fprintf('\n>>> Stage 2: Re-running ACO_R, Grid_Search, Random_Search with Widened Bounds (10 seeds) <<<\n');
    % Widened by one decade on each side:
    % Q: [-10, -3], R: [-5, 0]
    bnd_wide = [-10 -3; -10 -3; -10 -3; -5 0];
    fprintf('  Widened bounds: log10 Q in [-10, -3], log10 R in [-5, 0]\n');

    pre_nasa_wide = prepare_scenarios('nasa', [1, 60]);
    pre_syn_wide  = prepare_scenarios('synthetic', []);

    res_nasa_wide = run_wide_methods(pre_nasa_wide, bnd_wide, opts.max_evals, opts.seeds);
    res_syn_wide  = run_wide_methods(pre_syn_wide,  bnd_wide, opts.max_evals, opts.seeds);

    wide_rows = [res_syn_wide.rows; res_nasa_wide.rows];
    tbl_wide = cell2table(wide_rows, 'VariableNames', { ...
        'method', 'source', 'seed', 'best_training_fitness', 'held_out_mae', 'evaluations_used', 'runtime_s', ...
        'log10_Q1', 'log10_Q2', 'log10_Q3', 'log10_R', ...
        'frac_bnd_Q1', 'frac_bnd_Q2', 'frac_bnd_Q3', 'frac_bnd_R', 'is_at_bound', ...
        'mae_B0005_1', 'mae_B0005_60', 'mae_B0005_120', 'mae_B0007_1', 'mae_B0007_60', 'mae_B0007_120', ...
        'mae_fresh', 'mae_aged'});

    csv_wide1 = fullfile(resdir, 'tuner_compare_wide.csv');
    csv_wide2 = fullfile(top_resdir, 'tuner_compare_wide.csv');
    writetable(tbl_wide, csv_wide1);
    writetable(tbl_wide, csv_wide2);
    fprintf('  [OK] Exported widened bounds results to: %s\n', csv_wide1);
    study.tbl_wide = tbl_wide;

    report_wide_summary(tbl_wide);

    %% -------------------------------------------------------------------------
    %% 5. Diverse Training Variant (B0006 cycles 1, 60, and 120)
    %% -------------------------------------------------------------------------
    fprintf('\n>>> Stage 3: Testing Training Diversity Variant (B0006 cycles 1, 60, and 120) <<<\n');
    pre_nasa_diverse = prepare_scenarios('nasa', [1, 60, 120]);
    seeds_div = opts.seeds(1:min(5, numel(opts.seeds)));
    res_diverse = run_diverse_variant(pre_nasa_diverse, pre_nasa_orig, bnd_orig, opts.max_evals, seeds_div);
    study.res_diverse = res_diverse;

    report_diverse_comparison(res_diverse);
end

%% ==================== METHODS SUITE EXECUTION ====================

function out = run_methods_suite(pre, bnd, max_evals, seeds, is_wide)
    fitness_fn = @(lp) eval_fitness(lp, pre.train, pre.P);
    rows = {};
    details = struct();

    % 1. Hand-tuned
    t_start = tic;
    [p_hand, f_hand, ev_hand] = run_hand_tuned(fitness_fn);
    rt_hand = toc(t_start);
    [mae_hand, per_sc_hand] = eval_held_out_detailed(p_hand, pre.test, pre.P);
    r_hand = make_row('Hand_Tuned', pre.src, NaN, f_hand, mae_hand, ev_hand, rt_hand, p_hand, bnd, per_sc_hand, pre.src);
    rows(end + 1, :) = r_hand;
    details.Hand_Tuned = struct('p', p_hand, 'f', f_hand, 'mae', mae_hand, 'per_sc', per_sc_hand);

    % 2. Random Search
    rs_p = []; rs_f = []; rs_mae = []; rs_persc = [];
    for s = seeds
        t_start = tic;
        [p_rs, f_rs, ev_rs] = run_random_search(fitness_fn, bnd, max_evals, s);
        rt_rs = toc(t_start);
        [mae_rs, per_sc_rs] = eval_held_out_detailed(p_rs, pre.test, pre.P);
        r_rs = make_row('Random_Search', pre.src, s, f_rs, mae_rs, ev_rs, rt_rs, p_rs, bnd, per_sc_rs, pre.src);
        rows(end + 1, :) = r_rs;
        rs_p(end + 1, :) = p_rs; %#ok<AGROW>
        rs_f(end + 1) = f_rs; %#ok<AGROW>
        rs_mae(end + 1) = mae_rs; %#ok<AGROW>
        rs_persc(end + 1, :) = per_sc_rs; %#ok<AGROW>
    end
    details.Random_Search = struct('p', rs_p, 'f', rs_f, 'mae', rs_mae, 'per_sc', rs_persc);

    % 3. Grid Search
    t_start = tic;
    [p_grid, f_grid, ev_grid] = run_grid_search(fitness_fn, bnd, max_evals);
    rt_grid = toc(t_start);
    [mae_grid, per_sc_grid] = eval_held_out_detailed(p_grid, pre.test, pre.P);
    r_grid = make_row('Grid_Search', pre.src, NaN, f_grid, mae_grid, ev_grid, rt_grid, p_grid, bnd, per_sc_grid, pre.src);
    rows(end + 1, :) = r_grid;
    details.Grid_Search = struct('p', p_grid, 'f', f_grid, 'mae', mae_grid, 'per_sc', per_sc_grid);

    % 4. Nelder-Mead (only in standard suite)
    if ~is_wide
        t_start = tic;
        [p_nm, f_nm, ev_nm] = run_nelder_mead(fitness_fn, bnd, max_evals);
        rt_nm = toc(t_start);
        [mae_nm, per_sc_nm] = eval_held_out_detailed(p_nm, pre.test, pre.P);
        r_nm = make_row('Nelder_Mead', pre.src, NaN, f_nm, mae_nm, ev_nm, rt_nm, p_nm, bnd, per_sc_nm, pre.src);
        rows(end + 1, :) = r_nm;
        details.Nelder_Mead = struct('p', p_nm, 'f', f_nm, 'mae', mae_nm, 'per_sc', per_sc_nm);
    end

    % 5. ACO_R
    aco_p = []; aco_f = []; aco_mae = []; aco_persc = [];
    for s = seeds
        t_start = tic;
        [p_aco, f_aco, ev_aco] = run_aco_r(fitness_fn, bnd, max_evals, s);
        rt_aco = toc(t_start);
        [mae_aco, per_sc_aco] = eval_held_out_detailed(p_aco, pre.test, pre.P);
        r_aco = make_row('ACO_R', pre.src, s, f_aco, mae_aco, ev_aco, rt_aco, p_aco, bnd, per_sc_aco, pre.src);
        rows(end + 1, :) = r_aco;
        aco_p(end + 1, :) = p_aco; %#ok<AGROW>
        aco_f(end + 1) = f_aco; %#ok<AGROW>
        aco_mae(end + 1) = mae_aco; %#ok<AGROW>
        aco_persc(end + 1, :) = per_sc_aco; %#ok<AGROW>
    end
    details.ACO_R = struct('p', aco_p, 'f', aco_f, 'mae', aco_mae, 'per_sc', aco_persc);

    out.rows = rows;
    out.details = details;
    out.src = pre.src;
end

function out = run_wide_methods(pre, bnd, max_evals, seeds)
    fitness_fn = @(lp) eval_fitness(lp, pre.train, pre.P);
    rows = {};
    details = struct();

    % 1. Random Search (10 seeds)
    rs_p = []; rs_f = []; rs_mae = []; rs_persc = [];
    fprintf('  Running Random Search on widened bounds (%d seeds)...', numel(seeds));
    for s = seeds
        t_start = tic;
        [p_rs, f_rs, ev_rs] = run_random_search(fitness_fn, bnd, max_evals, s);
        rt_rs = toc(t_start);
        [mae_rs, per_sc_rs] = eval_held_out_detailed(p_rs, pre.test, pre.P);
        r_rs = make_row('Random_Search', pre.src, s, f_rs, mae_rs, ev_rs, rt_rs, p_rs, bnd, per_sc_rs, pre.src);
        rows(end + 1, :) = r_rs;
        rs_p(end + 1, :) = p_rs; %#ok<AGROW>
        rs_f(end + 1) = f_rs; %#ok<AGROW>
        rs_mae(end + 1) = mae_rs; %#ok<AGROW>
        rs_persc(end + 1, :) = per_sc_rs; %#ok<AGROW>
    end
    fprintf(' Done.\n');
    details.Random_Search = struct('p', rs_p, 'f', rs_f, 'mae', rs_mae, 'per_sc', rs_persc);

    % 2. Grid Search (1 run, 192 pts)
    fprintf('  Running Grid Search on widened bounds...');
    t_start = tic;
    [p_grid, f_grid, ev_grid] = run_grid_search(fitness_fn, bnd, max_evals);
    rt_grid = toc(t_start);
    [mae_grid, per_sc_grid] = eval_held_out_detailed(p_grid, pre.test, pre.P);
    r_grid = make_row('Grid_Search', pre.src, NaN, f_grid, mae_grid, ev_grid, rt_grid, p_grid, bnd, per_sc_grid, pre.src);
    rows(end + 1, :) = r_grid;
    details.Grid_Search = struct('p', p_grid, 'f', f_grid, 'mae', mae_grid, 'per_sc', per_sc_grid);
    fprintf(' Done. (MAE: %.3f%%)\n', mae_grid);

    % 3. ACO_R (10 seeds)
    aco_p = []; aco_f = []; aco_mae = []; aco_persc = [];
    fprintf('  Running ACO_R on widened bounds (%d seeds)...', numel(seeds));
    for s = seeds
        t_start = tic;
        [p_aco, f_aco, ev_aco] = run_aco_r(fitness_fn, bnd, max_evals, s);
        rt_aco = toc(t_start);
        [mae_aco, per_sc_aco] = eval_held_out_detailed(p_aco, pre.test, pre.P);
        r_aco = make_row('ACO_R', pre.src, s, f_aco, mae_aco, ev_aco, rt_aco, p_aco, bnd, per_sc_aco, pre.src);
        rows(end + 1, :) = r_aco;
        aco_p(end + 1, :) = p_aco; %#ok<AGROW>
        aco_f(end + 1) = f_aco; %#ok<AGROW>
        aco_mae(end + 1) = mae_aco; %#ok<AGROW>
        aco_persc(end + 1, :) = per_sc_aco; %#ok<AGROW>
    end
    fprintf(' Done.\n');
    details.ACO_R = struct('p', aco_p, 'f', aco_f, 'mae', aco_mae, 'per_sc', aco_persc);

    out.rows = rows;
    out.details = details;
    out.src = pre.src;
end

function out = run_diverse_variant(pre_div, pre_orig, bnd, max_evals, seeds)
    fn_div  = @(lp) eval_fitness(lp, pre_div.train, pre_div.P);
    fn_orig = @(lp) eval_fitness(lp, pre_orig.train, pre_orig.P);

    aco_2c_mae = zeros(1, numel(seeds));
    aco_3c_mae = zeros(1, numel(seeds));
    aco_2c_fresh = zeros(1, numel(seeds));
    aco_3c_fresh = zeros(1, numel(seeds));
    aco_2c_aged  = zeros(1, numel(seeds));
    aco_3c_aged  = zeros(1, numel(seeds));

    fprintf('  Comparing ACO_R on 2-cycle training ([#1, #60]) vs 3-cycle training ([#1, #60, #120])...\n');
    for i = 1:numel(seeds)
        s = seeds(i);
        % 2-cycle baseline
        [p2, f2] = run_aco_r(fn_orig, bnd, max_evals, s);
        [m2, ps2] = eval_held_out_detailed(p2, pre_orig.test, pre_orig.P);
        aco_2c_mae(i) = m2;
        aco_2c_fresh(i) = (ps2(1) + ps2(4)) / 2;
        aco_2c_aged(i)  = (ps2(3) + ps2(6)) / 2;

        % 3-cycle diverse variant
        [p3, f3] = run_aco_r(fn_div, bnd, max_evals, s);
        [m3, ps3] = eval_held_out_detailed(p3, pre_orig.test, pre_div.P);
        aco_3c_mae(i) = m3;
        aco_3c_fresh(i) = (ps3(1) + ps3(4)) / 2;
        aco_3c_aged(i)  = (ps3(3) + ps3(6)) / 2;

        fprintf('    Seed %5d | 2-cycle MAE: %.3f%% (fit: %.2e) | 3-cycle MAE: %.3f%% (fit: %.2e)\n', ...
            s, m2, f2, m3, f3);
    end

    out.seeds = seeds;
    out.aco_2c_mae = aco_2c_mae;
    out.aco_3c_mae = aco_3c_mae;
    out.aco_2c_fresh = aco_2c_fresh;
    out.aco_3c_fresh = aco_3c_fresh;
    out.aco_2c_aged  = aco_2c_aged;
    out.aco_3c_aged  = aco_3c_aged;
end

%% ==================== HELPER ROW MAKER ====================

function r = make_row(method, src, seed, fit, mae, evals, rt, p, bnd, per_sc, src_type)
    % Compute fraction relative to bounds
    frac = (p - bnd(:,1)') ./ (bnd(:,2)' - bnd(:,1)');
    at_bound = nnz(frac <= 0.02 | frac >= 0.98);

    if strcmp(src_type, 'nasa') && numel(per_sc) >= 6
        m_b5_1  = per_sc(1);
        m_b5_60 = per_sc(2);
        m_b5_120= per_sc(3);
        m_b7_1  = per_sc(4);
        m_b7_60 = per_sc(5);
        m_b7_120= per_sc(6);
        m_fresh = (m_b5_1 + m_b7_1) / 2.0;
        m_aged  = (m_b5_120 + m_b7_120) / 2.0;
    else
        m_b5_1  = NaN; m_b5_60 = NaN; m_b5_120 = NaN;
        m_b7_1  = NaN; m_b7_60 = NaN; m_b7_120 = NaN;
        m_fresh = NaN; m_aged  = NaN;
    end

    r = {method, src, seed, fit, mae, evals, rt, ...
         p(1), p(2), p(3), p(4), ...
         frac(1), frac(2), frac(3), frac(4), at_bound, ...
         m_b5_1, m_b5_60, m_b5_120, m_b7_1, m_b7_60, m_b7_120, ...
         m_fresh, m_aged};
end

%% ==================== TUNER ALGORITHMS ====================

function [best_p, best_f, n_evals] = run_hand_tuned(fitness_fn)
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
        n_pts = [4, 4, 4, 3];
    elseif max_evals >= 81
        n_pts = [3, 3, 3, 3];
    elseif max_evals >= 48
        n_pts = [3, 4, 2, 2];
    elseif max_evals >= 36
        n_pts = [3, 3, 2, 2];
    else
        n_pts = [2, 2, 2, 2];
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
    lp0 = mean(bnd, 2)';
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

    arc = bnd(:,1)' + rand(K, nd) .* (bnd(:,2) - bnd(:,1))';
    afit = zeros(K, 1);
    for i = 1:K, afit(i) = fitness_fn(arc(i, :)); end
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

%% ==================== SCENARIO PREPARATION ====================

function pre = prepare_scenarios(src, train_cycles)
    P = bms_params(src);
    if strcmp(src, 'nasa')
        if nargin < 2 || isempty(train_cycles)
            train_cycles = [1, 60];
        end
        train_raw = cell(1, numel(train_cycles));
        for i = 1:numel(train_cycles)
            train_raw{i} = bms_replay_scenario('B0006', train_cycles(i), -0.15);
        end
        test_raw = { ...
            bms_replay_scenario('B0005', 1, -0.15), ...
            bms_replay_scenario('B0005', 60, -0.15), ...
            bms_replay_scenario('B0005', 120, -0.15), ...
            bms_replay_scenario('B0007', 1, -0.15), ...
            bms_replay_scenario('B0007', 60, -0.15), ...
            bms_replay_scenario('B0007', 120, -0.15) ...
        };
    else
        [I1, s1] = bms_profile('udds');
        [I2, s2] = bms_profile('cc');
        train_raw = { ...
            struct('I', I1, 'soc0', s1, 'init_err', -0.10, 'seed', 1001, 'fault_mode', 0), ...
            struct('I', I2(1:1800), 'soc0', s2, 'init_err', 0.10, 'seed', 1002, 'fault_mode', 0) ...
        };
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
        if isfield(sc, 'name'), c.name = sc.name; else, c.name = sprintf('sc_%d', i); end
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

function [mae, per_sc] = eval_held_out_detailed(lp, test_scs, P, skip)
    if nargin < 4 || isempty(skip), skip = 120; end
    Q = (10.^lp(1:3))';  R = 10^lp(4);
    n_sc = numel(test_scs);
    per_sc = zeros(1, n_sc);
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
            per_sc(i) = mean(abs(soc_ekf(skip+1:end) - cs.soc_true(skip+1:end)));
        else
            per_sc(i) = mean(abs(soc_ekf - cs.soc_true));
        end
    end
    mae = mean(per_sc);
end

%% ==================== REPORTING FUNCTIONS ====================

function report_bounds_position(res, bnd, label)
    fprintf('\n----------------------------------------------------------------------------------------\n');
    fprintf('  BOUNDS SATURATION REPORT: %s\n', label);
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf('  %-16s %10s %10s %10s %10s  | %12s | %8s\n', ...
        'Method', 'u(Q_soc)', 'u(Q_v1)', 'u(Q_v2)', 'u(R)', 'At-Bound Par', 'Held-Out');
    fprintf('  %-16s %10s %10s %10s %10s  | %12s | %8s\n', ...
        '', '[0..1]', '[0..1]', '[0..1]', '[0..1]', 'Fraction', 'MAE (%)');
    fprintf('  --------------------------------------------------------------------------------------\n');

    fn = fieldnames(res.details);
    total_pars = 0;
    total_at_bounds = 0;

    for i = 1:numel(fn)
        m = fn{i};
        d = res.details.(m);
        p_mean = mean(d.p, 1);
        mae_mean = mean(d.mae);
        u = (p_mean - bnd(:,1)') ./ (bnd(:,2)' - bnd(:,1)');
        at_bnd = nnz(u <= 0.02 | u >= 0.98);
        frac_str = sprintf('%d / %d (%.0f%%)', at_bnd, 4, (at_bnd / 4) * 100);

        total_pars = total_pars + 4;
        total_at_bounds = total_at_bounds + at_bnd;

        fprintf('  %-16s %9.1f%% %9.1f%% %9.1f%% %9.1f%%  | %12s | %7.3f%%\n', ...
            m, u(1)*100, u(2)*100, u(3)*100, u(4)*100, frac_str, mae_mean);
    end
    fprintf('  --------------------------------------------------------------------------------------\n');
    fprintf('  >> OVERALL AT-BOUND FRACTION: %d / %d parameters (%.1f%%) sit at search boundaries! <<\n', ...
        total_at_bounds, total_pars, (total_at_bounds / total_pars) * 100);
    fprintf('----------------------------------------------------------------------------------------\n');
end

function report_wide_summary(tbl)
    fprintf('\n========================================================================================\n');
    fprintf('  WIDENED BOUNDS BENCHMARK SUMMARY (results/tuner_compare_wide.csv)\n');
    fprintf('  Search Bounds: log10 Q in [-10, -3], log10 R in [-5, 0] (10 Seeds x 200 Evaluations)\n');
    fprintf('========================================================================================\n');
    fprintf('  %-10s %-15s %8s %8s %8s %8s | %8s %8s %8s\n', ...
        'Source', 'Method', 'Mean MAE', 'Std MAE', 'Min MAE', 'Max MAE', 'Fresh #1', 'Aged 120', 'TrainFit');
    fprintf('  --------------------------------------------------------------------------------------\n');

    sources = unique(tbl.source);
    for s_idx = 1:numel(sources)
        src = sources{s_idx};
        sub = tbl(strcmp(tbl.source, src), :);
        methods = unique(sub.method, 'stable');

        for m_idx = 1:numel(methods)
            meth = methods{m_idx};
            m_rows = sub(strcmp(sub.method, meth), :);
            m_mae = mean(m_rows.held_out_mae);
            s_mae = std(m_rows.held_out_mae);
            min_mae = min(m_rows.held_out_mae);
            max_mae = max(m_rows.held_out_mae);
            m_fit = mean(m_rows.best_training_fitness);
            m_fresh = mean(m_rows.mae_fresh);
            m_aged = mean(m_rows.mae_aged);

            if isnan(m_fresh)
                fprintf('  %-10s %-15s %7.3f%% %7.3f%% %7.3f%% %7.3f%% | %8s %8s %9.2e\n', ...
                    src, meth, m_mae, s_mae, min_mae, max_mae, 'N/A', 'N/A', m_fit);
            else
                fprintf('  %-10s %-15s %7.3f%% %7.3f%% %7.3f%% %7.3f%% | %7.3f%% %7.3f%% %9.2e\n', ...
                    src, meth, m_mae, s_mae, min_mae, max_mae, m_fresh, m_aged, m_fit);
            end
        end
        fprintf('  --------------------------------------------------------------------------------------\n');
    end
end

function report_diverse_comparison(res)
    fprintf('========================================================================================\n');
    fprintf('  TRAINING DIVERSITY VARIANT REPORT (NASA Replay Held-out Error on Unseen B0005 & B0007)\n');
    fprintf('========================================================================================\n');
    fprintf('  Training Set                    Mean Held-Out MAE    Fresh (#1) Cells    Aged (#120) Cells\n');
    fprintf('  --------------------------------------------------------------------------------------\n');
    fprintf('  2-Cycle Baseline (B0006 #1, #60)    %7.3f%%             %7.3f%%             %7.3f%%\n', ...
        mean(res.aco_2c_mae), mean(res.aco_2c_fresh), mean(res.aco_2c_aged));
    fprintf('  3-Cycle Diverse  (B0006 #1,#60,#120)%7.3f%%             %7.3f%%             %7.3f%%\n', ...
        mean(res.aco_3c_mae), mean(res.aco_3c_fresh), mean(res.aco_3c_aged));
    fprintf('  --------------------------------------------------------------------------------------\n');

    diff_mae = mean(res.aco_3c_mae) - mean(res.aco_2c_mae);
    if diff_mae < 0
        fprintf('  >> RESULT: Training diversity IMPROVED held-out generalization by %.3f pp (%.3f%% -> %.3f%%)! <<\n', ...
            abs(diff_mae), mean(res.aco_2c_mae), mean(res.aco_3c_mae));
    else
        fprintf('  >> RESULT: Training diversity shifted overall held-out MAE by %+.3f pp (%.3f%% vs %.3f%%). <<\n', ...
            diff_mae, mean(res.aco_3c_mae), mean(res.aco_2c_mae));
    end
    fprintf('========================================================================================\n\n');
end
