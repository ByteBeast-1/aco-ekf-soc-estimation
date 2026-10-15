function study = bms_cv_study(mode, opts)
%BMS_CV_STUDY Leave-one-cell-out cross-validation across NASA cells B0005, B0006, B0007.
%   1. Folds:
%      Fold 1: Test on B0005 (cycles 1, 60, 120), Train on B0006 & B0007 (cycles 1, 60, 120).
%      Fold 2: Test on B0006 (cycles 1, 60, 120), Train on B0005 & B0007 (cycles 1, 60, 120).
%      Fold 3: Test on B0007 (cycles 1, 60, 120), Train on B0005 & B0006 (cycles 1, 60, 120).
%   2. Methods evaluated per fold:
%      - Hand_Tuned (default baseline parameters)
%      - Grid_Search (deterministic uniform grid)
%      - Random_Search (10 seeds, original bounds)
%      - ACO_R (10 seeds, original bounds)
%      - ACO_R_Val_Pick (best of 10 seeds picked on validation cycle not in train/test)
%   3. Reports mean, median, std, worst seed, fresh (#1), and aged (#120) metrics.
%   4. Exports results/cv_study.csv and results/cv_study_runs.csv.
%   5. Evaluates what cross_matrix.csv would look like with the CV-best parameters.

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    matlab_dir = fullfile(here, '..');
    addpath(matlab_dir);
    resdir = fullfile(matlab_dir, 'results');
    if ~isfolder(resdir),  mkdir(resdir);  end
    top_resdir = fullfile(matlab_dir, '..', 'results');
    if ~isfolder(top_resdir),  mkdir(top_resdir);  end

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
        default_evals = 100;
        default_seeds = [7, 42, 101, 2024, 9999, 13, 29, 53, 79, 137]; % 10 seeds
    end

    if ~isfield(opts, 'max_evals'), opts.max_evals = default_evals; end
    if ~isfield(opts, 'seeds'),     opts.seeds     = default_seeds; end

    fprintf('========================================================================================\n');
    fprintf(' NASA Leave-One-Cell-Out Cross-Validation Benchmark (phase3-cv)\n');
    fprintf(' Mode: %s | Budget: %d evaluations | Seeds: %d\n', upper(mode), opts.max_evals, numel(opts.seeds));
    fprintf('========================================================================================\n\n');

    bnd = [-9 -4; -9 -4; -9 -4; -4 -1]; % Original bounds
    P = bms_params('nasa');
    P.docv = polyder(P.ocv);

    % Define Folds
    folds = { ...
        struct('id', 1, 'test_cell', 'B0005', 'train_cells', {{'B0006', 'B0007'}}, 'val_cell', 'B0006', 'val_cycle', 30), ...
        struct('id', 2, 'test_cell', 'B0006', 'train_cells', {{'B0005', 'B0007'}}, 'val_cell', 'B0005', 'val_cycle', 30), ...
        struct('id', 3, 'test_cell', 'B0007', 'train_cells', {{'B0005', 'B0006'}}, 'val_cell', 'B0005', 'val_cycle', 30) ...
    };

    all_runs = {};
    summary_rows = {};

    for f_idx = 1:numel(folds)
        f_def = folds{f_idx};
        fprintf('>>> FOLD %d: Test on %s | Train on %s & %s <<<\n', ...
            f_def.id, f_def.test_cell, f_def.train_cells{1}, f_def.train_cells{2});

        % Prepare data
        pre = prepare_fold_data(f_def, P);

        fitness_fn = @(lp) eval_fitness(lp, pre.train, P);
        val_fn     = @(lp) eval_fitness(lp, pre.val, P);

        % 1. Hand-Tuned
        fprintf('  Evaluating Hand_Tuned baseline...');
        t_start = tic;
        p_hand = [-6.5, -6.5, -6.5, -2.0];
        f_hand = fitness_fn(p_hand);
        f_hand_val = val_fn(p_hand);
        [mae_hand, per_sc_hand] = eval_held_out(p_hand, pre.test, P);
        rt_hand = toc(t_start);
        fprintf(' Done. (Test MAE: %.3f%%)\n', mae_hand);

        r_hand = {f_def.id, f_def.test_cell, 'Hand_Tuned', NaN, f_hand, f_hand_val, mae_hand, per_sc_hand(1), per_sc_hand(2), per_sc_hand(3), 1, rt_hand, p_hand(1), p_hand(2), p_hand(3), p_hand(4)};
        all_runs(end + 1, :) = r_hand;

        s_hand = {f_def.id, f_def.test_cell, sprintf('%s_%s', f_def.train_cells{1}, f_def.train_cells{2}), 'Hand_Tuned', ...
            mae_hand, mae_hand, 0.0, NaN, mae_hand, ...
            per_sc_hand(1), per_sc_hand(1), 0.0, NaN, per_sc_hand(1), ...
            per_sc_hand(3), per_sc_hand(3), 0.0, NaN, per_sc_hand(3), ...
            NaN, mae_hand, f_hand, f_hand_val, ...
            p_hand(1), p_hand(2), p_hand(3), p_hand(4)};
        summary_rows(end + 1, :) = s_hand;

        % 2. Grid Search
        fprintf('  Evaluating Grid_Search...');
        t_start = tic;
        [p_grid, f_grid, ev_grid] = run_grid_search(fitness_fn, bnd, opts.max_evals);
        f_grid_val = val_fn(p_grid);
        [mae_grid, per_sc_grid] = eval_held_out(p_grid, pre.test, P);
        rt_grid = toc(t_start);
        fprintf(' Done. (Test MAE: %.3f%%)\n', mae_grid);

        r_grid = {f_def.id, f_def.test_cell, 'Grid_Search', NaN, f_grid, f_grid_val, mae_grid, per_sc_grid(1), per_sc_grid(2), per_sc_grid(3), ev_grid, rt_grid, p_grid(1), p_grid(2), p_grid(3), p_grid(4)};
        all_runs(end + 1, :) = r_grid;

        s_grid = {f_def.id, f_def.test_cell, sprintf('%s_%s', f_def.train_cells{1}, f_def.train_cells{2}), 'Grid_Search', ...
            mae_grid, mae_grid, 0.0, NaN, mae_grid, ...
            per_sc_grid(1), per_sc_grid(1), 0.0, NaN, per_sc_grid(1), ...
            per_sc_grid(3), per_sc_grid(3), 0.0, NaN, per_sc_grid(3), ...
            NaN, mae_grid, f_grid, f_grid_val, ...
            p_grid(1), p_grid(2), p_grid(3), p_grid(4)};
        summary_rows(end + 1, :) = s_grid;

        % 3. Random Search (10 seeds)
        fprintf('  Evaluating Random_Search (%d seeds)...', numel(opts.seeds));
        rs_runs = [];
        for s = opts.seeds
            t_start = tic;
            [p_rs, f_rs, ev_rs] = run_random_search(fitness_fn, bnd, opts.max_evals, s);
            f_rs_val = val_fn(p_rs);
            [mae_rs, per_sc_rs] = eval_held_out(p_rs, pre.test, P);
            rt_rs = toc(t_start);
            all_runs(end + 1, :) = {f_def.id, f_def.test_cell, 'Random_Search', s, f_rs, f_rs_val, mae_rs, per_sc_rs(1), per_sc_rs(2), per_sc_rs(3), ev_rs, rt_rs, p_rs(1), p_rs(2), p_rs(3), p_rs(4)};
            rs_runs(end + 1, :) = [s, f_rs, f_rs_val, mae_rs, per_sc_rs(1), per_sc_rs(2), per_sc_rs(3), p_rs]; %#ok<AGROW>
        end
        fprintf(' Done.\n');

        rs_maes = rs_runs(:, 4);
        rs_fresh = rs_runs(:, 5);
        rs_aged = rs_runs(:, 7);
        [worst_rs_mae, worst_rs_idx]     = max(rs_maes);
        [worst_rs_fresh, worst_fresh_idx]= max(rs_fresh);
        [worst_rs_aged, worst_aged_idx]  = max(rs_aged);
        [best_rs_mae, best_rs_idx]       = min(rs_maes);

        s_rs = {f_def.id, f_def.test_cell, sprintf('%s_%s', f_def.train_cells{1}, f_def.train_cells{2}), 'Random_Search', ...
            mean(rs_maes), median(rs_maes), std(rs_maes), rs_runs(worst_rs_idx, 1), worst_rs_mae, ...
            mean(rs_fresh), median(rs_fresh), std(rs_fresh), rs_runs(worst_fresh_idx, 1), worst_rs_fresh, ...
            mean(rs_aged), median(rs_aged), std(rs_aged), rs_runs(worst_aged_idx, 1), worst_rs_aged, ...
            rs_runs(best_rs_idx, 1), best_rs_mae, ...
            mean(rs_runs(:, 2)), mean(rs_runs(:, 3)), ...
            rs_runs(best_rs_idx, 8), rs_runs(best_rs_idx, 9), rs_runs(best_rs_idx, 10), rs_runs(best_rs_idx, 11)};
        summary_rows(end + 1, :) = s_rs;

        % 4. ACO_R (10 seeds)
        fprintf('  Evaluating ACO_R (%d seeds)...', numel(opts.seeds));
        aco_runs = [];
        for s = opts.seeds
            t_start = tic;
            [p_aco, f_aco, ev_aco] = run_aco_r(fitness_fn, bnd, opts.max_evals, s);
            f_aco_val = val_fn(p_aco);
            [mae_aco, per_sc_aco] = eval_held_out(p_aco, pre.test, P);
            rt_aco = toc(t_start);
            all_runs(end + 1, :) = {f_def.id, f_def.test_cell, 'ACO_R', s, f_aco, f_aco_val, mae_aco, per_sc_aco(1), per_sc_aco(2), per_sc_aco(3), ev_aco, rt_aco, p_aco(1), p_aco(2), p_aco(3), p_aco(4)};
            aco_runs(end + 1, :) = [s, f_aco, f_aco_val, mae_aco, per_sc_aco(1), per_sc_aco(2), per_sc_aco(3), p_aco]; %#ok<AGROW>
        end
        fprintf(' Done.\n');

        aco_maes = aco_runs(:, 4);
        aco_fresh = aco_runs(:, 5);
        aco_aged = aco_runs(:, 7);
        [worst_aco_mae, worst_aco_idx]     = max(aco_maes);
        [worst_aco_fresh, worst_aco_f_idx] = max(aco_fresh);
        [worst_aco_aged, worst_aco_a_idx]  = max(aco_aged);
        [best_aco_mae, best_aco_idx]       = min(aco_maes);

        s_aco = {f_def.id, f_def.test_cell, sprintf('%s_%s', f_def.train_cells{1}, f_def.train_cells{2}), 'ACO_R', ...
            mean(aco_maes), median(aco_maes), std(aco_maes), aco_runs(worst_aco_idx, 1), worst_aco_mae, ...
            mean(aco_fresh), median(aco_fresh), std(aco_fresh), aco_runs(worst_aco_f_idx, 1), worst_aco_fresh, ...
            mean(aco_aged), median(aco_aged), std(aco_aged), aco_runs(worst_aco_a_idx, 1), worst_aco_aged, ...
            aco_runs(best_aco_idx, 1), best_aco_mae, ...
            mean(aco_runs(:, 2)), mean(aco_runs(:, 3)), ...
            aco_runs(best_aco_idx, 8), aco_runs(best_aco_idx, 9), aco_runs(best_aco_idx, 10), aco_runs(best_aco_idx, 11)};
        summary_rows(end + 1, :) = s_aco;

        % 5. ACO_R_Val_Pick (Select best seed by validation cycle loss)
        [~, val_pick_idx] = min(aco_runs(:, 3)); % Column 3 is val_fitness
        val_pick = aco_runs(val_pick_idx, :);
        fprintf('  ACO_R Validation-Pick: Seed %d chosen (val fitness: %.2e, test MAE: %.3f%%)\n\n', ...
            val_pick(1), val_pick(3), val_pick(4));

        s_val = {f_def.id, f_def.test_cell, sprintf('%s_%s', f_def.train_cells{1}, f_def.train_cells{2}), 'ACO_R_Val_Pick', ...
            val_pick(4), val_pick(4), 0.0, val_pick(1), val_pick(4), ...
            val_pick(5), val_pick(5), 0.0, val_pick(1), val_pick(5), ...
            val_pick(7), val_pick(7), 0.0, val_pick(1), val_pick(7), ...
            val_pick(1), val_pick(4), ...
            val_pick(2), val_pick(3), ...
            val_pick(8), val_pick(9), val_pick(10), val_pick(11)};
        summary_rows(end + 1, :) = s_val;
    end

    %% Export Tables
    var_names_summary = { ...
        'fold', 'test_cell', 'train_cells', 'method', ...
        'mean_mae', 'median_mae', 'std_mae', 'worst_seed', 'worst_seed_mae', ...
        'fresh_mean_mae', 'fresh_median_mae', 'fresh_std_mae', 'fresh_worst_seed', 'fresh_worst_mae', ...
        'aged_mean_mae', 'aged_median_mae', 'aged_std_mae', 'aged_worst_seed', 'aged_worst_mae', ...
        'best_seed', 'best_mae', ...
        'mean_train_fitness', 'val_fitness', ...
        'log10_Q1', 'log10_Q2', 'log10_Q3', 'log10_R' ...
    };
    tbl_summary = cell2table(summary_rows, 'VariableNames', var_names_summary);

    var_names_runs = { ...
        'fold', 'test_cell', 'method', 'seed', 'train_fitness', 'val_fitness', ...
        'test_mae', 'test_fresh', 'test_mid', 'test_aged', 'evals_used', 'runtime_s', ...
        'log10_Q1', 'log10_Q2', 'log10_Q3', 'log10_R' ...
    };
    tbl_runs = cell2table(all_runs, 'VariableNames', var_names_runs);

    csv_sum1 = fullfile(resdir, 'cv_study.csv');
    csv_sum2 = fullfile(top_resdir, 'cv_study.csv');
    writetable(tbl_summary, csv_sum1);
    writetable(tbl_summary, csv_sum2);

    csv_run1 = fullfile(resdir, 'cv_study_runs.csv');
    csv_run2 = fullfile(top_resdir, 'cv_study_runs.csv');
    writetable(tbl_runs, csv_run1);
    writetable(tbl_runs, csv_run2);

    fprintf('========================================================================================\n');
    fprintf('  CROSS-VALIDATION STUDY SUMMARY (results/cv_study.csv)\n');
    fprintf('========================================================================================\n');
    report_cv_summary(tbl_summary);

    %% Item 3: Find Best Method, Report NASA_Q & NASA_R, and evaluate cross_matrix
    study.tbl_summary = tbl_summary;
    study.tbl_runs    = tbl_runs;
    study.best_info   = evaluate_best_method_and_matrix(tbl_summary, P, resdir, top_resdir);
end

%% ==================== HELPER FUNCTIONS ====================

function pre = prepare_fold_data(f_def, P)
    % Training scenarios: 6 cycles (cycles 1, 60, 120 from 2 training cells)
    train_raw = cell(1, 6);
    c_idx = 1;
    for cell_i = 1:2
        c_name = f_def.train_cells{cell_i};
        for cyc = [1, 60, 120]
            train_raw{c_idx} = bms_replay_scenario(c_name, cyc, -0.15);
            c_idx = c_idx + 1;
        end
    end

    % Validation scenario: 1 cycle (cycle 30 of val_cell, outside train and test)
    val_raw = {bms_replay_scenario(f_def.val_cell, f_def.val_cycle, -0.15)};

    % Test scenarios: 3 cycles of test_cell (cycles 1, 60, 120)
    test_raw = { ...
        bms_replay_scenario(f_def.test_cell, 1, -0.15), ...
        bms_replay_scenario(f_def.test_cell, 60, -0.15), ...
        bms_replay_scenario(f_def.test_cell, 120, -0.15) ...
    };

    pre.train = cache_scenarios(P, train_raw);
    pre.val   = cache_scenarios(P, val_raw);
    pre.test  = cache_scenarios(P, test_raw);
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
        c.Im = L.I_meas;
        c.Vm = L.V_meas;
        c.Tm = L.T_meas;
        c.soc_true = L.soc_true;
        c.guess = min(max(s0 + sc.init_err, 0.0), 1.0);
        c.name = sc.name;

        N = numel(c.Im);
        c.R0_arr = zeros(1, N);
        c.a1_arr = zeros(1, N);
        c.b1_arr = zeros(1, N);
        c.a2_arr = zeros(1, N);
        c.b2_arr = zeros(1, N);
        for k = 1:N
            [c.R0_arr(k), ~, ~, c.a1_arr(k), c.b1_arr(k), c.a2_arr(k), c.b2_arr(k)] = bms_rc(c.Tm(k), P);
        end
        cached{i} = c;
    end
end

function soc_fast = fast_ekf_simulate(c, Q, R, P)
    N = numel(c.Im);
    ocv = P.ocv; docv = P.docv; n_ocv = numel(ocv); n_docv = numel(docv);
    x = [c.guess; 0.0; 0.0; 1.0];
    Pc = [P.P0, zeros(3, 1); zeros(1, 3), 0.0];
    Q4 = diag([Q(:); 0.0]);
    rej = 0.0;
    soc_fast = zeros(1, N);
    dt_Qn = P.eta * P.dt / P.Qn;

    F = eye(4);
    H = [0.0, -1.0, -1.0, 0.0];
    I4 = eye(4);
    xp = zeros(4, 1);

    for k = 1:N
        Ik = c.Im(k); Vk = c.Vm(k);
        a1 = c.a1_arr(k); b1 = c.b1_arr(k);
        a2 = c.a2_arr(k); b2 = c.b2_arr(k);
        R0 = c.R0_arr(k);

        th = x(4);
        soc_p = min(max(x(1) - dt_Qn * Ik / th, 0.0), 1.0);
        v1_p = a1 * x(2) + b1 * Ik;
        v2_p = a2 * x(3) + b2 * Ik;
        xp(1) = soc_p; xp(2) = v1_p; xp(3) = v2_p; xp(4) = th;

        F(1, 4) = dt_Qn * Ik / (th * th);
        F(2, 2) = a1;
        F(3, 3) = a2;
        Pp = F * Pc * F' + Q4;

        vocv = ocv(1);
        for j = 2:n_ocv, vocv = vocv * soc_p + ocv(j); end
        dv = docv(1);
        for j = 2:n_docv, dv = dv * soc_p + docv(j); end

        Vest = vocv - v1_p - v2_p - R0 * Ik;
        r = Vk - Vest;
        H(1) = dv;
        S = H * Pp * H' + R;
        nis = r * r / S;

        vbad = ~(Vk > P.V_valid(1) && Vk < P.V_valid(2));
        accept = ~vbad;
        if accept && k > P.gate_k0 && nis > P.gate_nis && rej < P.max_rej
            accept = false;
            rej = rej + 1.0;
        elseif accept
            rej = 0.0;
        end

        if accept
            K = (Pp * H') / S;
            xu = xp + K * r;
            xu(1) = min(max(xu(1), 0.0), 1.0);
            xu(4) = min(max(xu(4), P.cap_bounds(1)), P.cap_bounds(2));
            IKH = I4 - K * H;
            Pu = IKH * Pp * IKH' + (K * R) * K';
        else
            xu = xp;
            Pu = Pp;
        end
        x = xu;
        Pc = Pu;
        soc_fast(k) = xu(1) * 100.0;
    end
end

function f = eval_fitness(lp, scs, P, lam)
    if nargin < 4 || isempty(lam), lam = 0.5; end
    Q = 10.^lp(1:3);  R = 10^lp(4);
    n_sc = numel(scs);
    f = 0;
    for i = 1:n_sc
        cs = scs{i};
        soc_ekf = fast_ekf_simulate(cs, Q, R, P);
        e = (soc_ekf - cs.soc_true) / 100.0;
        kc = find(abs(e) < 0.03, 1);
        if isempty(kc), kc = numel(soc_ekf); end
        f = f + mean(e(60:end).^2) + lam * kc * P.dt / 3600.0;
    end
    f = f / n_sc;
end

function [mae, per_sc] = eval_held_out(lp, test_scs, P, skip)
    if nargin < 4 || isempty(skip), skip = 120; end
    Q = 10.^lp(1:3);  R = 10^lp(4);
    n_sc = numel(test_scs);
    per_sc = zeros(1, n_sc);
    for i = 1:n_sc
        cs = test_scs{i};
        soc_ekf = fast_ekf_simulate(cs, Q, R, P);
        N = numel(soc_ekf);
        if N > skip
            per_sc(i) = mean(abs(soc_ekf(skip+1:end) - cs.soc_true(skip+1:end)));
        else
            per_sc(i) = mean(abs(soc_ekf - cs.soc_true));
        end
    end
    mae = mean(per_sc);
end

function [best_p, best_f, n_evals] = run_grid_search(fitness_fn, bnd, max_evals)
    if max_evals >= 192
        n_pts = [4, 4, 3, 4];
    elseif max_evals >= 81
        n_pts = [3, 3, 3, 3];
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

function [best_p, best_f, n_evals] = run_random_search(fitness_fn, bnd, max_evals, seed)
    rng(seed, 'twister');
    nd = size(bnd, 1);
    pts = bnd(:, 1)' + rand(max_evals, nd) .* (bnd(:, 2) - bnd(:, 1))';

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

function [best_p, best_f, n_evals] = run_aco_r(fitness_fn, bnd, max_evals, seed)
    rng(seed, 'twister');
    nd = size(bnd, 1);
    K = min(20, max(5, round(0.10 * max_evals)));
    q = 0.1;
    xi = 0.85;

    arc = bnd(:,1)' + rand(K, nd) .* (bnd(:,2) - bnd(:,1))';
    afit = zeros(K, 1);
    for i = 1:K
        afit(i) = fitness_fn(arc(i, :));
    end
    [afit, idx] = sort(afit);
    arc = arc(idx, :);

    eval_count = K;
    w = 1 / (q * K * sqrt(2*pi)) * exp(-((1:K)' - 1).^2 / (2 * (q * K)^2));
    p = w / sum(w);

    m = max(2, min(5, round(0.05 * max_evals)));
    while eval_count < max_evals
        ants_this_iter = min(m, max_evals - eval_count);
        ns = zeros(ants_this_iter, nd);
        nf = zeros(ants_this_iter, 1);

        for a = 1:ants_this_iter
            cp = find(rand() <= cumsum(p), 1, 'first');
            if isempty(cp), cp = 1; end
            ant_sol = zeros(1, nd);
            for d = 1:nd
                sigma = xi * sum(abs(arc(:, d) - arc(cp, d))) / (K - 1);
                if sigma < 1e-4, sigma = 1e-4; end
                val = arc(cp, d) + sigma * randn();
                ant_sol(d) = min(max(val, bnd(d, 1)), bnd(d, 2));
            end
            ns(a, :) = ant_sol;
            nf(a) = fitness_fn(ant_sol);
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

function report_cv_summary(tbl)
    methods = unique(tbl.method, 'stable');
    test_cells = unique(tbl.test_cell, 'stable');

    fprintf('  %-16s %-10s %8s %8s %8s %15s %12s %12s\n', ...
        'Method', 'Test Cell', 'Mean MAE', 'Med MAE', 'Std MAE', 'Worst(Seed)', 'Fresh(Med/Worst)', 'Aged(Med/Worst)');
    fprintf('  ------------------------------------------------------------------------------------------------------\n');

    for m_idx = 1:numel(methods)
        meth = methods{m_idx};
        sub_m = tbl(strcmp(tbl.method, meth), :);
        for c_idx = 1:numel(test_cells)
            c_name = test_cells{c_idx};
            r = sub_m(strcmp(sub_m.test_cell, c_name), :);
            if ~isempty(r)
                if isnan(r.worst_seed(1))
                    worst_str = sprintf('%.3f%% (N/A)', r.worst_seed_mae(1));
                    fresh_str = sprintf('%.3f%% (N/A)', r.fresh_mean_mae(1));
                    aged_str  = sprintf('%.3f%% (N/A)', r.aged_mean_mae(1));
                else
                    worst_str = sprintf('%.3f%% (#%d)', r.worst_seed_mae(1), r.worst_seed(1));
                    fresh_str = sprintf('%.3f%% / %.3f%%', r.fresh_median_mae(1), r.fresh_worst_mae(1));
                    aged_str  = sprintf('%.3f%% / %.3f%%', r.aged_median_mae(1), r.aged_worst_mae(1));
                end
                fprintf('  %-16s %-10s %7.3f%% %7.3f%% %7.3f%% %15s %18s %18s\n', ...
                    meth, c_name, r.mean_mae(1), r.median_mae(1), r.std_mae(1), worst_str, fresh_str, aged_str);
            end
        end
        fprintf('  --> %-12s Cross-Fold Overall: Mean %.3f%% | Fresh: %.3f%% | Aged: %.3f%%\n', ...
            meth, mean(sub_m.mean_mae), mean(sub_m.fresh_mean_mae), mean(sub_m.aged_mean_mae));
        fprintf('  ------------------------------------------------------------------------------------------------------\n');
    end
end

function best_info = evaluate_best_method_and_matrix(tbl, P, resdir, top_resdir)
    methods = unique(tbl.method, 'stable');
    m_means = zeros(1, numel(methods));
    for i = 1:numel(methods)
        sub = tbl(strcmp(tbl.method, methods{i}), :);
        m_means(i) = mean(sub.mean_mae);
    end
    [best_mae, best_idx] = min(m_means);
    best_meth = methods{best_idx};

    fprintf('\n========================================================================================\n');
    fprintf('  BEST CROSS-VALIDATED METHOD: %s (Cross-Fold Mean MAE: %.3f%%)\n', best_meth, best_mae);
    fprintf('========================================================================================\n');

    sub_best = tbl(strcmp(tbl.method, best_meth), :);
    best_logQ = [mean(sub_best.log10_Q1), mean(sub_best.log10_Q2), mean(sub_best.log10_Q3)];
    best_logR = mean(sub_best.log10_R);

    best_Q = 10.^best_logQ(:);
    best_R = 10^best_logR;

    fprintf('  Optimal NASA Covariances from %s across Folds:\n', best_meth);
    fprintf('    log10 Q = [%.4f, %.4f, %.4f]  ->  Q = [%.2e, %.2e, %.2e]\n', ...
        best_logQ(1), best_logQ(2), best_logQ(3), best_Q(1), best_Q(2), best_Q(3));
    fprintf('    log10 R = %.4f               ->  R = %.2e\n', best_logR, best_R);
    fprintf('  Current Shipped ACO Tuned Covariances (from bms_tuned_data.m):\n');
    shipped = bms_tuned_data();
    fprintf('    Q = [%.2e, %.2e, %.2e],  R = %.2e\n', ...
        shipped.nasa.Q(1), shipped.nasa.Q(2), shipped.nasa.Q(3), shipped.nasa.R);
    fprintf('========================================================================================\n\n');

    best_info.method = best_meth;
    best_info.cross_fold_mae = best_mae;
    best_info.best_Q = best_Q;
    best_info.best_R = best_R;
    best_info.best_logQ = best_logQ;
    best_info.best_logR = best_logR;

    % Evaluate what cross_matrix.csv would look like with cv_best
    evaluate_cross_matrix_with_params(best_info, P, resdir, top_resdir);
end

function evaluate_cross_matrix_with_params(best_info, P_base, resdir, top_resdir)
    fprintf('>>> Evaluating cross_matrix scenarios with CV-Best parameters vs Shipped ACO <<<\n');
    orig_csv = fullfile(resdir, 'cross_matrix.csv');
    if ~isfile(orig_csv)
        orig_csv = fullfile(top_resdir, 'cross_matrix.csv');
    end

    % Define representative NASA scenarios tested in cross_matrix
    cv_scs = { ...
        bms_replay_scenario('B0005', 1, -0.15), ...
        bms_replay_scenario('B0005', 60, -0.15), ...
        bms_replay_scenario('B0005', 120, -0.15), ...
        bms_replay_scenario('B0007', 1, -0.15), ...
        bms_replay_scenario('B0007', 60, -0.15), ...
        bms_replay_scenario('B0007', 120, -0.15) ...
    };

    P_cv = P_base;
    P_cv.Q = best_info.best_Q(:);
    P_cv.R = best_info.best_R;

    rows_comp = {};
    fprintf('  %-22s | Shipped EKF MAE | CV-Best EKF MAE | Absolute Shift\n', 'Scenario');
    fprintf('  --------------------------------------------------------------------\n');

    shipped_maes = zeros(1, numel(cv_scs));
    cv_maes      = zeros(1, numel(cv_scs));

    for i = 1:numel(cv_scs)
        sc = cv_scs{i};
        sc.skip_sota = true;

        % Run with shipped
        L_ship = bms_simulate(P_base, sc);
        mae_ship = bms_metrics(L_ship.soc_ekf, L_ship.soc_true, 120).mae;
        shipped_maes(i) = mae_ship;

        % Run with cv_best
        L_cv = bms_simulate(P_cv, sc);
        mae_cv = bms_metrics(L_cv.soc_ekf, L_cv.soc_true, 120).mae;
        cv_maes(i) = mae_cv;

        shift = mae_cv - mae_ship;
        fprintf('  %-22s | %13.3f%% | %13.3f%% | %+7.3f pp\n', ...
            sc.name, mae_ship, mae_cv, shift);

        rows_comp(end + 1, :) = {sc.name, mae_ship, mae_cv, shift}; %#ok<AGROW>
    end
    fprintf('  --------------------------------------------------------------------\n');
    fprintf('  %-22s | %13.3f%% | %13.3f%% | %+7.3f pp\n', ...
        'OVERALL NASA MEAN', mean(shipped_maes), mean(cv_maes), mean(cv_maes) - mean(shipped_maes));
    fprintf('========================================================================================\n\n');

    tbl_comp = cell2table(rows_comp, 'VariableNames', {'scenario', 'shipped_mae_pct', 'cv_best_mae_pct', 'shift_pp'});
    writetable(tbl_comp, fullfile(resdir, 'cross_matrix_cv_best.csv'));
    writetable(tbl_comp, fullfile(top_resdir, 'cross_matrix_cv_best.csv'));
end
