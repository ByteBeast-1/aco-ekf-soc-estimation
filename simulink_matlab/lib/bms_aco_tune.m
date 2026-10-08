function T = bms_aco_tune(variant, n_ants, n_iter, seed, write_file)
%BMS_ACO_TUNE Multi-objective continuous ACO (ACO_R) tuning of the EKF noise covariances.
%   Decision vector (log10): [Q_soc, Q_v1, Q_v2, R].
%   Fitness = SOC MSE + lambda * convergence time [h]  (accuracy + speed; averaged over training scenarios).
%   Training data is DISJOINT from the evaluation data:
%       'synthetic' : UDDS + CC profiles with their own noise seeds (regression uses other seeds)
%       'nasa'      : real cell B0006 discharge #1 (regression uses B0005 / B0007 and aged cycles)
%   The result is written to lib/bms_tuned_data.m so the Simulink models consume the optimiser output.
    if nargin < 1,  variant  = 'synthetic';  end
    if nargin < 2,  n_ants   = 20;  end
    if nargin < 3,  n_iter   = 30;  end
    if nargin < 4,  seed     = 7;   end
    if nargin < 5,  write_file = true;  end

    rng(seed, 'twister');
    P = bms_params(variant);
    scs = training_scenarios(variant);

    K = 20;  q = 0.1;  xi = 0.85;  lam = 0.5;
    bnd = [-9 -4; -9 -4; -9 -4; -4 -1];        % log10 bounds [Q1 Q2 Q3 R]
    nd  = 4;

    arc  = bnd(:,1)' + rand(K, nd) .* (bnd(:,2) - bnd(:,1))';
    afit = zeros(K, 1);
    for i = 1:K,  afit(i) = fitness(arc(i,:), P, scs, lam);  end
    [afit, ix] = sort(afit);  arc = arc(ix, :);
    hist = zeros(1, n_iter);

    w  = exp(-((1:K)' - 1).^2 / (2 * (q * K)^2));  w = w / sum(w);  cp = cumsum(w);
    for it = 1:n_iter
        ns = zeros(n_ants, nd);  nf = zeros(n_ants, 1);
        for a = 1:n_ants
            ki = find(cp >= rand(), 1);  if isempty(ki),  ki = K;  end
            s = zeros(1, nd);
            for d = 1:nd
                sg = xi * sum(abs(arc(:,d) - arc(ki,d))) / (K - 1);
                s(d) = min(max(arc(ki,d) + sg * randn(), bnd(d,1)), bnd(d,2));
            end
            ns(a,:) = s;  nf(a) = fitness(s, P, scs, lam);
        end
        [af, ix] = sort([afit; nf]);  al = [arc; ns];
        arc = al(ix(1:K), :);  afit = af(1:K);  hist(it) = afit(1);
    end

    b = arc(1,:);
    T.Q = (10.^b(1:3))';  T.R = 10^b(4);  T.fitness = afit(1);  T.history = hist;  T.variant = variant;
    fprintf('  [MO-ACOR/%s] Q* = [%.2e %.2e %.2e]  R* = %.3e  fitness = %.3e\n', variant, T.Q, T.R, T.fitness);

    if write_file,  write_tuned(variant, T);  end
end

function scs = training_scenarios(variant)
    if strcmp(variant, 'nasa')
        scs = {bms_replay_scenario('B0006', 1, -0.15), bms_replay_scenario('B0006', 60, -0.15)};
    else
        [I1, s1] = bms_profile('udds');
        [I2, s2] = bms_profile('cc');
        scs = {struct('I', I1, 'soc0', s1, 'init_err', -0.10, 'seed', 1001, 'fault_mode', 0), ...
               struct('I', I2(1:1800), 'soc0', s2, 'init_err', 0.10, 'seed', 1002, 'fault_mode', 0)};
    end
end

function f = fitness(lp, P, scs, lam)
    Q = (10.^lp(1:3))';  R = 10^lp(4);
    f = 0;
    for i = 1:numel(scs)
        sc = scs{i};  sc.Q = Q;  sc.R = R;  sc.skip_sota = true;
        L = bms_simulate(P, sc);
        e = (L.soc_ekf - L.soc_true) / 100;
        N = numel(e);
        kc = find(abs(e) < 0.03, 1);  if isempty(kc),  kc = N;  end
        f = f + mean(e(60:end).^2) + lam * kc * P.dt / 3600;
    end
    f = f / numel(scs);
end

function write_tuned(variant, T)
    here = fileparts(mfilename('fullpath'));
    old  = bms_tuned_data();
    old.(variant).Q = T.Q;  old.(variant).R = T.R;
    fid = fopen(fullfile(here, 'bms_tuned_data.m'), 'w');
    fprintf(fid, 'function T = bms_tuned_data()\n');
    fprintf(fid, '%%BMS_TUNED_DATA Noise covariances produced by BMS_ACO_TUNE (auto-generated; do not edit by hand).\n');
    for v = {'synthetic', 'nasa'}
        fprintf(fid, '    T.%s.Q = [%.6e; %.6e; %.6e];\n', v{1}, old.(v{1}).Q);
        fprintf(fid, '    T.%s.R = %.6e;\n', v{1}, old.(v{1}).R);
    end
    fprintf(fid, 'end\n');
    fclose(fid);
    clear bms_tuned_data bms_params
end
