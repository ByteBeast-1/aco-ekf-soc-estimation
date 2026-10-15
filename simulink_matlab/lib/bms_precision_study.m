function R = bms_precision_study(verbose)
%BMS_PRECISION_STUDY How much does running the EKF in 32-bit float (the firmware's arithmetic) change it?
%   Every scenario is run twice on identical measurements: estimator in double, then in single.
%   Reported per scenario (SOC in percent, voltage in millivolt):
%     max_dsoc / rms_dsoc : worst and RMS |SOC_single - SOC_double|
%     max_dv_mV           : worst |V_est_single - V_est_double|
%     mae_d / mae_s       : mean absolute error against the reference, double and single
%     flags_equal         : diagnostic codes identical sample by sample
%     finite              : no NaN or Inf anywhere in the single-precision run
%   Returns a struct with table T and the worst case over all scenarios.
    if nargin < 1,  verbose = true;  end
    skip = 120;
    rows = {};

    % ---- simulated cell (model-matched plant, ACO-tuned Q and R) ---------------
    P = bms_params('synthetic');
    profs = {'udds', 'pulse', 'cc', 'cccv'};
    for i = 1:numel(profs)
        [I, soc0] = bms_profile(profs{i});  I = I(1:min(numel(I), 1800));
        for fm = [0 1 2 3]
            if fm > 0 && ~strcmp(profs{i}, 'udds'),  continue;  end
            sc = struct('I', I, 'soc0', soc0, 'init_err', -0.10, 'seed', 40 + i, 'fault_mode', fm);
            rows(end + 1, :) = compare(P, sc, sprintf('sim %s fault %d', profs{i}, fm), skip); %#ok<AGROW>
        end
    end

    % ---- real NASA cycles (calibrated cell) ------------------------------------------
    Pn = bms_params('nasa');
    real = {{'B0005',1,'discharge'},{'B0005',60,'discharge'},{'B0005',120,'discharge'}, ...
            {'B0007',1,'discharge'},{'B0007',60,'discharge'},{'B0007',120,'discharge'}, ...
            {'B0006',120,'discharge'},{'B0005',1,'charge'},{'B0007',1,'charge'}};
    for i = 1:numel(real)
        r = real{i};
        ie = -0.15;  if strcmp(r{3}, 'charge'),  ie = 0.15;  end
        sc = bms_replay_scenario(r{1}, r{2}, ie, r{3});
        rows(end + 1, :) = compare(Pn, sc, sc.name, skip); %#ok<AGROW>
    end

    T = cell2table(rows, 'VariableNames', {'scenario', 'n', 'max_dsoc', 'rms_dsoc', 'max_dv_mV', 'mae_d', 'mae_s', 'flags_equal', 'finite'});
    R.T = T;
    R.worst_dsoc = max(T.max_dsoc);
    R.worst_dv_mV = max(T.max_dv_mV);
    R.max_mae_shift = max(abs(T.mae_s - T.mae_d));
    R.all_flags_equal = all(T.flags_equal);
    R.all_finite = all(T.finite);

    if verbose
        fprintf('  %-26s %6s %10s %10s %10s %8s %8s %6s\n', 'scenario', 'N', 'max dSOC%', 'rms dSOC%', 'max dV mV', 'MAE dbl', 'MAE sgl', 'flags=');
        for i = 1:height(T)
            fprintf('  %-26s %6d %10.5f %10.5f %10.4f %8.3f %8.3f %6d\n', T.scenario{i}, T.n(i), T.max_dsoc(i), T.rms_dsoc(i), ...
                T.max_dv_mV(i), T.mae_d(i), T.mae_s(i), T.flags_equal(i));
        end
        fprintf('  worst |dSOC| = %.5f %% | worst |dV| = %.4f mV | largest MAE shift = %.4f pp | flags identical: %d | finite: %d\n', ...
            R.worst_dsoc, R.worst_dv_mV, R.max_mae_shift, R.all_flags_equal, R.all_finite);
    end
end

function row = compare(P, sc, name, skip)
    sc.skip_sota = true;
    sc.precision = 'double';  Ld = bms_simulate(P, sc);
    sc.precision = 'single';  Ls = bms_simulate(P, sc);
    d  = abs(Ls.soc_ekf - Ld.soc_ekf);
    dv = abs(Ls.V_est - Ld.V_est) * 1e3;
    md = bms_metrics(Ld.soc_ekf, Ld.soc_true, skip);
    ms = bms_metrics(Ls.soc_ekf, Ls.soc_true, skip);
    fin = all(isfinite(Ls.soc_ekf)) && all(isfinite(Ls.ptrace)) && all(isfinite(Ls.V_est));
    flags_eq = (nnz(Ls.fault_ekf ~= Ld.fault_ekf) <= 1);
    row = {name, numel(d), max(d), sqrt(mean(d.^2)), max(dv), md.mae, ms.mae, flags_eq, fin};
end
