function R = bms_sil_run(P, sc, name)
%BMS_SIL_RUN Software-in-the-loop: MATLAB estimator (32-bit float) versus the compiled C firmware.
%   Both receive exactly the same float measurements. The MATLAB estimator is bms_est_step in single precision
%   (estimator mode 0); the other side is c_firmware/bms_ekf.c built by BMS_SIL_BUILD.
%   R fields: max_dsoc [%], max_dv_mV, max_dinnov_mV, max_dnis (absolute), max_dptrace_rel, flags_equal, n, name
    if nargin < 3,  name = 'scenario';  end
    sc.skip_sota = true;  sc.precision = 'single';
    L = bms_simulate(P, sc);                    % the reference, plus the exact measurements the estimator saw
    Ps = bms_cast(P, 'single');
    N = numel(L.soc_ekf);

    soc0 = sc_start(sc);
    guess = single(min(max(soc0 + sc.init_err, 0), 1));
    Q = single(P.Q(:)');  if isfield(sc, 'Q'),  Q = single(sc.Q(:)');  end
    Rv = single(P.R);     if isfield(sc, 'R'),  Rv = single(sc.R);      end
    cfg = single([Ps.R0 Ps.R1 Ps.R2 Ps.C1 Ps.C2 Ps.Qn Ps.eta Ps.dt Ps.Ea Ps.Tref, ...
                  Ps.ocv(:)', Q(1:3), Rv, diag(Ps.P0)', guess, ...
                  Ps.V_valid(1) Ps.V_valid(2) Ps.T_valid(1) Ps.T_valid(2) Ps.I_max Ps.gate_nis Ps.gate_k0 Ps.max_rej ...
                  Ps.nis_alpha Ps.nis_limit]);
    assert(numel(cfg) == 37, 'configuration vector must hold 37 numbers');

    exe  = bms_sil_build('sil');
    tmp  = tempname;  mkdir(tmp);
    fin  = fullfile(tmp, 'in.txt');  fout = fullfile(tmp, 'out.txt');
    fid = fopen(fin, 'w');
    fprintf(fid, '%d\n', N);
    fprintf(fid, '%.9g ', cfg);  fprintf(fid, '\n');
    fprintf(fid, '%.9g %.9g %.9g\n', [L.I_meas; L.V_meas; L.T_meas]);
    fclose(fid);

    [st, out] = system(sprintf('"%s" "%s" "%s"', exe, fin, fout));
    if st ~= 0,  error('bms_sil_run:exe', 'Firmware program failed (exit %d): %s', st, out);  end
    C = readmatrix(fout);   % columns: soc v_est innov nis p_trace fault
    rmdir(tmp, 's');
    assert(size(C, 1) == N, 'firmware returned %d rows, expected %d', size(C, 1), N);

    rel = @(a, b) max(abs(a - b) ./ max(abs(b), 1e-12));
    R.name = name;  R.n = N;
    R.max_dsoc        = max(abs(C(:, 1)' - L.soc_ekf));
    R.max_dv_mV       = max(abs(C(:, 2)' - L.V_est)) * 1e3;
    R.max_dinnov_mV   = max(abs(C(:, 3)' - L.innov)) * 1e3;
    R.max_dnis        = max(abs(C(:, 4)' - L.nis));          % absolute: NIS is ~0 on quiet samples
    R.max_dptrace_rel = rel(C(:, 5)', L.ptrace);
    R.flags_equal     = isequal(C(:, 6)', L.fault_ekf);
    R.n_flagged       = nnz(C(:, 6));
end

function s0 = sc_start(sc)
    if isfield(sc, 'replay') && sc.replay,  s0 = sc.soc_ref(1);  else,  s0 = sc.soc0;  end
end
