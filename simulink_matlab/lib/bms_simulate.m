function L = bms_simulate(P, sc)
%BMS_SIMULATE Pure-MATLAB closed-loop run (plant -> sensors -> estimators). Used for ACO tuning,
%   Monte Carlo, regression tests and back-to-back comparison against the Simulink models.
%
%   sc fields (synthetic): I, soc0, init_err, seed, fault_mode, Q, R
%   sc fields (replay)   : I, V, T, soc_ref, init_err, Q, R       (sc.replay = true)
%   sc.precision = 'single' runs the ESTIMATOR in 32-bit float (plant and sensors stay double), which is
%   the arithmetic of the embedded firmware; the logged measurements are then the float values it saw.
    N = numel(sc.I);
    if ~isfield(sc, 'replay'),      sc.replay = false;  end
    if ~isfield(sc, 'fault_mode'),  sc.fault_mode = 0;  end
    if ~isfield(sc, 'seed'),        sc.seed = 1;        end
    if ~isfield(sc, 'skip_sota'),   sc.skip_sota = false;  end
    if ~isfield(sc, 'Q'),           sc.Q = P.Q;         end
    if ~isfield(sc, 'R'),           sc.R = P.R;         end

    if sc.replay
        soc0 = sc.soc_ref(1);
    else
        soc0 = sc.soc0;
    end
    guess = min(max(soc0 + sc.init_err, 0), 1);

    prec = 'double';
    if isfield(sc, 'precision'),  prec = sc.precision;  end
    Pe = P;
    if strcmp(prec, 'single'),  Pe = bms_cast(P, 'single');  end
    cv = @(x) cast(x, prec);

    ekf  = bms_est_init(Pe, cv(guess), cv(sc.Q), cv(sc.R), 0);
    sota = bms_est_init(Pe, cv(guess), cv(sc.Q), cv(sc.R), P.sota_mode);
    x  = [soc0; 0; 0];
    ss = bms_seed(sc.seed);

    f = {'soc_true','soc_ekf','soc_sota','soc_cc','V_true','V_meas','V_est','innov','ptrace', ...
         'nis','fault_ekf','fault_sota','r0_sota','R_sota','I_meas','T_meas','cap_sota'};
    for i = 1:numel(f),  L.(f{i}) = zeros(1, N);  end

    for k = 1:N
        if sc.replay
            Vm = sc.V(k);  Im = sc.I(k);  Tm = sc.T(k);
            L.soc_true(k) = 100 * sc.soc_ref(k);  L.V_true(k) = Vm;
        else
            T = bms_temp_profile(k);
            [x, Vt] = bms_plant_step(x, sc.I(k), T, P);
            [Vm, Im, Tm, ss] = bms_sensor_step(Vt, sc.I(k), T, k, ss, P, sc.fault_mode);
            L.soc_true(k) = 100 * x(1);  L.V_true(k) = Vt;
        end
        Im = cv(Im);  Vm = cv(Vm);  Tm = cv(Tm);          % what the estimator actually sees
        [ekf,  oe] = bms_est_step(ekf,  Im, Vm, Tm, Pe, 0);
        if sc.skip_sota
            os = oe;                       % tuning runs only need the EKF
        else
            [sota, os] = bms_est_step(sota, Im, Vm, Tm, Pe, P.sota_mode);
        end
        L.soc_ekf(k) = oe.soc;   L.soc_sota(k) = os.soc;   L.soc_cc(k) = oe.soc_cc;
        L.V_meas(k) = Vm;        L.V_est(k) = oe.vest;     L.innov(k) = oe.innov;
        L.ptrace(k) = oe.ptrace; L.nis(k) = oe.nis;
        L.fault_ekf(k) = oe.fault;  L.fault_sota(k) = os.fault;
        L.r0_sota(k) = os.r0;       L.R_sota(k) = sota.R;
        L.I_meas(k) = Im;           L.T_meas(k) = Tm;        L.cap_sota(k) = os.cap;
    end
end
