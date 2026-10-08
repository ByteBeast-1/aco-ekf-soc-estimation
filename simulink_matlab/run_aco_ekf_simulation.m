function run_aco_ekf_simulation()
% RUN_ACO_EKF_SIMULATION
% 2-RC Thermal MO-ACO-EKF Battery SOC Estimation — MATLAB Standalone Runner
% Generates 6 publication-grade figures for faculty presentation.
    clc; close all;

    fprintf('=================================================================\n');
    fprintf('  2-RC Thermal MO-ACO-EKF | Battery SOC Estimation\n');
    fprintf('=================================================================\n\n');

    %% Configuration
    cfg = get_config();

    %% Step 1: Generate 4 drive profiles
    fprintf('[1/4] Generating drive profiles...\n');
    profiles = generate_profiles(cfg);

    %% Step 2: Run MO-ACOR optimizer
    fprintf('[2/4] Running MO-ACOR optimizer (%d ants, %d iterations)...\n', ...
        cfg.n_ants, cfg.n_iter);
    [Q_opt, R_opt, aco_hist] = run_mo_acor(profiles{3}, cfg);
    fprintf('      Q* = %.3e  |  R* = %.4e\n', Q_opt(1,1), R_opt);

    %% Step 3: Run EKF on all 4 profiles
    fprintf('[3/4] Running 3-State Joseph-Form EKF on 4 profiles...\n');
    results = cell(4,1);
    for p = 1:4
        results{p} = run_ekf(profiles{p}, Q_opt, R_opt, cfg);
        fprintf('      [%s]  MAE=%.4f%%  RMSE=%.4f%%\n', ...
            profiles{p}.name, results{p}.mae, results{p}.rmse);
    end

    %% Step 4: Plot 6 professional figures
    fprintf('[4/4] Plotting figures...\n\n');
    plot_figures(profiles, results, aco_hist, Q_opt, R_opt);

    fprintf('=================================================================\n');
    fprintf('  Complete! Q*=%.2e | R*=%.4e\n', Q_opt(1,1), R_opt);
    fprintf('=================================================================\n');
end

%% ====================================================================
function cfg = get_config()
    cfg.R0  = 0.012; cfg.R1  = 0.008; cfg.R2  = 0.015;
    cfg.C1  = 1500;  cfg.C2  = 12000;
    cfg.Qn  = 7200;  cfg.dt  = 1.0;
    cfg.Ea  = 18000; cfg.T0  = 298.15;
    cfg.sv  = 0.008; % voltage sensor noise std [V]
    cfg.ocv = [-21.7497, 82.2625, -125.3296, 99.6313, ...
               -44.5010,  11.3416,  -1.5825,  1.5032, 3.0130];
    cfg.P0      = diag([0.01, 0.0025, 0.0025]);
    cfg.n_ants  = 20;
    cfg.n_iter  = 30;
    cfg.K       = 20;
    cfg.q_aco   = 0.1;
    cfg.xi      = 0.85;
    cfg.bounds  = [-8,-5; -8,-5; -8,-5; -5,-2];  % [4 x 2] log10 bounds
end

%% ====================================================================
function profiles = generate_profiles(cfg)
    dt = cfg.dt;

    % Profile 1: Constant Current — Highway Cruise 0.5C
    N1 = 3600; t1 = (0:N1-1)*dt;
    I1 = ones(1,N1);
    T1 = linspace(297.15, 310.15, N1);
    [Vt1, S1] = sim_plant(I1, T1, 0.95, cfg);
    profiles{1} = make_prof('CC Highway', t1, I1, Vt1 + cfg.sv*randn(1,N1), Vt1, S1, T1, 0.95);

    % Profile 2: Pulse Discharge — Stop-Start City
    N2 = 2400; t2 = (0:N2-1)*dt;
    I2 = zeros(1,N2);
    for k=1:N2; if mod(t2(k),90)<60; I2(k)=2.0; end; end
    T2 = linspace(297.15, 306.15, N2);
    [Vt2, S2] = sim_plant(I2, T2, 0.90, cfg);
    profiles{2} = make_prof('Pulse City', t2, I2, Vt2 + cfg.sv*randn(1,N2), Vt2, S2, T2, 0.90);

    % Profile 3: Dynamic UDDS — Urban Drive Cycle
    N3 = 1800; t3 = (0:N3-1)*dt;
    I3 = abs(1.0 + 0.8*sin(2*pi*t3/200) + 1.2*(sin(2*pi*t3/35)>0.3));
    I3 = min(max(I3,0), 3.5);
    T3 = linspace(297.15, 312.15, N3);
    [Vt3, S3] = sim_plant(I3, T3, 0.85, cfg);
    profiles{3} = make_prof('UDDS Dynamic', t3, I3, Vt3 + 0.010*randn(1,N3), Vt3, S3, T3, 0.85);

    % Profile 4: CCCV Fast Charge
    N4 = 3600; t4 = (0:N4-1)*dt;
    I4 = zeros(1,N4);
    for k=1:N4
        s = 0.20 + k/(N4*1.2);
        if s < 0.80; I4(k) = -2.0; else; I4(k) = -(2.0*(1-s)/0.20); end
    end
    T4 = 297.15*ones(1,N4);
    [Vt4, S4] = sim_plant(I4, T4, 0.20, cfg);
    profiles{4} = make_prof('CCCV Charge', t4, I4, Vt4 + 0.006*randn(1,N4), Vt4, S4, T4, 0.20);
end

function p = make_prof(name, t, I, Vm, Vt, S, T, soc0)
    p.name = name; p.t = t; p.I = I;
    p.V_meas = Vm; p.V_true = Vt;
    p.SOC_ref = S; p.Temp = T; p.soc0 = soc0;
end

%% ====================================================================
function [Vt, S] = sim_plant(I, T, soc0, cfg)
    N = length(I); Vt = zeros(1,N); S = zeros(1,N);
    x = [soc0; 0; 0];
    for k = 1:N
        S(k)  = x(1);
        Vt(k) = vt_eq(x, I(k), T(k), cfg);
        x     = step_x(x, I(k), T(k), cfg);
    end
end

function x = step_x(x, I, Tk, cfg)
    [~,R1,R2] = Rarr(Tk,cfg);
    a1=exp(-cfg.dt/(R1*cfg.C1)); b1=R1*(1-a1);
    a2=exp(-cfg.dt/(R2*cfg.C2)); b2=R2*(1-a2);
    x = [min(max(x(1)-(I*cfg.dt)/cfg.Qn,0),1); a1*x(2)+b1*I; a2*x(3)+b2*I];
end

function v = vt_eq(x, I, Tk, cfg)
    [R0,~,~] = Rarr(Tk,cfg);
    v = polyval(cfg.ocv, min(max(x(1),0),1)) - x(2) - x(3) - R0*I;
end

function [R0,R1,R2] = Rarr(Tk, cfg)
    tf = exp((cfg.Ea/8.314)*(1/Tk - 1/cfg.T0));
    R0=cfg.R0*tf; R1=cfg.R1*tf; R2=cfg.R2*tf;
end

%% ====================================================================
function [Q_opt, R_opt, hist] = run_mo_acor(prof, cfg)
    K=cfg.K; n=cfg.n_ants; nd=4; lam=0.5;
    bnd=cfg.bounds;
    arc = bnd(:,1)' + rand(K,nd) .* (bnd(:,2)-bnd(:,1))';
    afit = arrayfun(@(i) fitness(arc(i,:),prof,cfg,lam), 1:K)';
    [afit,ix] = sort(afit); arc = arc(ix,:);
    hist = zeros(1,cfg.n_iter);
    for it = 1:cfg.n_iter
        rk = (1:K)';
        w  = exp(-((rk-1).^2)/(2*(cfg.q_aco*K)^2));
        w  = w/sum(w); cp = cumsum(w);
        ns = zeros(n,nd); nf = zeros(n,1);
        for a = 1:n
            ki = find(cp>=rand(),1); if isempty(ki); ki=K; end
            s  = zeros(1,nd);
            for d=1:nd
                sg = cfg.xi*sum(abs(arc(:,d)-arc(ki,d)))/(K-1+eps);
                s(d) = max(bnd(d,1), min(bnd(d,2), arc(ki,d)+sg*randn()));
            end
            ns(a,:) = s; nf(a) = fitness(s,prof,cfg,lam);
        end
        all_s = [arc;ns]; all_f = [afit;nf];
        [all_f,ix] = sort(all_f);
        arc = all_s(ix(1:K),:); afit = all_f(1:K);
        hist(it) = afit(1);
    end
    b=arc(1,:);
    Q_opt = diag([10^b(1), 10^b(2), 10^b(3)]);
    R_opt = 10^b(4);
end

function f = fitness(lp, prof, cfg, lam)
    Q=diag([10^lp(1),10^lp(2),10^lp(3)]); R=10^lp(4);
    N=length(prof.t); x=[prof.soc0-0.10;0;0]; P=cfg.P0;
    se=zeros(1,N); conv_k=N;
    for k=1:N
        [~,R1,R2]=Rarr(prof.Temp(k),cfg);
        a1=exp(-cfg.dt/(R1*cfg.C1)); a2=exp(-cfg.dt/(R2*cfg.C2));
        F=diag([1,a1,a2]);
        xp=step_x(x,prof.I(k),prof.Temp(k),cfg);
        Pp=F*P*F'+Q;
        vp=vt_eq(xp,prof.I(k),prof.Temp(k),cfg);
        dp=polyder(cfg.ocv); H=[polyval(dp,min(max(xp(1),0),1)),-1,-1];
        Sk=H*Pp*H'+R; Kk=(Pp*H')/Sk;
        xu=xp+Kk*(prof.V_meas(k)-vp); xu(1)=min(max(xu(1),0),1);
        P=(eye(3)-Kk*H)*Pp*(eye(3)-Kk*H)'+Kk*R*Kk'; x=xu;
        se(k)=x(1);
        if abs(se(k)-prof.SOC_ref(k))<0.02 && conv_k==N; conv_k=k; end
    end
    f = mean((se-prof.SOC_ref).^2) + lam*(conv_k*cfg.dt/3600);
end

%% ====================================================================
function res = run_ekf(prof, Q_opt, R_opt, cfg)
    N=length(prof.t);
    se=zeros(1,N); eV=zeros(1,N); ie=zeros(1,N); pt=zeros(1,N);
    x=[prof.soc0-0.10;0;0]; P=cfg.P0;
    for k=1:N
        [~,R1,R2]=Rarr(prof.Temp(k),cfg);
        a1=exp(-cfg.dt/(R1*cfg.C1)); a2=exp(-cfg.dt/(R2*cfg.C2));
        F=diag([1,a1,a2]);
        xp=step_x(x,prof.I(k),prof.Temp(k),cfg);
        Pp=F*P*F'+Q_opt;
        vp=vt_eq(xp,prof.I(k),prof.Temp(k),cfg);
        ie(k)=prof.V_meas(k)-vp; eV(k)=vp;
        dp=polyder(cfg.ocv); H=[polyval(dp,min(max(xp(1),0),1)),-1,-1];
        Sk=H*Pp*H'+R_opt; Kk=(Pp*H')/Sk;
        xu=xp+Kk*ie(k); xu(1)=min(max(xu(1),0),1);
        P=(eye(3)-Kk*H)*Pp*(eye(3)-Kk*H)'+Kk*R_opt*Kk'; x=xu;
        se(k)=x(1); pt(k)=trace(P);
    end
    err = (se - prof.SOC_ref)*100;
    res.se  = se; res.eV = eV; res.ie = ie; res.pt = pt;
    res.mae = mean(abs(err)); res.rmse = sqrt(mean(err.^2)); res.maxe = max(abs(err));
end

%% ====================================================================
%  PUBLICATION-GRADE FIGURES
%% ====================================================================
function plot_figures(profiles, results, aco_hist, Q_opt, R_opt)

    % Color palette — consistent across all figures
    C_true  = [0.15, 0.45, 0.80];   % Blue
    C_est   = [0.90, 0.25, 0.25];   % Red
    C_noisy = [0.60, 0.80, 0.95];   % Light blue
    C_err   = [0.20, 0.65, 0.35];   % Green
    C_aco   = [0.70, 0.20, 0.80];   % Purple

    set(0, 'DefaultAxesFontName',  'Helvetica');
    set(0, 'DefaultTextFontName',  'Helvetica');
    set(0, 'DefaultAxesFontSize',  11);
    set(0, 'DefaultLineLineWidth', 1.6);

    %% ----------------------------------------------------------------
    %  FIGURE 1 — SOC Tracking: True vs Estimated (4 profiles)
    %% ----------------------------------------------------------------
    fig1 = figure('Name','SOC Tracking','NumberTitle','off', ...
        'Position',[60,60,1380,780], 'Color','w');
    sgtitle(sprintf('SOC Tracking — 2-RC Thermal MO-ACO-EKF    |    Q* = %.1e   R* = %.4e', ...
        Q_opt(1,1), R_opt), 'FontSize', 14, 'FontWeight', 'bold');

    profile_titles = {'CC Discharge — Highway Cruise 0.5C', ...
                      'Pulse Discharge — Stop-Start City', ...
                      'UDDS — Dynamic Urban Drive Cycle', ...
                      'CCCV — Fast Charging Profile'};

    for p = 1:4
        ax = subplot(2,2,p);
        t  = profiles{p}.t;
        sr = profiles{p}.SOC_ref * 100;
        se = results{p}.se * 100;
        plot(t, sr, '-',  'Color', C_true, 'LineWidth', 2.0, 'DisplayName', 'True SOC'); hold on;
        plot(t, se, '--', 'Color', C_est,  'LineWidth', 1.6, 'DisplayName', 'EKF Estimated SOC');
        xlabel('Time (s)'); ylabel('SOC (%)');
        title(sprintf('%s\nMAE = %.4f%%  |  RMSE = %.4f%%', ...
            profile_titles{p}, results{p}.mae, results{p}.rmse), 'FontSize', 10.5);
        legend('Location','best','FontSize',9.5);
        grid on; box on;
        ylim([max(0,min(sr)-4), min(100,max(sr)+4)]);
        set(ax,'GridAlpha',0.25,'GridLineStyle','--');
    end

    %% ----------------------------------------------------------------
    %  FIGURE 2 — SOC Error Over Time (4 profiles)
    %% ----------------------------------------------------------------
    figure('Name','SOC Error','NumberTitle','off', ...
        'Position',[80,80,1380,780], 'Color','w');
    sgtitle('SOC Estimation Error  (EKF − True SOC)', ...
        'FontSize',14,'FontWeight','bold');

    for p = 1:4
        ax = subplot(2,2,p);
        t   = profiles{p}.t;
        err = (results{p}.se - profiles{p}.SOC_ref) * 100;
        fill([t, fliplr(t)], [err, zeros(1,length(t))], C_err, ...
            'FaceAlpha', 0.25, 'EdgeColor', 'none'); hold on;
        plot(t, err, '-', 'Color', C_err*0.7, 'LineWidth', 1.4);
        yline(0,   'k--', 'LineWidth', 1.0);
        yline(+0.5,'r:',  'LineWidth', 0.9);
        yline(-0.5,'r:',  'LineWidth', 0.9);
        xlabel('Time (s)'); ylabel('Error (%)');
        title(sprintf('%s\nRMSE = %.4f%%   MaxError = %.4f%%', ...
            profile_titles{p}, results{p}.rmse, results{p}.maxe), 'FontSize', 10.5);
        ylim([-1.5, 1.5]); grid on; box on;
        set(ax,'GridAlpha',0.25,'GridLineStyle','--');
    end

    %% ----------------------------------------------------------------
    %  FIGURE 3 — Voltage Tracking (Profile 3: UDDS)
    %% ----------------------------------------------------------------
    p = 3;
    figure('Name','Voltage Tracking','NumberTitle','off', ...
        'Position',[100,100,1200,500], 'Color','w');
    t = profiles{p}.t;
    plot(t, profiles{p}.V_meas,  '-', 'Color', C_noisy, 'LineWidth', 0.8, ...
        'DisplayName', 'V_{meas}  (±8 mV noise)'); hold on;
    plot(t, profiles{p}.V_true,  '-', 'Color', C_true,  'LineWidth', 2.0, ...
        'DisplayName', 'True Voltage V_t');
    plot(t, results{p}.eV,       '--','Color', C_est,   'LineWidth', 1.6, ...
        'DisplayName', 'EKF Estimated Voltage');
    xlabel('Time (s)'); ylabel('Terminal Voltage (V)');
    title(sprintf('Voltage Tracking — Profile: %s  |  Sensor Noise ±8 mV  |  EKF vs True', ...
        profiles{p}.name), 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location','best','FontSize',10); grid on; box on;
    set(gca,'GridAlpha',0.25,'GridLineStyle','--');

    %% ----------------------------------------------------------------
    %  FIGURE 4 — Drive Profile + Thermal Transient (Profile 3)
    %% ----------------------------------------------------------------
    p = 3;
    figure('Name','Drive Profile + Temperature','NumberTitle','off', ...
        'Position',[120,120,1200,580], 'Color','w');

    subplot(2,1,1);
    t = profiles{p}.t;
    area(t, profiles{p}.I, 'FaceColor', C_true, 'FaceAlpha', 0.4, 'EdgeColor', C_true);
    xlabel('Time (s)'); ylabel('Current  I  (A)');
    title(sprintf('Drive Cycle: %s — NET CURRENT (I)', profiles{p}.name), 'FontSize',11,'FontWeight','bold');
    grid on; box on; set(gca,'GridAlpha',0.25,'GridLineStyle','--');

    subplot(2,1,2);
    T_C = profiles{p}.Temp - 273.15;
    plot(t, T_C, '-', 'Color', [0.85, 0.35, 0.10], 'LineWidth', 2.0);
    xlabel('Time (s)'); ylabel('Temperature  T  (°C)');
    title(sprintf('Cell Self-Heating: %.1f °C  →  %.1f °C  (Arrhenius Thermal Model)', T_C(1), T_C(end)), ...
        'FontSize',11,'FontWeight','bold');
    grid on; box on; set(gca,'GridAlpha',0.25,'GridLineStyle','--');
    sgtitle('Thermal Operating Conditions', 'FontSize',13,'FontWeight','bold');

    %% ----------------------------------------------------------------
    %  FIGURE 5 — MO-ACOR Convergence Curve
    %% ----------------------------------------------------------------
    figure('Name','MO-ACOR Convergence','NumberTitle','off', ...
        'Position',[140,140,900,470], 'Color','w');
    iters = 1:length(aco_hist);
    semilogy(iters, aco_hist, 'o-', 'Color', C_aco, 'LineWidth', 2.0, ...
        'MarkerSize', 5, 'MarkerFaceColor', C_aco);
    xlabel('ACO Iteration'); ylabel('Multi-Objective Fitness  (log scale)');
    title(sprintf('MO-ACOR Swarm Convergence  |  %d Ants  |  %d Iterations\nConverged: Q* = %.2e   R* = %.4e', ...
        20, length(aco_hist), Q_opt(1,1), R_opt), 'FontSize', 12, 'FontWeight', 'bold');
    grid on; box on;
    set(gca,'GridAlpha',0.25,'GridLineStyle','--');
    text(length(aco_hist)*0.55, aco_hist(end)*4, ...
        sprintf('\\bfOptimal Q*\\rm = %.2e\n\\bfOptimal R*\\rm = %.4e', Q_opt(1,1), R_opt), ...
        'FontSize', 10.5, 'BackgroundColor', [0.96,0.96,1.0], ...
        'EdgeColor', [0.7,0.7,0.8], 'Margin', 6);

    %% ----------------------------------------------------------------
    %  FIGURE 6 — Benchmark Summary: MAE + RMSE Bar Chart
    %% ----------------------------------------------------------------
    figure('Name','Benchmark Summary','NumberTitle','off', ...
        'Position',[160,160,1000,520], 'Color','w');

    maes  = [results{1}.mae,  results{2}.mae,  results{3}.mae,  results{4}.mae];
    rmses = [results{1}.rmse, results{2}.rmse, results{3}.rmse, results{4}.rmse];
    xlbls = {'CC Highway', 'Pulse City', 'UDDS Dynamic', 'CCCV Charge'};

    b = bar([maes; rmses]', 'grouped');
    b(1).FaceColor = C_true; b(1).EdgeColor = 'none';
    b(2).FaceColor = C_est;  b(2).EdgeColor = 'none';

    set(gca, 'XTickLabel', xlbls, 'FontSize', 11);
    ylabel('Error  (%)');
    title(sprintf('Benchmark: MAE & RMSE Across 4 Drive Profiles\n2-RC Thermal MO-ACO-EKF  |  Q* = %.2e   R* = %.4e', ...
        Q_opt(1,1), R_opt), 'FontSize', 12, 'FontWeight', 'bold');
    legend('MAE (%)', 'RMSE (%)', 'Location', 'northeast', 'FontSize', 10.5);
    grid on; box on;
    set(gca, 'GridAlpha', 0.25, 'GridLineStyle', '--', 'YGrid', 'on', 'XGrid', 'off');

    % Value labels
    for i = 1:4
        text(i - 0.15, maes(i)  + max(maes)*0.04,  sprintf('%.4f', maes(i)),  ...
            'HorizontalAlignment','center','FontSize',9,'FontWeight','bold','Color',C_true*0.7);
        text(i + 0.15, rmses(i) + max(rmses)*0.04, sprintf('%.4f', rmses(i)), ...
            'HorizontalAlignment','center','FontSize',9,'Color',C_est*0.7);
    end

    % Print summary table
    fprintf('\n  %-22s  %8s  %8s  %8s\n', 'Profile', 'MAE %', 'RMSE %', 'MaxErr %');
    fprintf('  %s\n', repmat('-', 52, 1));
    for p = 1:4
        fprintf('  %-22s  %8.4f  %8.4f  %8.4f\n', ...
            profiles{p}.name, results{p}.mae, results{p}.rmse, results{p}.maxe);
    end
    fprintf('\n');
end
