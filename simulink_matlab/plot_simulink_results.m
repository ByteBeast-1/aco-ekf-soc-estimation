function plot_simulink_results(out, variant, label)
%PLOT_SIMULINK_RESULTS Plot the logged signals (out.bms_log) of either BMS model.
%   Log columns - synthetic: 1 true SOC | 2 EKF SOC | 3 Coulomb | 4 true V | 5 meas V | 6 est V |
%                            7 innovation | 8 trace(P) | 9 NIS | 10 fault code | 11 T [K]
%                 nasa     : 1 ref SOC | 2 EKF | 3 adaptive EKF | 4 Coulomb | 5 meas V | 6 est V |
%                            7 innovation | 8 NIS | 9 fault code | 10 R0 | 11 R (Sage-Husa) |
%                            12 EKF+capacity SOC | 13 capacity ratio estimate
    if nargin < 2,  variant = 'synthetic';  end
    if nargin < 3,  label = '';  end
    d = out.bms_log;  t = out.tout;
    figure('Name', ['BMS results - ' variant], 'Color', 'w', 'Position', [60 60 1250 800]);
    if strcmp(variant, 'nasa')
        ref = d(:, 1);  e1 = d(:, 2) - ref;  e2 = d(:, 3) - ref;
        subplot(3, 2, 1);  plot(t, ref, 'k', t, d(:, 2), 'b', t, d(:, 3), 'r--', t, d(:, 4), ':', 'LineWidth', 1.3);
        legend('Reference', 'EKF', 'Adaptive EKF', 'Coulomb', 'Location', 'best');  title('SOC [%]');
        subplot(3, 2, 2);  plot(t, e1, 'b', t, e2, 'r--', 'LineWidth', 1.2);  yline(0, 'k:');
        title(sprintf('SOC error [%%] - MAE EKF %.2f | adaptive %.2f', mean(abs(e1(120:end))), mean(abs(e2(120:end)))));
        subplot(3, 2, 3);  plot(t, d(:, 5), 'k', t, d(:, 6), 'r--');  legend('Measured', 'Estimated');  title('Terminal voltage [V]');
        subplot(3, 2, 4);  plot(t, d(:, 8));  title('NIS (expect ~1)');  set(gca, 'YScale', 'log');
        subplot(3, 2, 5);  plot(t, d(:, 13), 'm', 'LineWidth', 1.4);  hold on;
        try,  yline(evalin('base', 'NASA_CapRatio'), 'k--', 'true');  catch,  end
        ylim([0.5 1.05]);
        title(sprintf('Online capacity ratio (SOH) - MAE of EKF+capacity %.2f %%', mean(abs(d(120:end, 12) - ref(120:end)))));
        subplot(3, 2, 6);  semilogy(t, d(:, 11));  title('Sage-Husa R adaptation');
    else
        e1 = d(:, 2) - d(:, 1);
        subplot(3, 2, 1);  plot(t, d(:, 1), 'k', t, d(:, 2), 'b--', t, d(:, 3), ':', 'LineWidth', 1.3);
        legend('True', 'EKF', 'Coulomb', 'Location', 'best');  title('SOC [%]');
        subplot(3, 2, 2);  plot(t, e1, 'b');  yline(0, 'k:');
        title(sprintf('SOC error [%%] - MAE %.3f | max %.2f', mean(abs(e1(120:end))), max(abs(e1(120:end)))));
        subplot(3, 2, 3);  plot(t, d(:, 5), 'c', t, d(:, 4), 'k', t, d(:, 6), 'r--');  legend('Measured', 'True', 'Estimated');  title('Terminal voltage [V]');
        subplot(3, 2, 4);  plot(t, d(:, 9));  title('NIS (expect ~1)');  set(gca, 'YScale', 'log');
        subplot(3, 2, 5);  plot(t, d(:, 10), 'LineWidth', 1.5);  title('Diagnostic code (1 V | 2 T | 4 I | 8 rejected | 16 inconsistent)');
        subplot(3, 2, 6);  plot(t, d(:, 11) - 273.15);  title('Cell temperature [\circ C]');
    end
    for i = 1:6,  subplot(3, 2, i);  grid on;  xlabel('Time (s)');  end
    if ~isempty(label),  sgtitle(label, 'FontWeight', 'bold');  end
end
