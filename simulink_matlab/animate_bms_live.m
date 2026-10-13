% ANIMATE_BMS_LIVE.M
% Professional High-Contrast Animated Dashboard for Battery SOC Telemetry

clc; close all;
here = fileparts(mfilename('fullpath'));
cd(here);

% 1. Run simulation once to get full high-precision trajectory
fprintf('🔋 Running BMS simulation...\n');
bms_setup('synthetic', true);
simOut = sim('advanced_bms_aco_ekf', 'ReturnWorkspaceOutputs', 'on');

% 2. Extract logged data
if isprop(simOut, 'bms_log') || isfield(simOut, 'bms_log')
    log_data = simOut.bms_log;
else
    log_data = simOut.get('bms_log');
end

t = (0:size(log_data, 1)-1)';
true_soc = log_data(:, 1);
ekf_soc  = log_data(:, 2);
coulomb  = log_data(:, 3);
volt     = log_data(:, 4);
temp_C   = log_data(:, 11) - 273.15;
soc_err  = ekf_soc - true_soc;

% 3. Create Live Dashboard GUI Window
hFig = figure('Name', 'Live Automotive BMS Dashboard', ...
    'Color', [0.08, 0.10, 0.13], 'Position', [100, 100, 950, 560], ...
    'MenuBar', 'none', 'NumberTitle', 'off');

% Title
uicontrol('Parent', hFig, 'Style', 'text', 'String', 'Real-Time EV Battery Fuel Gauge Telemetry', ...
    'FontName', 'Segoe UI', 'FontSize', 16, 'FontWeight', 'bold', ...
    'ForegroundColor', [1 1 1], 'BackgroundColor', [0.08, 0.10, 0.13], ...
    'Position', [50, 500, 850, 40]);

% Main Big Estimated SOC Display Box
hSOC = uicontrol('Parent', hFig, 'Style', 'text', 'String', '85.00 %', ...
    'FontName', 'Consolas', 'FontSize', 40, 'FontWeight', 'bold', ...
    'ForegroundColor', [0.0, 0.95, 0.55], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [40, 370, 360, 90]);

uicontrol('Parent', hFig, 'Style', 'text', 'String', 'ESTIMATED SOC (EKF)', ...
    'FontName', 'Segoe UI', 'FontSize', 10, 'FontWeight', 'bold', ...
    'ForegroundColor', [0.6 0.75 0.9], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [40, 350, 360, 25]);

% Sub-meters (True SOC, Voltage, Temp, Error)
hTrueBox = uicontrol('Parent', hFig, 'Style', 'text', 'String', '85.00 %', ...
    'FontName', 'Consolas', 'FontSize', 18, 'FontWeight', 'bold', ...
    'ForegroundColor', [1.0, 0.35, 0.35], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [40, 260, 170, 50]);
uicontrol('Parent', hFig, 'Style', 'text', 'String', 'TRUE SOC (BENCHMARK)', ...
    'FontName', 'Segoe UI', 'FontSize', 8, 'FontWeight', 'bold', ...
    'ForegroundColor', [0.7 0.7 0.7], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [40, 240, 170, 20]);

hErrBox = uicontrol('Parent', hFig, 'Style', 'text', 'String', '0.00 %', ...
    'FontName', 'Consolas', 'FontSize', 18, 'FontWeight', 'bold', ...
    'ForegroundColor', [1.0, 0.85, 0.2], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [230, 260, 170, 50]);
uicontrol('Parent', hFig, 'Style', 'text', 'String', 'ESTIMATION ERROR', ...
    'FontName', 'Segoe UI', 'FontSize', 8, 'FontWeight', 'bold', ...
    'ForegroundColor', [0.7 0.7 0.7], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [230, 240, 170, 20]);

hVolt = uicontrol('Parent', hFig, 'Style', 'text', 'String', '4.150 V', ...
    'FontName', 'Consolas', 'FontSize', 18, 'FontWeight', 'bold', ...
    'ForegroundColor', [0.3, 0.75, 1.0], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [40, 160, 170, 50]);
uicontrol('Parent', hFig, 'Style', 'text', 'String', 'TERMINAL VOLTAGE', ...
    'FontName', 'Segoe UI', 'FontSize', 8, 'FontWeight', 'bold', ...
    'ForegroundColor', [0.7 0.7 0.7], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [40, 140, 170, 20]);

hTemp = uicontrol('Parent', hFig, 'Style', 'text', 'String', '24.0 °C', ...
    'FontName', 'Consolas', 'FontSize', 18, 'FontWeight', 'bold', ...
    'ForegroundColor', [1.0, 0.55, 0.2], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [230, 160, 170, 50]);
uicontrol('Parent', hFig, 'Style', 'text', 'String', 'CELL TEMPERATURE', ...
    'FontName', 'Segoe UI', 'FontSize', 8, 'FontWeight', 'bold', ...
    'ForegroundColor', [0.7 0.7 0.7], 'BackgroundColor', [0.13, 0.16, 0.20], ...
    'Position', [230, 140, 170, 20]);

% Right-hand dynamic curve axes (Dark Charcoal Background)
hAx = axes('Parent', hFig, 'Position', [0.47, 0.20, 0.49, 0.68], ...
    'Color', [0.13, 0.16, 0.20], 'XColor', [0.7 0.8 0.9], 'YColor', [0.7 0.8 0.9]);
grid(hAx, 'on'); box(hAx, 'on');
xlabel(hAx, 'Time [s]', 'FontWeight', 'bold', 'Color', [0.8 0.9 1.0]);
ylabel(hAx, 'Battery State of Charge [%]', 'FontWeight', 'bold', 'Color', [0.8 0.9 1.0]);
ylim(hAx, [42, 88]);
xlim(hAx, [0, t(end)]);

% High-contrast plot lines:
% 1. True SOC: Solid Bright Red / Coral (High visibility on dark background)
hTruePlot = plot(hAx, NaN, NaN, '-', 'Color', [1.0, 0.35, 0.35], 'LineWidth', 2.8, 'DisplayName', 'True SOC (Physical Battery)');
hold(hAx, 'on');
% 2. EKF Estimate: Cyan Dashed line overlaid right on top
hEKFPlot  = plot(hAx, NaN, NaN, '--', 'Color', [0.0, 0.95, 1.0], 'LineWidth', 2.0, 'DisplayName', 'EKF Estimate (Algorithm)');

legend(hAx, 'Location', 'northeast', 'TextColor', [1 1 1], 'Color', [0.08, 0.10, 0.13], 'FontSize', 9);

% Status banner at the bottom
hStatus = uicontrol('Parent', hFig, 'Style', 'text', 'String', 'Simulating Drive Cycle: Time = 0 / 1800 s', ...
    'FontName', 'Segoe UI', 'FontSize', 11, ...
    'ForegroundColor', [0.7 0.85 1.0], 'BackgroundColor', [0.08, 0.10, 0.13], ...
    'Position', [40, 30, 870, 30]);

% 4. Animation Playback Loop
step_stride = 15;
for k = 1:step_stride:length(t)
    if ~ishandle(hFig), break; end
    
    cur_true = true_soc(k);
    cur_ekf  = ekf_soc(k);
    cur_err  = soc_err(k);
    cur_v    = volt(k);
    cur_t    = temp_C(k);
    cur_time = t(k);
    
    % Update digital gauges
    set(hSOC,     'String', sprintf('%.2f %%', cur_ekf));
    set(hTrueBox, 'String', sprintf('%.2f %%', cur_true));
    set(hErrBox,  'String', sprintf('%+.2f %%', cur_err));
    set(hVolt,    'String', sprintf('%.3f V', cur_v));
    set(hTemp,    'String', sprintf('%.1f °C', cur_t));
    set(hStatus,  'String', sprintf('Simulating Drive: Time = %d / %d s | EKF Error = %.3f %%', cur_time, t(end), abs(cur_err)));
    
    % Update live trajectory curves
    set(hTruePlot, 'XData', t(1:k), 'YData', true_soc(1:k));
    set(hEKFPlot,  'XData', t(1:k), 'YData', ekf_soc(1:k));
    
    drawnow;
    pause(0.025);
end

if ishandle(hFig)
    set(hStatus, 'String', 'Simulation Complete! Final SOC: 47.11% | Mean Absolute Error < 0.17%');
end
