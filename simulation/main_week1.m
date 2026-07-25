% MAIN_WEEK1  Week 1 deliverable: build and validate the R-2RC battery model.
%
%   Runs all four discharge profiles from the base paper (Fig. 7) through
%   the R-2RC plant model, and plots SoC and terminal voltage over time
%   for each -- so you can visually sanity-check the model behaves like
%   a real battery before Week 2 builds the EKF on top of it.
%
%   HOW TO RUN
%       In MATLAB: open this folder, then run main_week1
%       In Octave: octave --no-gui main_week1.m

clear; clc; close all;

% --- Make sibling project folders visible to MATLAB, regardless of
%     which folder you launched this script from ---
this_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(this_dir, '..', 'models'));

Qn_Ah = 12;      % nominal capacity, Ah (Table III)
SoC0  = 1.0;     % start fully charged
dt    = 1;       % 1 second sampling, matches Table IV

profiles = {
    'constant',   4500, '7.2 A constant discharge';
    'pulse',      6000, '6.5 A pulse (25 s ON / 5 s OFF)';
    'dual_pulse', 6000, '6.25 A / 1.7 A dual-level pulse';
    'motor',      6000, '5 A DC motor load (25 s ON / 5 s OFF)';
};

figure('Position', [100, 100, 1000, 800]);

for i = 1:size(profiles, 1)
    name     = profiles{i, 1};
    duration = profiles{i, 2};
    label    = profiles{i, 3};

    [t, I] = generate_profile(name, dt, duration);
    out = simulate_battery(t, I, SoC0, Qn_Ah, false);

    fprintf('%-12s | final SoC = %.3f | final VT = %.3f V\n', ...
            name, out.SoC_true(end), out.VT_true(end));

    subplot(2, 2, i);
    yyaxis left
    plot(out.t, out.SoC_true, 'LineWidth', 1.5);
    ylabel('SoC');
    ylim([0, 1.05]);

    yyaxis right
    plot(out.t, out.VT_true, 'LineWidth', 1.2);
    ylabel('Terminal Voltage (V)');

    title(label, 'Interpreter', 'none');
    xlabel('Time (s)');
    grid on;
end

sgtitle('Week 1: R-2RC Battery Model -- SoC and Terminal Voltage vs. Time');

results_dir = fullfile(this_dir, '..', 'results', 'plots');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
saveas(gcf, fullfile(results_dir, 'week1_battery_model_validation.png'));
fprintf('\nSaved plot to results/plots/week1_battery_model_validation.png\n');

% ---------------------------------------------------------------------
% Quick sanity checks worth confirming by eye in the plot:
%   1. SoC should decrease smoothly and roughly linearly for 'constant'.
%   2. Terminal voltage should sit below OCV (due to Ri, Vp, Vc drops)
%      and should dip further during each current pulse.
%   3. 'pulse' and 'motor' profiles (which touch 0 A) should show visible
%      voltage recovery ("relaxation") during OFF periods -- consistent
%      with the paper's observation that zero-current gaps make SoC
%      estimation harder later, in the EKF stage.
%   4. 'dual_pulse' (never reaches 0 A) should look smoother, without
%      full relaxation bounces.
% ---------------------------------------------------------------------
