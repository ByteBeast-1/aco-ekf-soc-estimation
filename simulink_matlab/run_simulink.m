% RUN_SIMULINK.M
% This script prepares the Drive Profile data and runs the Simulink Model!

clc; clear; close all;
model_name = 'advanced_bms_aco_ekf';

fprintf('==================================================\n');
fprintf('🔋 Preparing Data for Simulink BMS Simulation...\n');
fprintf('==================================================\n\n');

%% 1. Define the 4 Drive Profiles
t = (0:3600)';  % 3600 seconds simulation

% Profile 1: Highway Constant Current (CC)
I_highway = 1.0 * ones(size(t));

% Profile 2: Pulse Stop-Start (2A on, 0A off)
I_pulse = 2.0 * (mod(t, 90) < 60);

% Profile 3: UDDS Dynamic (Sine wave mix to simulate urban driving)
I_udds = 1.5 + 1.0*sin(2*pi*t/600) + 0.5*sin(2*pi*t/150);

% Profile 4: Fast Charging (Negative Current)
I_charge = -2.0 * ones(size(t));
I_charge(t > 2880) = -0.5; % Taper off at the end

%% 2. Ask User Which Profile to Run (Defaulting to Highway CC here)
% We pack it into a 2-column matrix: [Time, Current]
% This is EXACTLY what the 'From Workspace' block expects!
Drive_Profile = [t, I_udds]; 

% Send it to the Base Workspace so Simulink can see it
assignin('base', 'Drive_Profile', Drive_Profile);

fprintf('✅ Data Loaded into Workspace!\n');
fprintf('   Variable ''Drive_Profile'' is now ready.\n\n');

%% 3. Open and Run the Simulink Model
if ~bdIsLoaded(model_name)
    open_system(model_name);
end

% Enable Simulation Pacing so the Live SOC display animates dynamically
try
    set_param(model_name, 'EnablePacing', 'on', 'SimulationPacingRate', 20);
catch
end

fprintf('🚀 Starting Simulink Simulation...\n');
simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');

fprintf('\n🎉 SIMULATION COMPLETE!\n');
fprintf('👉 Double-click any of the Scopes in Simulink to see the results!\n');
