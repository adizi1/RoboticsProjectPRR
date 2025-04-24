% Define initial and end positions for the simulations
P_s1 = [10, 3, 2]; % Starting position 1 [cm]
P_e1 = [13, 1, 2]; % Ending position 1 [cm]
P_e2 = [8, 4, 2]; % Ending position 2 [cm]

% Define alphas for the simulations
alpha1 = 1; % No damping effect
alpha2 = 0.05; % Some damping effect

% Run simulations with alpha = 1
fprintf('Running Simulation 1 with alpha = 1...\n');
PRR_simulation_ProjectAZ(P_s1, P_e1, alpha1);

% Run simulations with alpha between 0 and 0.1
fprintf('Running Simulation 2 with alpha = 0.05...\n');
PRR_simulation(P_s1, P_e1, alpha2);
%%
close all

fprintf('Running Simulation 3 with alpha = 1...\n');
PRR_simulation(P_s1, P_e2, alpha1);

fprintf('Running Simulation 4 with alpha = 0.05...\n');
PRR_simulation(P_s1, P_e2, alpha2);
