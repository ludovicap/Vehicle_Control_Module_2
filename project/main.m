%% =========================================================================
%                         VEHICLE CONTROL MODULE 2
% ==========================================================================
% Author:       Ludovica Perroni
% Affiliation:  UNICAL - DIMES, Robotics and Automation Engineering
% Updated:      20 / 03 / 2026
%
% ==========================================================================

clear all;
close all;
clc;

%% ========================================================================
%  MAIN SCRIPT - LPV H2 LATERAL CONTROL VALIDATION
%
%  Workflow:
%  1) Load or generate reference data from driving scenario
%  2) Export initial conditions and references to base workspace
%  3) Optional scenario animation
%  4) Compute LPV H2 gains over velocity intervals
%  5) Run Simulink model
%  6) Plot tracking and control results
%  ========================================================================

%% ---------------------- USER OPTIONS -------------------------------------
refFile = 'reference_data.mat';   % File used to store reference signals
forceRebuild = false;             % true -> regenerate references from scenario
showScenarioAnimation = false;    % true -> show scenario preview animation

%% ---------------------- 1. LOAD OR GENERATE REFERENCES -------------------
if isfile(refFile) && ~forceRebuild

    % Load previously saved reference data
    load(refFile, ...
        'ego_x', 'ego_y', 'ego_yaw', 'ego_v', 't', ...
        'x0', 'y0', 'yaw0', 'v0');

    % Rebuild scenario only for visualization / plotting
    [scenario, egoVehicle] = scenario_build();

else
    % Build scenario and extract reference trajectory from ego vehicle
    [scenario, egoVehicle] = scenario_build();

    % Initial conditions
    x0   = egoVehicle.Position(1);
    y0   = egoVehicle.Position(2);
    yaw0 = deg2rad(egoVehicle.Yaw);

    vx0 = egoVehicle.Velocity(1);
    vy0 = egoVehicle.Velocity(2);
    v0  = vx0*cos(yaw0) + vy0*sin(yaw0);

    % Storage initialization
    ego_x   = x0;
    ego_y   = y0;
    ego_yaw = yaw0;

    vx = vx0;
    vy = vy0;
    ego_v = v0;

    t = 0;

    % Advance scenario and collect reference data
    while advance(scenario)

        currentYaw = deg2rad(egoVehicle.Yaw);
        currentVx  = egoVehicle.Velocity(1);
        currentVy  = egoVehicle.Velocity(2);
        currentV   = currentVx*cos(currentYaw) + currentVy*sin(currentYaw);

        ego_x   = [ego_x; egoVehicle.Position(1)];
        ego_y   = [ego_y; egoVehicle.Position(2)];
        ego_yaw = [ego_yaw; currentYaw];
        ego_v   = [ego_v; currentV];
        t       = [t; scenario.SimulationTime];
    end

    % Save numeric reference data
    save(refFile, ...
        'ego_x', 'ego_y', 'ego_yaw', 'ego_v', 't', ...
        'x0', 'y0', 'yaw0', 'v0');
end

%% ---------------------- 2. EXPORT TO BASE WORKSPACE ----------------------
assignin('base', 'x0', x0);
assignin('base', 'y0', y0);
assignin('base', 'yaw0', yaw0);
assignin('base', 'v0', v0);
assignin('base', 'scenario0', scenario);

v_ref   = timeseries(ego_v, t);
yaw_ref = timeseries(ego_yaw, t);

assignin('base', 'v_ref', v_ref);
assignin('base', 'yaw_ref', yaw_ref);

%% ---------------------- 3. OPTIONAL SCENARIO VISUALIZATION ---------------
if showScenarioAnimation

    [scenario_vis, egoVehicle_vis] = scenario_build();

    figAnim = figure( ...
        'Name', 'Driving Scenario Preview', ...
        'Color', 'w', ...
        'WindowState', 'maximized');

    tl = tiledlayout(figAnim, 1, 2, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    % --- Top view
    axTop = nexttile(tl, 1);
    plot(scenario_vis, 'Parent', axTop, 'Meshes', 'on');
    view(axTop, 2);
    axis(axTop, 'equal');
    grid(axTop, 'on');
    grid(axTop, 'minor');
    box(axTop, 'on');
    xlabel(axTop, 'X [m]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel(axTop, 'Y [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title(axTop, 'Scenario Top View', 'FontSize', 14, 'FontWeight', 'bold');
    set(axTop, 'FontSize', 11, 'LineWidth', 1);

    % --- Chase view
    axChase = nexttile(tl, 2);
    chasePlot(egoVehicle_vis, 'Parent', axChase, 'Meshes', 'on');
    grid(axChase, 'on');
    box(axChase, 'on');
    title(axChase, 'Ego-Vehicle Chase View', ...
        'FontSize', 14, 'FontWeight', 'bold');
    set(axChase, 'FontSize', 11, 'LineWidth', 1);

    sgtitle(tl, 'Driving Scenario Animation', ...
        'FontSize', 16, 'FontWeight', 'bold');

    while advance(scenario_vis)
        drawnow limitrate;
    end
end

%% ---------------------- 4. LPV H2 GAIN COMPUTATION -----------------------
L = 2.8;  % Vehicle wheelbase

V_interval = { ...
    [0.0 1.2], ...
    [1.2 2.4], ...
    [2.4 3.6], ...
    [3.6 4.8], ...
    [4.8 6.0], ...
    [6.0 7.0]};

nIntervals = numel(V_interval);

sim_model = 'VehiclePlant';
load_system(sim_model);

xeq = [0; 0];
ueq = 0;

Kvec = cell(1, nIntervals);

for i = 1:nIntervals

    % Lower bound of the interval
    Vx = V_interval{i}(1); 
    assignin('base', 'Vx', Vx);

    [A, B, ~, ~] = linmod(sim_model, xeq, ueq);
    [Aa, Ba, Cza, Dzua, Bwa, ~] = AugmentedState(A, B);
    K_low = fun_H2(Aa, Ba, Bwa, Cza, Dzua);

    % Upper bound of the interval
    Vx = V_interval{i}(2); 
    assignin('base', 'Vx', Vx);

    [A, B, ~, ~] = linmod(sim_model, xeq, ueq);
    [Aa, Ba, Cza, Dzua, Bwa, ~] = AugmentedState(A, B);
    K_high = fun_H2(Aa, Ba, Bwa, Cza, Dzua);

    Kvec{i} = [K_low K_high];
end

K1 = Kvec{1};
K2 = Kvec{2};
K3 = Kvec{3};
K4 = Kvec{4};
K5 = Kvec{5};
K6 = Kvec{6};

assignin('base', 'K1', K1);
assignin('base', 'K2', K2);
assignin('base', 'K3', K3);
assignin('base', 'K4', K4);
assignin('base', 'K5', K5);
assignin('base', 'K6', K6);

%% ---------------------- 5. SIMULATION PARAMETERS -------------------------
Ts   = 0.01;
Tend = t(end);
tau  = 0.1;

assignin('base', 'Ts', Ts);
assignin('base', 'Tend', Tend);
assignin('base', 'tau', tau);

%% ---------------------- 6. RUN SIMULINK MODEL ----------------------------
simOut = sim('simulation');

%% ---------------------- 7. EXTRACT SIMULATION OUTPUTS --------------------
t_sim = simOut.tout;

x_sim = simOut.position.Data(:,1);
y_sim = simOut.position.Data(:,2);

u = simOut.u.Data(:,1);

yaw_ref_sim = simOut.yaw.Data(:,1);
yaw_sim     = simOut.yaw.Data(:,2);

v_ref_sim = simOut.v.Data(:,1);
v_sim     = simOut.v.Data(:,2);

sigmoid = simOut.sigmoid.Data;
noise = simOut.disturbance.Data;

%% ---------------------- 8. PERFORMANCE INDICES ---------------------------
yaw_error = yaw_ref_sim - yaw_sim;
v_error   = v_ref_sim - v_sim;

rmse_yaw = sqrt(mean(yaw_error.^2));
rmse_v   = sqrt(mean(v_error.^2));
max_u    = max(abs(u));

disp(' ');
disp('==================== PERFORMANCE INDICES ====================');
disp(['Yaw RMSE = ', num2str(rmse_yaw, '%.6f'), ' rad']);
disp(['Velocity RMSE = ', num2str(rmse_v,   '%.6f'), ' m/s']);
disp(['Maximum |steering| = ', num2str(max_u,    '%.6f'), ' rad']);
disp('=============================================================');
disp(' ');

%% ---------------------- 9. YAW TRACKING ----------------------------------
figure( ...
    'Name', 'Yaw Tracking - H2 Control', ...
    'Color', 'w', ...
    'WindowState', 'maximized');

hold on;
plot(t_sim, yaw_ref_sim, '--', 'LineWidth', 2.0, ...
    'DisplayName', 'Reference yaw');
plot(t_sim, yaw_sim, '-', 'LineWidth', 2.0, ...
    'DisplayName', 'Simulated yaw');

grid on;
grid minor;
box on;
axis tight;

xlabel('Time [s]', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Yaw angle [rad]', 'FontSize', 13, 'FontWeight', 'bold');
title('Yaw Tracking Performance', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1);

hold off;

%% ---------------------- 10. YAW TRACKING ERROR ---------------------------
figure( ...
    'Name', 'Yaw Error - H2 Control', ...
    'Color', 'w', ...
    'WindowState', 'maximized');

plot(t_sim, yaw_error, 'LineWidth', 2.0);

grid on;
grid minor;
box on;
axis tight;

xlabel('Time [s]', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Yaw error [rad]', 'FontSize', 13, 'FontWeight', 'bold');
title('Yaw Tracking Error', 'FontSize', 15, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'LineWidth', 1);

%% ---------------------- 11. TRAJECTORY COMPARISON ------------------------
figure( ...
    'Name', 'Trajectory Comparison - H2 Control', ...
    'Color', 'w', ...
    'WindowState', 'maximized');

ax = gca;
plot(scenario, 'Parent', ax, 'Meshes', 'on');
hold(ax, 'on');

z_ref = 0.1 * ones(size(ego_x));
z_sim = 0.1 * ones(size(x_sim));

hRef = plot3(ax, ego_x, ego_y, z_ref, ...
    'LineWidth', 2.5, ...
    'DisplayName', 'Reference trajectory');

hSim = plot3(ax, x_sim, y_sim, z_sim, ...
    'LineWidth', 2.5, ...
    'DisplayName', 'Simulated trajectory');

view(ax, 2);
axis(ax, 'equal');
grid(ax, 'on');
grid(ax, 'minor');
box(ax, 'on');

xlabel(ax, 'X [m]', 'FontSize', 13, 'FontWeight', 'bold');
ylabel(ax, 'Y [m]', 'FontSize', 13, 'FontWeight', 'bold');
title(ax, 'Reference and Simulated Vehicle Trajectories', ...
    'FontSize', 15, 'FontWeight', 'bold');

legend(ax, [hRef, hSim], ...
    'Reference Trajectory', 'Simulated Trajectory', ...
    'Location', 'best', 'FontSize', 12);

set(ax, 'FontSize', 12, 'LineWidth', 1);

hold(ax, 'off');

%% ---------------------- 12. STEERING INPUT -------------------------------
figure( ...
    'Name', 'Steering Input - H2 Control', ...
    'Color', 'w', ...
    'WindowState', 'maximized');

plot(t_sim, u, 'LineWidth', 2.0);

grid on;
grid minor;
box on;
axis tight;

xlabel('Time [s]', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Steering angle \delta [rad]', 'FontSize', 13, 'FontWeight', 'bold');
title('Steering Control Input', 'FontSize', 15, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'LineWidth', 1);

%% ---------------------- 13. VELOCITY TRACKING ----------------------------
figure( ...
    'Name', 'Velocity Tracking - H2 Control', ...
    'Color', 'w', ...
    'WindowState', 'maximized');

hold on;
plot(t_sim, v_ref_sim, '--', 'LineWidth', 2.0, ...
    'DisplayName', 'Reference velocity');
plot(t_sim, v_sim, '-', 'LineWidth', 2.0, ...
    'DisplayName', 'Simulated velocity');

grid on;
grid minor;
box on;
axis tight;

xlabel('Time [s]', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Velocity [m/s]', 'FontSize', 13, 'FontWeight', 'bold');
title('Vehicle Speed Tracking', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1);

hold off;

%% ---------------------- 14. VELOCITY ERROR -------------------------------
figure( ...
    'Name', 'Velocity Error - H2 Control', ...
    'Color', 'w', ...
    'WindowState', 'maximized');

plot(t_sim, v_error, 'LineWidth', 2.0);

grid on;
grid minor;
box on;
axis tight;

xlabel('Time [s]', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Velocity error [m/s]', 'FontSize', 13, 'FontWeight', 'bold');
title('Velocity Tracking Error', 'FontSize', 15, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'LineWidth', 1);

%% ---------------------- 15. SIGMOID -------------------------------
figure( ...
    'Name', 'LPV Scheduling Weights (Sigmoid)', ...
    'Color', 'w', ...
    'WindowState', 'maximized');

plot(t_sim, sigmoid, 'LineWidth', 2.0);

grid on;
grid minor;
box on;
axis tight;

xlabel('Time [s]', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Weight value [-]', 'FontSize', 13, 'FontWeight', 'bold');
title('LPV Scheduling Weights (Sigmoid Functions)', 'FontSize', 15, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'LineWidth', 1);

%% ---------------------- 16. NOISE -------------------------------
figure( ...
    'Name', 'Noise', ...
    'Color', 'w', ...
    'WindowState', 'maximized');

plot(t_sim, noise, 'LineWidth', 2.0);

grid on;
grid minor;
box on;
axis tight;

xlabel('Time [s]', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Amplitude of noise [-]', 'FontSize', 13, 'FontWeight', 'bold');
title('Band-Limited White-Noise', 'FontSize', 15, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'LineWidth', 1);


%% ---------------------- SAVE ALL FIGURES TO PDF --------------------------

outputFolder = 'figures_pdf';

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

figHandles = findall(0, 'Type', 'figure');

for i = 1:length(figHandles)

    fig = figHandles(i);

    figure(fig);

    filename = fullfile(outputFolder, ...
        sprintf('Figure_%02d.pdf', i));

    exportgraphics(fig, filename, 'ContentType', 'vector');

end

disp('All figures saved in PDF format!');

%%
close all