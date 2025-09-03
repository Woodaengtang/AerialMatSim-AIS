close all; clear all; clc;
rng(0);
addpath("lib\");

%% Simulation Parameters
sim_time = 60;      % Total simulation time (s)
freq_omg = 1000;    % Control loop frequency (Hz)
dt = 1/freq_omg;    % Simulation time step

% Guidance Parameters
ego_goal_point = [50; 0; -10]; % Target position [N, E, D] (m)
k_nav = 0.5;                  % Position-to-velocity gain
v_max = 3.0;                  % Max commanded velocity per axis (m/s)

%% Class Setup
% Inertial Properties for the Multicopter
inertialProperties = struct('mass', 2, 'Jxx', 0.021667, 'Jyy', 0.021667, 'Jzz', 0.04, 'Jxy', 0.0, 'Jxz', 0.0, 'Jyz', 0.0);
Gamma_0 = [4.5, 0, 0;...
            0, 3, 0;...
            0, 0, 3];
Gamma_1 = 0.1.*Gamma_0;
% Gamma_1(3, 3) = 0;
dobGainK = 1500*ones([13, 1]);
dobGainK(6) = dobGainK(6)*1.2;
dobProperties = struct('gamma_0', Gamma_0, 'gamma_1', Gamma_1, 'K', diag(dobGainK), 'dt', dt);

% Initial Conditions
initCond = struct('pos', [0; 0; -10], 'vel', [0; 0; 0], 'quat', [1; 0; 0; 0], 'omg', [0; 0; 0]);
initInput = struct('T', inertialProperties.mass * 9.81, 'Mx', 0.0, 'My', 0.0, 'Mz', 0.0);

% Multicopter
QuadCopter = MultiCopter(initCond, initInput, inertialProperties, dt);

% PID Gains for the SuccessivePID Controller
GainsPID = struct('vel_kp', 10, 'vel_ki', 0, 'vel_kd', 0.1, ...
                  'att_kp', 1, 'att_ki', 0, 'att_kd', 0, ...
                  'omg_kp', 1, 'omg_ki', 0, 'omg_kd', 0);

% SuccessivePID Controller
Controller = SuccessivePID(dt, GainsPID);

% Wind Model (Dryden)
ambient_wind = [3; 0; 0]; % Ambient wind conditions
Dryden = WindDryden(dt, ambient_wind);

% Disturbance observer
Dob = DOB2(dobProperties, initCond);

%% Logging Setup
% Pre-allocate space for loggers for efficiency
num_steps = sim_time * freq_omg + 1;
StateLogger = Logger(12, num_steps);
CommandLogger = Logger(12, num_steps);
CommandPropLogger = Logger(4, num_steps);
DisturbanceLogger = Logger(3, num_steps);
WindGustLogger = Logger(3, num_steps);
DobStateLogger = Logger(13, num_steps);
DobLogger = Logger(3, num_steps);

%% Simulation Loop
time = 0;
step = 0;
vel_command = [0; 0; 0]; % Initialize velocity command

while time <= sim_time && norm(ego_goal_point - QuadCopter.pos) >= 1
    step = step + 1;

    height = -QuadCopter.pd;
    Va = norm(QuadCopter.vel - (ambient_wind + Dryden.get_gust()));
    Wb = Dryden.update_wind(height, Va, QuadCopter.att);
    QuadCopter.set_body_wind(Wb);

    pos_err = ego_goal_point - QuadCopter.pos;
    vel_command = k_nav * pos_err;
    vel_command = v_max.*vel_command./norm(vel_command);

    prop_command = Controller.update_PID(QuadCopter.get_state_quat(), vel_command);
    propInput.T = prop_command(1);
    propInput.Mx = prop_command(2);
    propInput.My = prop_command(3);
    propInput.Mz = prop_command(4);
    QuadCopter.set_input(propInput);    
    command_vec = [ego_goal_point; Controller.get_command()];
    
    QuadCopter.update_states();
    Dob.update_dist_estimate(QuadCopter.get_state_quat(), prop_command, QuadCopter);
    
    StateLogger.update(QuadCopter.get_state_eul(), step, time);
    CommandLogger.update(command_vec, step, time);
    CommandPropLogger.update(prop_command, step, time);
    DisturbanceLogger.update(quat2rotm(QuadCopter.quat')*QuadCopter.D*Wb, step, time);
    WindGustLogger.update(Dryden.get_gust(), step, time);
    DobStateLogger.update(Dob.get_state(), step, time);
    DobLogger.update(Dob.d_hat, step, time);
    
    time = time + dt;
end 

% Remove NaN values from loggers and save the cleaned data
StateLogger.remove_nan();
DobStateLogger.remove_nan();
CommandLogger.remove_nan();
CommandPropLogger.remove_nan();
DisturbanceLogger.remove_nan();
WindGustLogger.remove_nan();
DobLogger.remove_nan();

%% Post-Simulation Analysis & Plotting

Trajectory3D = figure();
Trajectory3D.Theme = 'light';
hold on; grid on;
plot3(StateLogger.log(1,:), StateLogger.log(2,:), StateLogger.log(3,:), 'LineWidth', 2, 'DisplayName', 'Trajectory');
plot3(initCond.pos(1), initCond.pos(2), initCond.pos(3), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
plot3(ego_goal_point(1), ego_goal_point(2), ego_goal_point(3), 'r*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');
xlabel('North (m)'); ylabel('East (m)'); zlabel('Down (m)');
title('3D Trajectory');
legend;

PosPlot = figure();
PosPlot.Theme = 'light';
subplot(3, 1, 1);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(1,:), 'LineWidth', 2);
plot(CommandLogger.time, CommandLogger.log(1,:), 'LineWidth', 2);
ylabel('North (m)');
title('Position - North');
legend('State', 'Command', 'Location', 'best');

subplot(3, 1, 2);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(2,:), 'LineWidth', 2);
plot(CommandLogger.time, CommandLogger.log(2,:), 'LineWidth', 2);
ylabel('East (m)');
title('Position - East');
legend('State', 'Command', 'Location', 'best');

subplot(3, 1, 3);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(3,:), 'LineWidth', 2);
plot(CommandLogger.time, CommandLogger.log(3,:), 'LineWidth', 2);
ylabel('Down (m)');
xlabel('Time (s)');
title('Position - Down');
legend('State', 'Command', 'Location', 'best');

VelPlot = figure();
VelPlot.Theme = 'light';
subplot(3, 1, 1);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(4,:), 'LineWidth', 2);
plot(CommandLogger.time, CommandLogger.log(4,:), 'LineWidth', 2);
ylabel('V_N (m/s)');
title('Velocity - North');
legend('State', 'Command', 'Location', 'best');

subplot(3, 1, 2);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(5,:), 'LineWidth', 2);
plot(CommandLogger.time, CommandLogger.log(5,:), 'LineWidth', 2);
ylabel('V_E (m/s)');
title('Velocity - East');
legend('State', 'Command', 'Location', 'best');

subplot(3, 1, 3);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(6,:), 'LineWidth', 2);
plot(CommandLogger.time, CommandLogger.log(6,:), 'LineWidth', 2);
ylabel('V_D (m/s)');
xlabel('Time (s)');
title('Velocity - Down');
legend('State', 'Command', 'Location', 'best');

AttPlot = figure();
AttPlot.Theme = 'light';
subplot(3, 1, 1);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(7,:)), 'LineWidth', 2);
plot(CommandLogger.time, rad2deg(CommandLogger.log(7,:)), 'LineWidth', 2);
ylabel('Roll (deg)');
title('Attitude - Roll');
legend('State', 'Command', 'Location', 'best');

subplot(3, 1, 2);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(8,:)), 'LineWidth', 2);
plot(CommandLogger.time, rad2deg(CommandLogger.log(8,:)), 'LineWidth', 2);
ylabel('Pitch (deg)');
title('Attitude - Pitch');
legend('State', 'Command', 'Location', 'best');

subplot(3, 1, 3);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(9,:)), 'LineWidth', 2);
plot(CommandLogger.time, rad2deg(CommandLogger.log(9,:)), 'LineWidth', 2);
ylabel('Yaw (deg)');
xlabel('Time (s)');
title('Attitude - Yaw');
legend('State', 'Command', 'Location', 'best');

OmgPlot = figure();
OmgPlot.Theme = 'light';
subplot(3, 1, 1);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(10,:)), 'LineWidth', 2);
plot(CommandLogger.time, rad2deg(CommandLogger.log(10,:)), 'LineWidth', 2);
ylabel('P (deg/s)');
title('Angular Rate - Roll Rate');
legend('State', 'Command', 'Location', 'best');

subplot(3, 1, 2);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(11,:)), 'LineWidth', 2);
plot(CommandLogger.time, rad2deg(CommandLogger.log(11,:)), 'LineWidth', 2);
ylabel('Q (deg/s)');
title('Angular Rate - Pitch Rate');
legend('State', 'Command', 'Location', 'best');

subplot(3, 1, 3);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(12,:)), 'LineWidth', 2);
plot(CommandLogger.time, rad2deg(CommandLogger.log(12,:) ), 'LineWidth', 2);
ylabel('R (deg/s)');
xlabel('Time (s)');
title('Angular Rate - Yaw Rate');
legend('State', 'Command', 'Location', 'best');

InputPlot = figure();
InputPlot.Theme = 'light';
subplot(4, 1, 1);
hold on; grid on;
plot(CommandPropLogger.time, CommandPropLogger.log(1,:));
ylabel('Thrust (N)');

subplot(4, 1, 2);
hold on; grid on;
plot(CommandPropLogger.time, CommandPropLogger.log(2,:));
ylabel('M_x (Nm)');

subplot(4, 1, 3);
hold on; grid on;
plot(CommandPropLogger.time, CommandPropLogger.log(3,:));
ylabel('M_y (Nm)');

subplot(4, 1, 4);
hold on; grid on;
plot(CommandPropLogger.time, CommandPropLogger.log(4,:));
ylabel('M_z (Nm)'); xlabel('Time (s)');

wind_width = 1.5;
WindPlot = figure();
WindPlot.Theme = 'light';
subplot(3, 1, 1);
hold on; grid on;
plot(DisturbanceLogger.time, DisturbanceLogger.log(1,:), 'DisplayName', 'Wind Dist', 'LineWidth', wind_width);
plot(DobLogger.time, DobLogger.log(1, :), 'DisplayName', 'Est Wind Dist', 'LineWidth', wind_width);
ylabel('$Wb_u (m/s^2)$', 'Interpreter', 'latex');
legend('Location','southeast');

subplot(3, 1, 2);
hold on; grid on;
plot(DisturbanceLogger.time, DisturbanceLogger.log(2,:), 'LineWidth', wind_width);
plot(DobLogger.time, DobLogger.log(2, :), 'LineWidth', wind_width);
ylabel('$Wb_v (m/s^2)$', 'Interpreter', 'latex');

subplot(3, 1, 3);
hold on; grid on;
plot(DisturbanceLogger.time, DisturbanceLogger.log(3,:), 'LineWidth', wind_width);
plot(DobLogger.time, DobLogger.log(3, :), 'LineWidth', wind_width);
ylabel('$Wb_w (m/s^2)$', 'Interpreter', 'latex'); xlabel('time (s)');

line_width = 1.5;
DobEstPosPlot = figure();
DobEstPosPlot.Theme = 'light';
subplot(3, 2, 1);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(1, :), 'DisplayName', 'MC state', 'LineWidth', line_width);
plot(DobStateLogger.time, DobStateLogger.log(1, :), 'DisplayName', 'DOB est', 'LineWidth', line_width);
ylabel('$p_n$ (m)', 'Interpreter', 'latex'); legend;
subtitle('Position');

subplot(3, 2, 2);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(1, :) - DobStateLogger.log(1, :), 'LineWidth', line_width);
subtitle('Position Error');

subplot(3, 2, 3);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(2, :), 'LineWidth', line_width);
plot(DobStateLogger.time, DobStateLogger.log(2, :), 'LineWidth', line_width);
ylabel('$p_e$ (m)', 'Interpreter', 'latex');

subplot(3, 2, 4);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(2, :) - DobStateLogger.log(2, :), 'LineWidth', line_width);

subplot(3, 2, 5);
hold on; grid on;
plot(DobStateLogger.time, DobStateLogger.log(3, :), 'LineWidth', line_width);
plot(DobStateLogger.time, DobStateLogger.log(3, :), 'LineWidth', line_width);
ylabel('$p_d$ (m)', 'Interpreter', 'latex'); xlabel('time (s)');

subplot(3, 2, 6);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(3, :) - DobStateLogger.log(3, :), 'LineWidth', line_width);
xlabel('time (s)');

DobEstVelPlot = figure();
DobEstVelPlot.Theme = 'light';
subplot(3, 2, 1);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(4, :), 'DisplayName', 'MC state', 'LineWidth', line_width);
plot(DobStateLogger.time, DobStateLogger.log(4, :), 'DisplayName', 'DOB est', 'LineWidth', line_width);
ylabel('$v_n$ (m/s)', 'Interpreter', 'latex'); legend;
subtitle('Velocity');

subplot(3, 2, 2);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(4, :) - DobStateLogger.log(4, :), 'LineWidth', line_width);
subtitle('Velocity Error');

subplot(3, 2, 3);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(5, :), 'LineWidth', line_width);
plot(DobStateLogger.time, DobStateLogger.log(5, :), 'LineWidth', line_width);
ylabel('$v_e$ (m/s)', 'Interpreter', 'latex');

subplot(3, 2, 4);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(5, :) - DobStateLogger.log(5, :), 'LineWidth', line_width);

subplot(3, 2, 5);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(6, :), 'LineWidth', line_width);
plot(DobStateLogger.time, DobStateLogger.log(6, :), 'LineWidth', line_width);
ylabel('$v_d$ (m/s)', 'Interpreter', 'latex'); xlabel('time (s)');

subplot(3, 2, 6);
hold on; grid on;
plot(StateLogger.time, StateLogger.log(6, :) - DobStateLogger.log(6, :), 'LineWidth', line_width);
xlabel('time (s)');

DobEstAttPlot = figure();
DobEstAttPlot.Theme = 'light';
DobEul = quat2eul(DobStateLogger.log(7:10, :)', 'XYZ')';
subplot(3, 2, 1);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(7, :)), 'LineWidth', line_width);
plot(DobStateLogger.time, rad2deg(DobEul(1, :)), 'LineWidth', line_width);
ylabel('$\phi$ (deg)', 'Interpreter', 'latex'); subtitle('Attitude');

subplot(3, 2, 2);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(7, :) - DobEul(1, :)), 'LineWidth', line_width);
subtitle('Attitude Error');

subplot(3, 2, 3);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(8, :)), 'LineWidth', line_width);
plot(DobStateLogger.time, rad2deg(DobEul(2, :)), 'LineWidth', line_width);
ylabel('$\theta$ (deg)', 'Interpreter', 'latex');

subplot(3, 2, 4);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(8, :) - DobEul(2, :)), 'LineWidth', line_width);

subplot(3, 2, 5);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(9, :)), 'LineWidth', line_width);
plot(DobStateLogger.time, rad2deg(DobEul(3, :)), 'LineWidth', line_width);
ylabel('$\psi$ (deg)', 'Interpreter', 'latex'); xlabel('time (s)');

subplot(3, 2, 6);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(9, :) - DobEul(3, :)), 'LineWidth', line_width);
xlabel('time (s)');

DobEstOmgPlot = figure();
DobEstOmgPlot.Theme = 'light';
subplot(3, 2, 1);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(10, :)), 'LineWidth', line_width);
plot(DobStateLogger.time, rad2deg(DobStateLogger.log(11, :)), 'LineWidth', line_width);
ylabel('p (deg/s)'); subtitle('Angular Rate');

subplot(3, 2, 2);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(10, :) - DobStateLogger.log(11, :)), 'LineWidth', line_width);
subtitle('Angular Rate Error');

subplot(3, 2, 3);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(11, :)), 'LineWidth', line_width);
plot(DobStateLogger.time, rad2deg(DobStateLogger.log(12, :)), 'LineWidth', line_width);
ylabel('q (deg/s)');

subplot(3, 2, 4);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(11, :) - DobStateLogger.log(12, :)), 'LineWidth', line_width);

subplot(3, 2, 5);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(12, :)), 'LineWidth', line_width);
plot(DobStateLogger.time, rad2deg(DobStateLogger.log(13, :)), 'LineWidth', line_width);
ylabel('r (deg/s)'); xlabel('time (s)');

subplot(3, 2, 6);
hold on; grid on;
plot(StateLogger.time, rad2deg(StateLogger.log(12, :) - DobStateLogger.log(13, :)), 'LineWidth', line_width);
xlabel('time (s)');

