close all; clear all; clc;
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

% Initial Conditions
initCond = struct('pos', [0; 0; -10], 'vel', [0; 0; 0], 'quat', [1; 0; 0; 0], 'omg', [0; 0; 0]);
initInput = struct('T', inertialProperties.mass * 9.81, 'Mx', 0.0, 'My', 0.0, 'Mz', 0.0);

% Instantiate the Multicopter
QuadCopter = MultiCopter(initCond, initInput, inertialProperties, dt);

% PID Gains for the SuccessivePID Controller
GainsPID = struct('vel_kp', 10, 'vel_ki', 0, 'vel_kd', 0.1, ...
                  'att_kp', 1.0, 'att_ki', 0, 'att_kd', 0, ...
                  'omg_kp', 1, 'omg_ki', 0, 'omg_kd', 0);

% Instantiate the SuccessivePID Controller
Controller = SuccessivePID(dt, GainsPID);

% Wind Model (Dryden)
ambient_wind = [3; 0; 0]; % Ambient wind conditions
Dryden = WindDryden(dt, ambient_wind);

%% Logging Setup
% Pre-allocate space for loggers for efficiency
num_steps = sim_time * freq_omg + 1;
StateLogger = Logger(12, num_steps);
CommandLogger = Logger(12, num_steps);
CommandPropLogger = Logger(4, num_steps);
WindBodyLogger = Logger(3, num_steps);
WindGustLogger = Logger(3, num_steps);

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
    WindBodyLogger.update(Wb, step, time);
    WindGustLogger.update(Dryden.get_gust(), step, time);

    pos_err = ego_goal_point - QuadCopter.pos;
    vel_command = k_nav * pos_err;
    vel_command = v_max.*vel_command./norm(vel_command);
    
    full_state = [QuadCopter.pos; QuadCopter.vel; QuadCopter.quat; QuadCopter.omg];
    prop_command = Controller.update_PID(full_state, vel_command);
    propInput.T = prop_command(1);
    propInput.Mx = prop_command(2);
    propInput.My = prop_command(3);
    propInput.Mz = prop_command(4);
    QuadCopter.set_input(propInput);
    CommandPropLogger.update(prop_command, step, time);
    command_vec = [ego_goal_point; Controller.get_command()];
    CommandLogger.update(command_vec, step, time);
    QuadCopter.update_states();
    StateLogger.update(QuadCopter.get_state(), step, time);
    time = time + dt;
end

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

WindPlot = figure();
WindPlot.Theme = 'light';
subplot(3, 1, 1);
hold on; grid on;
plot(WindBodyLogger.time, WindBodyLogger.log(1,:));
ylabel('Wb_u (m/s)');

subplot(3, 1, 2);
hold on; grid on;
plot(WindBodyLogger.time, WindBodyLogger.log(2,:));
ylabel('Wb_v (m/s)');

subplot(3, 1, 3);
hold on; grid on;
plot(WindBodyLogger.time, WindBodyLogger.log(3,:));
ylabel('Wb_w (m/s)'); xlabel('Time (s)');

GustPlot = figure();
GustPlot.Theme = 'light';
subplot(3, 1, 1);
hold on; grid on;
plot(WindGustLogger.time, WindGustLogger.log(1,:));
ylabel('Wb_u (m/s)');

subplot(3, 1, 2);
hold on; grid on;
plot(WindGustLogger.time, WindGustLogger.log(2,:));
ylabel('Wb_v (m/s)');

subplot(3, 1, 3);
hold on; grid on;
plot(WindGustLogger.time, WindGustLogger.log(3,:));
ylabel('Wb_w (m/s)'); xlabel('Time (s)');

