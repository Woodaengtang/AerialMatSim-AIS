close all; clear all; clc;
addpath("lib\");
%% Class Setup
inertialProperties = struct('mass', 2,...
								'Jxx', 0.021667,...
								'Jyy', 0.021667,...
								'Jzz', 0.04,...
								'Jxy', 0.0,...
								'Jxz', 0.0,...
								'Jyz', 0.0);
initCond = struct('pos', [0; 0; -10],...
				  'vel', [3; 0; 0],...
				  'quat', [1; 0; 0; 0],...
				  'omg', [0; 0; 0]);
initInput = struct('T', 2*9.81,...
				   'Mx', 0.0,...
				   'My', 0.0,...
				   'Mz', 0.0);

sim_time = 60;
time = 0;
freq_vel = 20;
freq_att = 200;
freq_omg = 1000;
DEG2RAD = pi/180;

QuadCopter = MultiCopter(initCond, initInput, inertialProperties, 1/freq_omg);

% Wind model (Dryden) for disturbance
ambient_wind = [0; 0; 0];
Dryden = WindDryden(1/freq_omg, ambient_wind);

[gain_vel.kp, gain_vel.ki, gain_vel.kd] = deal(1, 0.001, 0.005);
[gain_att.kp, gain_att.ki, gain_att.kd] = deal(5, 0.1, 0);
[gain_omg.kp, gain_omg.ki, gain_omg.kd] = deal(0.5, 0.2, 0);

CtrlVel     = ControllerPID(gain_vel.kp, gain_vel.ki, gain_vel.kd, 1/freq_vel);
CtrlThrust  = ControllerPID(gain_vel.kp, gain_vel.ki, gain_vel.kd, 1/freq_vel);
CtrlAtt     = ControllerPID(gain_att.kp, gain_att.ki, gain_att.kd, 1/freq_att);
CtrlOmg     = ControllerPID(gain_omg.kp, gain_omg.ki, gain_omg.kd, 1/freq_omg);

% Optional output saturations
CtrlVel.set_output_saturation(-15, 15); % deg for roll/pitch cmd before DEG2RAD mapping
CtrlAtt.set_output_saturation(-20*DEG2RAD, 20*DEG2RAD); % rad/s rate references
CtrlOmg.set_output_saturation(-2, 2); % rad/s^2 equivalent torque response

vel_command = zeros([3, 1]);
att_command = zeros([3, 1]);
omg_command = zeros([3, 1]);
prop_command = zeros([4, 1]);
rate_command = zeros([3, 1]);

% Reference generation parameters
k_nav = 0.5;    % position-to-velocity gain
v_max = 3.0;    % max commanded velocity per axis (m/s)

% Loggers for commands and states
CommandVelLogger = Logger(length(vel_command), sim_time*freq_omg + 1);
CommandAttLogger = Logger(length(att_command), sim_time*freq_omg + 1);
CommandOmgLogger = Logger(length(omg_command), sim_time*freq_omg + 1);
CommandPropLogger = Logger(length(prop_command), sim_time*freq_omg + 1);

StatePosLogger = Logger(3, sim_time*freq_omg + 1);
StateVelLogger = Logger(3, sim_time*freq_omg + 1);
StateAttLogger = Logger(3, sim_time*freq_omg + 1);
StateOmgLogger = Logger(3, sim_time*freq_omg + 1);

RefVelLogger = Logger(3, sim_time*freq_vel + 1);
RefAttLogger = Logger(3, sim_time*freq_att + 1);
RefOmgLogger = Logger(3, sim_time*freq_omg + 1);

% Wind logging
WindBodyLogger = Logger(3, sim_time*freq_omg + 1);      % Body frame wind components
WindInertialLogger = Logger(3, sim_time*freq_omg + 1);  % Inertial frame wind components
WindMagnitudeLogger = Logger(1, sim_time*freq_omg + 1); % Wind magnitude

simulTime = linspace(0, sim_time, sim_time*freq_omg+1);
%% Simul setup
ego_goal_point = [50; 0; -10];
Index = struct('successive_', 0,...
			   'guidance_', 50,...
			   'attitude_', 5,...
			   'guidance_loop_', 0,...
			   'attitude_loop_', 0);
guidance_idx = int32(floor(freq_omg / freq_vel));
attitude_idx = int32(floor(freq_omg / freq_att));

% Simulation step counter
actual_steps = 0;


%% Simulation
while and(time <= sim_time, norm(ego_goal_point - QuadCopter.pos) >= 1)
	% Wind update (body frame) and injection
	height = -QuadCopter.pd;         % NED down -> altitude
	Va = norm(QuadCopter.vel);      % approximate airspeed with ground speed
	Wb = Dryden.update_wind(height, Va, QuadCopter.att);
	QuadCopter.set_body_wind(Wb);
	
	% Log wind information
	WindBodyLogger.update(Wb, Index.successive_+1, time);
	
	% Convert body frame wind to inertial frame for analysis
	rotmB2I = QuadCopter.get_rotm_body2inertial();
	Wi = rotmB2I * Wb;
	WindInertialLogger.update(Wi, Index.successive_+1, time);
	
	% Log wind magnitude
	wind_magnitude = norm(Wb);
	WindMagnitudeLogger.update(wind_magnitude, Index.successive_+1, time);

	if mod(Index.successive_, Index.guidance_) == 0
		Index.guidance_loop_ = Index.guidance_loop_ + 1;
		
		% Position-based velocity command
		pos_err = ego_goal_point - QuadCopter.pos;
		vel_command = k_nav * pos_err;
		vel_command = max(min(vel_command, v_max), -v_max);
		
		% Thrust (vertical) via velocity tracking
		prop_command(1) = QuadCopter.mass*9.81 - CtrlThrust.update(vel_command(3), QuadCopter.vel(3));
		
		% Log references for velocity
		RefVelLogger.update(vel_command, Index.guidance_loop_, time);
		CommandVelLogger.update(vel_command, Index.guidance_loop_, time);
		
		% Map (vx, vy) tracking error to attitude references (roll, pitch)
        att_command(1:2) = CtrlVel.update(vel_command(1:2), QuadCopter.vel(1:2)).*DEG2RAD; % roll, pitch in rad
		att_command(3) = atan2(vel_command(2), vel_command(1)); % yaw towards travel direction
	end
	if mod(Index.successive_, Index.attitude_) == 0
		Index.attitude_loop_ = Index.attitude_loop_ + 1;
		RefAttLogger.update(att_command, Index.attitude_loop_, time);
		CommandAttLogger.update(att_command, Index.attitude_loop_, time);

		% Attitude controller -> body rate command
		rate_command = CtrlAtt.update(att_command, QuadCopter.att);
	end
	RefOmgLogger.update(rate_command, Index.successive_+1, time);
	CommandOmgLogger.update(rate_command, Index.successive_+1, time);

	% Body rate tracking -> moments
	prop_command(2:4) = CtrlOmg.update(rate_command, QuadCopter.omg);
	CommandPropLogger.update(prop_command, Index.successive_+1, time);

	% Apply inputs and integrate dynamics
	propInput.T = prop_command(1);
	propInput.Mx = prop_command(2);
	propInput.My = prop_command(3);
	propInput.Mz = prop_command(4);
	QuadCopter.set_input(propInput);
	QuadCopter.update_states();

	% Log current states at fast rate
	StatePosLogger.update(QuadCopter.pos, Index.successive_+1, time);
	StateVelLogger.update(QuadCopter.vel, Index.successive_+1, time);
	StateAttLogger.update(QuadCopter.att, Index.successive_+1, time);
	StateOmgLogger.update(QuadCopter.omg, Index.successive_+1, time);

	Index.successive_ = Index.successive_ + 1;
	time = time + 1/freq_omg;
	actual_steps = actual_steps + 1;
end

% Display simulation completion information
if norm(ego_goal_point - QuadCopter.pos) < 1
    disp(['Simulation completed successfully in ', num2str(actual_steps), ' steps (', num2str(time), ' seconds)']);
    disp(['Final position: [', num2str(QuadCopter.pos(1), '%.3f'), ', ', num2str(QuadCopter.pos(2), '%.3f'), ', ', num2str(QuadCopter.pos(3), '%.3f'), '] m']);
    disp(['Distance to goal: ', num2str(norm(ego_goal_point - QuadCopter.pos), '%.3f'), ' m']);
else
    disp(['Simulation reached time limit: ', num2str(time), ' seconds']);
    disp(['Final position: [', num2str(QuadCopter.pos(1), '%.3f'), ', ', num2str(QuadCopter.pos(2), '%.3f'), ', ', num2str(QuadCopter.pos(3), '%.3f'), '] m']);
    disp(['Distance to goal: ', num2str(norm(ego_goal_point - QuadCopter.pos), '%.3f'), ' m']);
end

% Wind statistics
wind_data = WindMagnitudeLogger.log(1,:);
wind_data = wind_data(~isnan(wind_data)); % Remove NaN values
if ~isempty(wind_data)
    disp('Wind Statistics:');
    disp(['  Average wind magnitude: ', num2str(mean(wind_data), '%.3f'), ' m/s']);
    disp(['  Max wind magnitude: ', num2str(max(wind_data), '%.3f'), ' m/s']);
    disp(['  Min wind magnitude: ', num2str(min(wind_data), '%.3f'), ' m/s']);
    disp(['  Wind magnitude std: ', num2str(std(wind_data), '%.3f'), ' m/s']);
    disp(['  Ambient wind: ', num2str(norm(ambient_wind), '%.3f'), ' m/s']);
else
    disp('No wind data available');
end

% Position statistics
pos_data = StatePosLogger.log;
pos_data = pos_data(:, ~isnan(pos_data(1,:))); % Remove NaN columns
if ~isempty(pos_data)
    disp('Position Statistics:');
    disp(['  Initial position: [', num2str(pos_data(1,1), '%.2f'), ', ', num2str(pos_data(2,1), '%.2f'), ', ', num2str(pos_data(3,1), '%.2f'), '] m']);
    disp(['  Final position: [', num2str(pos_data(1,end), '%.2f'), ', ', num2str(pos_data(2,end), '%.2f'), ', ', num2str(pos_data(3,end), '%.2f'), '] m']);
    disp(['  Goal position: [', num2str(ego_goal_point(1), '%.2f'), ', ', num2str(ego_goal_point(2), '%.2f'), ', ', num2str(ego_goal_point(3), '%.2f'), '] m']);
    
    % Calculate final position error
    final_pos_err = norm(pos_data(:,end) - ego_goal_point);
    disp(['  Final position error: ', num2str(final_pos_err, '%.3f'), ' m']);
    
    % Calculate total distance traveled
    total_distance = 0;
    for i = 2:size(pos_data, 2)
        total_distance = total_distance + norm(pos_data(:,i) - pos_data(:,i-1));
    end
    disp(['  Total distance traveled: ', num2str(total_distance, '%.3f'), ' m']);
    
    % Add simulation efficiency metrics
    if norm(ego_goal_point - QuadCopter.pos) < 1
        disp(['  Mission success! Reached goal in ', num2str(actual_steps), ' steps']);
        disp(['  Average speed: ', num2str(total_distance/time, '%.3f'), ' m/s']);
    else
        disp('  Mission incomplete - reached time limit');
    end
end

% Plots: commands vs states
PropFig = figure();
PropFig.Theme = 'light';
for i = 1:size(CommandPropLogger.log, 1)
	subplot(size(CommandPropLogger.log, 1), 1, i);
	hold on; grid on;
	plot(CommandPropLogger.time, CommandPropLogger.log(i,:))
end

grid on;

VelFig = figure();
VelFig.Theme = 'light';
sgtitle('Velocity tracking (ref vs actual)');
for i = 1:3
	subplot(3,1,i); grid on; hold on;
	plot(RefVelLogger.time, RefVelLogger.log(i,:), 'k--');
	plot(StateVelLogger.time, StateVelLogger.log(i,:), 'b-');
	legend('v_{ref}', 'v'); ylabel(['v_' char('x'+i-1) ' (m/s)']);
end
xlabel('Time (s)');

AttFig = figure();
AttFig.Theme = 'light';
sgtitle('Attitude tracking (ref vs actual)');
for i = 1:3
	subplot(3,1,i); grid on; hold on;
	plot(RefAttLogger.time, rad2deg(RefAttLogger.log(i,:)), 'k--');
	plot(StateAttLogger.time, rad2deg(StateAttLogger.log(i,:)), 'r-');
	legend('\phi/\theta/\psi_{ref}', '\phi/\theta/\psi'); ylabel('deg');
end
xlabel('Time (s)');

RateFig = figure();
RateFig.Theme = 'light';
sgtitle('Body rate tracking (ref vs actual)');
for i = 1:3
	subplot(3,1,i); grid on; hold on;
	plot(RefOmgLogger.time, rad2deg(RefOmgLogger.log(i,:)), 'k--');
	plot(StateOmgLogger.time, rad2deg(StateOmgLogger.log(i,:)), 'm-');
	legend('p/q/r_{ref}', 'p/q/r'); ylabel('deg/s');
end
xlabel('Time (s)');

% Position analysis plots
PosFig = figure();
PosFig.Theme = 'light';
sgtitle('Position trajectory analysis');

% 3D trajectory plot
subplot(2,2,1); grid on; hold on;
plot3(StatePosLogger.log(1,:), StatePosLogger.log(2,:), StatePosLogger.log(3,:), 'b-', 'LineWidth', 2);
plot3(ego_goal_point(1), ego_goal_point(2), ego_goal_point(3), 'r*', 'MarkerSize', 15, 'LineWidth', 3);
plot3(StatePosLogger.log(1,1), StatePosLogger.log(2,1), StatePosLogger.log(3,1), 'go', 'MarkerSize', 10, 'LineWidth', 3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); title('3D Trajectory');
legend('Trajectory', 'Goal', 'Start', 'Location', 'best');
view(45, 30); grid on;

% Individual position components over time
subplot(2,2,2); grid on; hold on;
plot(StatePosLogger.time, StatePosLogger.log(1,:), 'b-', 'LineWidth', 1.5);
plot(StatePosLogger.time, ego_goal_point(1)*ones(size(StatePosLogger.time)), 'r--', 'LineWidth', 1.5);
ylabel('X position (m)'); title('X Position vs Time');
legend('Actual', 'Goal', 'Location', 'best'); grid on;

subplot(2,2,3); grid on; hold on;
plot(StatePosLogger.time, StatePosLogger.log(2,:), 'g-', 'LineWidth', 1.5);
plot(StatePosLogger.time, ego_goal_point(2)*ones(size(StatePosLogger.time)), 'r--', 'LineWidth', 1.5);
ylabel('Y position (m)'); title('Y Position vs Time');
legend('Actual', 'Goal', 'Location', 'best'); grid on;

subplot(2,2,4); grid on; hold on;
plot(StatePosLogger.time, StatePosLogger.log(3,:), 'm-', 'LineWidth', 1.5);
plot(StatePosLogger.time, ego_goal_point(3)*ones(size(StatePosLogger.time)), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Z position (m)'); title('Z Position vs Time');
legend('Actual', 'Goal', 'Location', 'best'); grid on;

% Wind analysis plots
WindFig = figure();
WindFig.Theme = 'light';
sgtitle('Wind disturbance analysis');

% Body frame wind components
subplot(3,1,1); grid on; hold on;
for i = 1:3
	plot(WindBodyLogger.time, WindBodyLogger.log(i,:), 'DisplayName', ['Wb_' char('x'+i-1)]);
end
ylabel('Body frame wind (m/s)'); legend('Location', 'best'); title('Body frame wind components');

% Inertial frame wind components  
subplot(3,1,2); grid on; hold on;
for i = 1:3
	plot(WindInertialLogger.time, WindInertialLogger.log(i,:), 'DisplayName', ['Wi_' char('x'+i-1)]);
end
ylabel('Inertial frame wind (m/s)'); legend('Location', 'best'); title('Inertial frame wind components');

% Wind magnitude
subplot(3,1,3); grid on; hold on;
plot(WindMagnitudeLogger.time, WindMagnitudeLogger.log(1,:), 'b-', 'LineWidth', 1.5);
ylabel('Wind magnitude (m/s)'); title('Total wind magnitude');
grid on;
