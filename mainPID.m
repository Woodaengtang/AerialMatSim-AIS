close all; clear all; clc;
addpath("..\lib\");
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

[gain_vel.kp, gain_vel.ki, gain_vel.kd] = deal(1, 0.001, 0.005);
[gain_att.kp, gain_att.ki, gain_att.kd] = deal(1, 0, 0);
[gain_omg.kp, gain_omg.ki, gain_omg.kd] = deal(1, 0.2, 0);

CtrlVel = ControllerPID(gain_vel.kp, gain_vel.ki, gain_vel.kd, 1/freq_vel);
CtrlThrust = ControllerPID(gain_vel.kp, gain_vel.ki, gain_vel.kd, 1/freq_vel);
CtrlAtt = ControllerPID(gain_att.kp, gain_att.ki, gain_att.kd, 1/freq_att);
CtrlOmg = ControllerPID(gain_omg.kp, gain_omg.ki, gain_omg.kd, 1/freq_omg);

vel_command = zeros([3, 1]);
att_command = zeros([3, 1]);
omg_command = zeros([3, 1]);
prop_command = zeros([4, 1]);


CommandVelLogger = Logger(length(vel_command), sim_time*freq_vel + 1);
CommandAttLogger = Logger(length(att_command), sim_time*freq_att + 1);
CommandOmgLogger = Logger(length(omg_command), sim_time*freq_omg + 1);
CommandPropLogger = Logger(length(prop_command), sim_time*freq_omg + 1);

simulTime = linspace(0, sim_time, sim_time*freq_omg+1);
%% Simul setup
ego_goal_point = [0; 50; -10];
Index = struct('successive_', 0,...
               'guidance_', 50,...
               'attitude_', 5,...
               'guidance_loop_', 0,...
               'attitude_loop_', 0);
guidance_idx = int32(floor(freq_omg / freq_vel));
attitude_idx = int32(floor(freq_omg / freq_att));


%% Simulation
while time <= sim_time
    if mod(Index.successive_, Index.guidance_) == 0
        Index.guidance_loop_ = Index.guidance_loop_ + 1;
        
        % velocity command generation
        % vel_command = ~;
        prop_command(1) = QuadCopter.mass*9.81 - CtrlThrust.update(vel_command(3), QuadCopter.vel(3));

        CommandVelLogger.update(vel_command, Index.guidance_loop_, time);
        att_command(1:2) = CtrlVel.update(vel_command(1:2), QuadCopter.vel(1:2)).*DEG2RAD;
        att_command(3) = atan2(vel_command(2), vel_command(1));
        
    end
    if mod(Index.successive_, Index.attitude_) == 0
        Index.attitude_loop_ = Index.attitude_loop_ + 1;
        CommandAttLogger.update(att_command, Index.attitude_loop_, time);

        rate_command = CtrlAtt.update(att_command, QuadCopter.att);
        
    end
    CommandOmgLogger.update(rate_command, Index.successive_+1, time);
    prop_command(2:4) = CtrlOmg.update(rate_command, QuadCopter.omg);
    CommandPropLogger.update(prop_command, Index.successive_+1, time);
    propInput.T = prop_command(1);
    propInput.Mx = prop_command(2);
    propInput.My = prop_command(3);
    propInput.Mz = prop_command(4);
    QuadCopter.set_input(propInput);
    QuadCopter.update_states();

    Index.successive_ = Index.successive_ + 1;
    time = time + 1/freq_omg;
end

