close all; clear all; clc;
addpath("lib\");

sim_time = 60;
freq_vel = 20;
freq_att = 200;
freq_rate = 1000;

[gain_vel.kp, gain_vel.ki, gain_vel.kd] = deal(25, 0.001, 0.005);
[gain_att.kp, gain_att.ki, gain_att.kd] = deal(10, 0, 0);
[gain_rate.kp, gain_rate.ki, gain_rate.kd] = deal(10, 0.2, 0.1);

CtrlVel = ControllerPID(gain_vel.kp, gain_vel.ki, gain_vel.kd, 1/freq_vel);
CtrlAtt = ControllerPID(gain_att.kp, gain_att.ki, gain_att.kd, 1/freq_att);
CtrlRate = ControllerPID(gain_rate.kp, gain_rate.ki, gain_rate.kd, 1/freq_rate);

simulTime = linspace(0, sim_time, sim_time*freq_rate+1);

