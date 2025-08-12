close all; clear all; clc;
addpath('lib\');

%%
dt = 0.01;
ambient_wind = [3; 0; 0];

sim_time = 0 : dt : 60 - dt;
Dryden = WindDryden(dt, ambient_wind);
WindLog = Logger(3, length(sim_time));
wind_Va = norm(ambient_wind);

for i = 1:length(sim_time)
    Ww = Dryden.update_wind(50, wind_Va, [0, 0, 0]);
    wind_Va = norm(Ww);
    WindLog.update(Ww, i, sim_time(i));
end

WindPlot = figure();
grid on; hold on;
plot(WindLog.time, WindLog.log);
xlabel('Time (s)');
ylabel('Wind Velocity (m/s)');
title('Wind Velocity over Time');