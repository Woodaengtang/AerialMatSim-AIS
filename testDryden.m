close all; clear all; clc;
addpath('lib\');

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

height = linspace(0, 1000, 5000);
zero2end = Dryden.model_params.alt_e - Dryden.model_params.alt_0;
grad_L_u = (Dryden.model_params.L_u_e - Dryden.model_params.L_u_0)/zero2end;
grad_L_v = (Dryden.model_params.L_v_e - Dryden.model_params.L_v_0)/zero2end;
grad_L_w = (Dryden.model_params.L_w_e - Dryden.model_params.L_w_0)/zero2end;
grad_sig_l_u = (Dryden.model_params.sig_u_l_e - Dryden.model_params.sig_u_l_0)/zero2end;
grad_sig_l_v = (Dryden.model_params.sig_v_l_e - Dryden.model_params.sig_v_l_0)/zero2end;
grad_sig_l_w = (Dryden.model_params.sig_w_l_e - Dryden.model_params.sig_w_l_0)/zero2end;
grad_sig_m_u = (Dryden.model_params.sig_u_m_e - Dryden.model_params.sig_u_m_0)/zero2end;
grad_sig_m_v = (Dryden.model_params.sig_v_m_e - Dryden.model_params.sig_v_m_0)/zero2end;
grad_sig_m_w = (Dryden.model_params.sig_w_m_e - Dryden.model_params.sig_w_m_0)/zero2end;


WindPlot = figure();
WindPlot.Theme = 'light';
sgtitle('Wind Velocity Components'); % Title for the WindPlot figure
subplot(3, 1, 1);
grid on; hold on;
plot(WindLog.time, WindLog.log(1, :));
ylabel('u_w(m/s)');
legend('u_w');
subplot(3, 1, 2);
grid on; hold on;
plot(WindLog.time, WindLog.log(2, :));
ylabel('v_w(m/s)');
legend('v_w');
subplot(3, 1, 3);
grid on; hold on;
plot(WindLog.time, WindLog.log(3, :));
xlabel('Time (s)'); ylabel('w_w(m/s)');
legend('w_w');

plotCombinedParams = figure();
plotCombinedParams.Theme = 'light';
sgtitle('Linearly Interpolated Dryden Model Parameters');
subplot(6, 1, 1);
grid on; hold on;
plot(height, Dryden.model_params.L_u_0 + grad_L_u * height, 'DisplayName', 'L_u');
ylabel('L_u (m)');
subplot(6, 1, 2);
grid on; hold on;
plot(height, Dryden.model_params.L_v_0 + grad_L_v * height, 'DisplayName', 'L_v');
ylabel('L_v (m)');
subplot(6, 1, 3);
grid on; hold on;
plot(height, Dryden.model_params.L_w_0 + grad_L_w * height, 'DisplayName', 'L_w');
ylabel('L_w (m)');
subplot(6, 1, 4);
grid on; hold on;
plot(height, Dryden.model_params.sig_u_m_0 + grad_sig_m_u * height, 'DisplayName', 'sig_u_l');
ylabel('\sigma_u (m/s)');
subplot(6, 1, 5);
grid on; hold on;
plot(height, Dryden.model_params.sig_v_m_0 + grad_sig_m_v * height, 'DisplayName', 'sig_v_l');
ylabel('\sigma_v (m/s)');
subplot(6, 1, 6);
grid on; hold on;
plot(height, Dryden.model_params.sig_w_m_0 + grad_sig_m_w * height, 'DisplayName', 'sig_w_l');
ylabel('\sigma_w (m/s)');
xlabel('Height (m)');
