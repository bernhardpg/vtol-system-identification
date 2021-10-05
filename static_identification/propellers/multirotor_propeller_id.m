% PUSHER MOTOR
clc; clear all; close all;

% File locations
%data_location = "data/motor_tests/pusher_motor/rcbenchmark1585/";
data_location = "data/motor_tests/multirotor_motor/";
show_plots = true;

data_file = "ramptest";

% import plot settings
plot_settings;

esc_signal = [];
thrust_N = [];
torque_Nm = [];
current_a = [];
voltage_v = [];
rev_per_s = [];

ramp_test_index = 7; % This one looks good!

% Load data
data = readtable(data_location + data_file + ramp_test_index + ".csv");

% Read data
t = data.Time_s_;
curr_esc_signal = data.ESCSignal__s_;
curr_thrust_kg = data.Thrust_kgf_;
curr_torque_Nm = data.Torque_N_m_;
curr_current_a = data.Current_A_;
curr_voltage_v = data.Voltage_V_;
curr_rpm_electrical = data.MotorElectricalSpeed_RPM_; % unused
curr_rpm_optical = data.MotorOpticalSpeed_RPM_;
curr_rps = curr_rpm_optical / 60; % we use revolutions per second
curr_electrical_power = data.ElectricalPower_W_;
curr_mechanical_power = data.MechanicalPower_W_;

% Calculate more usable physical values
g = 9.81;
curr_thrust_N = curr_thrust_kg * g;

% Cut away dropout data
start_index = 29;
end_index = 151;
esc_signal = curr_esc_signal(start_index:end_index);
thrust_N = curr_thrust_N(start_index:end_index);
torque_Nm = curr_torque_Nm(start_index:end_index);
current_a = curr_current_a(start_index:end_index);
voltage_v = curr_voltage_v(start_index:end_index);
rev_per_s = curr_rps(start_index:end_index);

if show_plots
    figure
    subplot(4,1,1)
    scatter(1:length(rev_per_s), rev_per_s)
    title("n [rev}/s]")

    subplot(4,1,2)
    scatter(1:length(rev_per_s), esc_signal);
    title("ESC signal")

    subplot(4,1,4)
    scatter(1:length(rev_per_s), thrust_N);
    title("Thrust [N]")
    sgtitle("RampTest no. " + ramp_test_index)
end



%%
% Constants
rho = 1.225; % air density at sea level kg / m^3
kINCH_TO_METER = 0.0254;
prop_diam_top_in_inches = 16;
prop_diam_top = prop_diam_top_in_inches * kINCH_TO_METER;


% Least Squares Estimation for thrust
phi = rho * prop_diam_top^4 * rev_per_s.^2';
y = thrust_N;
[theta, RMSE] = LSE(phi, y);

% Plot fit
c_T = theta;
disp("c_T: " + c_T);

fig = figure;
fig.Position = [100 100 800 400];

subplot(1,2,1)
n = 0:0.1:max(rev_per_s);
estimated_thrust = c_T * rho * prop_diam_top^4 * n.^2;
scatter(rev_per_s, thrust_N); hold on;
plot(n, estimated_thrust);
xlabel("n [rev/s]", 'interpreter','Latex','FontSize',font_size_small);
ylabel("[N]", 'interpreter','Latex','FontSize',font_size_small)
title("Thrust", 'interpreter','Latex','FontSize',font_size)

% Least Squares Estimation for torque
phi = rho * prop_diam_top^5 * rev_per_s.^2';
y = torque_Nm;
[theta, RMSE] = LSE(phi, y);

% Plot fit
c_Q = theta;
disp("c_Q: " + c_Q);

subplot(1,2,2)
n = 0:0.1:max(rev_per_s);
estimated_torque = c_Q * rho * prop_diam_top^5 * n.^2;
scatter(rev_per_s, torque_Nm); hold on;
plot(n, estimated_torque);
legend("Measured","Model", 'interpreter','Latex','FontSize',font_size);
xlabel("n [rev/s]", 'interpreter','Latex','FontSize',font_size_small);
ylabel("[Nm]", 'interpreter','Latex','FontSize',font_size_small)
title("Torque", 'interpreter','Latex','FontSize',font_size)
sgtitle("Multirotor Propeller", 'interpreter','Latex','FontSize',font_size_large)

function [theta, RMSE] = LSE(phi, y)
    % Estimate parameters
    P = (phi * phi')^-1;
    B = phi * y;
    theta = P * B;

    % Calculate RMSE error
    y_hat = phi' * theta;
    squared_error = 0.5 * (y - y_hat).^2;
    mean_squared_error = mean(squared_error);
    RMSE = sqrt(mean_squared_error);
end