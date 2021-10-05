% PUSHER MOTOR
clc; clear all; close all;

% File locations
data_location = "data/motor_tests/pusher_motor/rcbenchmark1585/";
show_plots = false;

data_file = "ramptest";

esc_signal = [];
thrust_N = [];
torque_Nm = [];
current_a = [];
voltage_v = [];
rev_per_s = [];

for ramp_test_index = 6:8 % These are the only complete ones

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
    
    % Collect all data which is not time dependent
    esc_signal = [esc_signal; curr_esc_signal];
    thrust_N = [thrust_N; curr_thrust_N];
    torque_Nm = [torque_Nm; curr_torque_Nm];
    current_a = [current_a; curr_current_a];
    voltage_v = [voltage_v; curr_voltage_v];
    rev_per_s = [rev_per_s; curr_rps];

    if show_plots
        figure
        subplot(4,1,1)
        scatter(t, curr_rps)
        title("n [rev/s]")

        subplot(4,1,2)
        scatter(t, curr_esc_signal);
        title("ESC signal")

        subplot(4,1,3)
        scatter(t, curr_thrust_kg);
        title("Thrust [kg]")

        subplot(4,1,4)
        scatter(t, curr_thrust_N);
        title("Thrust [N]")
        sgtitle("RampTest no. " + ramp_test_index)
    end

end

% Constants
rho = 1.225; % air density at sea level kg / m^3
kINCH_TO_METER = 0.0254;
% prop_diam_top_in_inches = 16;
% prop_diam_top = prop_diam_top_in_inches * kINCH_TO_METER;
prop_diam_pusher_in_inches = 15;
prop_diam_pusher = prop_diam_pusher_in_inches * kINCH_TO_METER;

% Least Squares Estimation for thrust
phi = rho * prop_diam_pusher^4 * rev_per_s.^2';
y = thrust_N;
[theta, RMSE] = LSE(phi, y);

% Plot fit
c_T = theta;

subplot(1,2,1)
n = 0:0.1:max(rev_per_s);
estimated_thrust = c_T * rho * prop_diam_pusher^4 * n.^2;
scatter(rev_per_s, thrust_N); hold on;
plot(n, estimated_thrust);
xlabel("n [rev/s]");
ylabel("Thrust [N]")
title("c_T = " + c_T)

% Least Squares Estimation for torque
phi = rho * prop_diam_pusher^5 * rev_per_s.^2';
y = torque_Nm;
[theta, RMSE] = LSE(phi, y);

% Plot fit
c_Q = theta;

subplot(1,2,2)
n = 0:0.1:max(rev_per_s);
estimated_torque = c_Q * rho * prop_diam_pusher^5 * n.^2;
scatter(rev_per_s, torque_Nm); hold on;
plot(n, estimated_torque);
xlabel("n [rev/s]");
ylabel("Torque [Nm]")
title("c_Q = " + c_T)
sgtitle("Pusher motor characteristics")

%% Calculate conversion from PWM to rps

% From PWM to rps
cleaned_esc_signal = esc_signal(esc_signal>1200);
cleaned_rev_per_s = rev_per_s(esc_signal>1200);
y = rev_per_s(esc_signal>1200);
%phi = [ones(length(cleaned_esc_signal),1) cleaned_esc_signal]';
%[theta, RMSE1] = LSE(phi, y); % Better to use quadratic fit than linear,
% by looking at RMSE1 vs RMSE2

phi = [ones(length(cleaned_esc_signal),1) cleaned_esc_signal cleaned_esc_signal.^2]';
[theta, RMSE2] = LSE(phi, y);

pwm_test = 1000:0.1:2200;
predicted_y = theta(1) + theta(2) * pwm_test + theta(3) * pwm_test.^2;

figure
plot(pwm_test, predicted_y); hold on
scatter(cleaned_esc_signal, cleaned_rev_per_s);
xlabel("ESC signal [PWM]")
ylabel("Input [rev/s]")

% Follows the following format
%    pwm = scale * input + offset
min_pwm = 950; % From PX4 parameters
max_pwm = 2000; % From PX4 parameters
pusher_motor_input_to_pwm_scale = max_pwm - min_pwm;
pusher_motor_input_to_pwm_offset = min_pwm;

disp("[0,1] input to pwm [950,2000] = " + pusher_motor_input_to_pwm_offset + " + " + pusher_motor_input_to_pwm_scale + " * input");

% Calculate input to PWM
disp("pwm_to_rpm = " + theta(1) + " + " + theta(2) + " * pwm" + theta(3) + " * pwm^2");

%%
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