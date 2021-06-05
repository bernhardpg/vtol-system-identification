% PUSHER MOTOR


clc; clear all; close all;

% File locations
data_location = "data/";

% Plot all ramp tests to find the best one
if 0
    data_file = "ramptest_";
    num_ramp_tests = 7;

    for ramp_test_index = 1:num_ramp_tests

        % Load data
        data = readtable(data_location + data_file + ramp_test_index + ".csv");

        % Read data
        t = data.Time_s_;
        esc_signal = data.ESCSignal__s_;
        thrust_kg = data.Thrust_kgf_;
        current_a = data.Current_A_;
        voltage_v = data.Voltage_V_;
        rpm_electrical = data.MotorElectricalSpeed_RPM_;
        rpm_mechanical = rpm_electrical; % It seems that the naming in the logs
        rps = rpm_mechanical / 60; % we use revolutions per second
        % is actually wrong, otherwise the motor would have a RPM of 1 Hz to
        % 8Hz. I assume that the RPM has already been divided by
        % kNUM_MAG_POLES = 14.
        power = data.ElectricalPower_W_;

        % Constants
        g = 9.81;

        % Calculate more usable physical values
        thrust_N = thrust_kg * g;

        figure
        subplot(4,1,1)
        plot(t, rps)
        title("n [rev/s]")

        subplot(4,1,2)
        plot(t, esc_signal);
        title("ESC signal")

        subplot(4,1,3)
        plot(t, thrust_kg);
        title("Thrust [kg]")

        subplot(4,1,4)
        plot(t, thrust_N);
        title("Thrust [N]")
        sgtitle("RampTest no. " + ramp_test_index)

    end
end

% Number 5 looks good
data_file = "ramptest_5.csv";

% Load data
data = readtable(data_location + data_file);

% Read data
t = data.Time_s_;
esc_signal = data.ESCSignal__s_;
thrust_kg = data.Thrust_kgf_;
current_a = data.Current_A_;
voltage_v = data.Voltage_V_;
rpm_electrical = data.MotorElectricalSpeed_RPM_;
rpm_mechanical = rpm_electrical; % It seems that the naming in the logs
rev_per_s = rpm_mechanical / 60;
% is actually wrong, otherwise the motor would have a RPM of 1 Hz to
% 8Hz. I assume that the RPM has already been divided by
% kNUM_MAG_POLES = 14.
power = data.ElectricalPower_W_;

% Constants
g = 9.81;

% Calculate more usable physical values
thrust_N = thrust_kg * g;

% Constants
rho = 1.225; % air density at sea level kg / m^3
kINCH_TO_METER = 0.0254;
prop_diam_top_in_inches = 16;
prop_diam_top = prop_diam_top_in_inches * kINCH_TO_METER;
prop_diam_pusher_in_inches = 15;
prop_diam_pusher = prop_diam_pusher_in_inches * kINCH_TO_METER;

% Least Squares Estimation
% Construct regressor vector
phi_raw = rho * prop_diam_pusher^4 * rev_per_s.^2';

% Clean data
found_start = false;
first_non_zero_index = 0;
last_non_zero_index = 0;
for i = 1:length(phi_raw)
   % Find first data sample
   if (found_start == false) && (phi_raw(i) ~= 0)
       first_non_zero_index = i;
       found_start = true;
   end
   % Found last sample
   if (found_start == true) && (phi_raw(i) == 0)
       last_non_zero_index = i - 1;
       break;
   end
end

phi = phi_raw(first_non_zero_index:last_non_zero_index);

% Output vector
y = thrust_N(first_non_zero_index:last_non_zero_index);

% Plot data points
scatter(rev_per_s(first_non_zero_index:last_non_zero_index), ...
    thrust_N(first_non_zero_index:last_non_zero_index)); hold on;

% Estimate parameters
P = (phi * phi')^-1;
B = phi * y;
theta = P * B;

% Calculate RMSE error
y_hat = phi' * theta;
squared_error = 0.5 * (y - y_hat).^2;
mean_squared_error = mean(squared_error);
RMSE = sqrt(mean_squared_error)

% Plot fit
c_T = theta

n = 0:0.1:max(rev_per_s);
estimated_thrust = c_T * rho * prop_diam_pusher^4 * n.^2;
plot(n, estimated_thrust);
xlabel("n [rev/s]");
ylabel("Thrust [N]")
title("Motor characteristics")

%%


% Follows the following format
%    pwm = scale * input + offset
min_pwm = 950; % From PX4 parameters
max_pwm = 2000; % From PX4 parameters
pusher_motor_input_to_pwm_scale = max_pwm - min_pwm;
pusher_motor_input_to_pwm_offset = min_pwm;

disp("[0,1] input to pwm [950,2000] = " + pusher_motor_input_to_pwm_offset + " + " + pusher_motor_input_to_pwm_scale + " * input");

% Calculate PWM to RPM
phi = [ones(1, length(esc_signal))
       esc_signal'];
y = rpm_mechanical;
P = inv(phi * phi');
B = phi * y;
theta = P * B;

esc_signal_test = 0:1:2000;
rpm_estimated = theta(1) + theta(2) * esc_signal_test;
scatter(esc_signal, rpm_mechanical); hold on;
plot(esc_signal_test, rpm_estimated);
xlabel("ESC signal (PWM)")
ylabel("Input (RPM)")

% Calculate input to PWM
disp("pwm_to_rpm = " + theta(1) + " + " + theta(2) + " * pwm");