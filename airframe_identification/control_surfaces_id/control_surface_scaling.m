clc; clear all; close all;

kDEG_OFFSET = 90;

% Experiments where performed with these trim values, which give zero
% deflection angles:
trim_roll = -0.11;
trim_pitch = -0.03;
trim_yaw = -0.02;

% Max angles
aileron_max_deg = 25;
elevator_max_deg = 25;
rudder_max_deg = 22;

% Input signals before trim is given
amplitudes = [0, 0.3, 0.6, 0.9, 1.2];
N_data_points = length(amplitudes)-1;

% The ampltitudes seems to be cutoff before 1.2 amplitude,
% therefore we remove the last datapoint before LSE

% Rudder
figure
left_rudder_angle_deg = [90, 96, 103, 110, 112] - kDEG_OFFSET;
amplitudes_rudder = amplitudes - trim_yaw;

plot(amplitudes_rudder, left_rudder_angle_deg, '-o'); hold on
yline(rudder_max_deg);
xline(1);

linear_coeffs_yaw = least_squares_est(...
    [ones(1,N_data_points); amplitudes_rudder(1:end-1)], left_rudder_angle_deg(1:end-1)'...
    );

offset_rudder = linear_coeffs_yaw(1)
linear_term_rudder = linear_coeffs_yaw(2)
plot(amplitudes_rudder, offset_rudder + amplitudes_rudder * linear_term_rudder);
title("Rudder")

% Elevator
figure
left_elevator_angle_deg = [90, 97, 105, 113, 115] - kDEG_OFFSET;
amplitudes_elevator = amplitudes - trim_pitch;

plot(amplitudes_elevator, left_elevator_angle_deg, '-o'); hold on
yline(elevator_max_deg);
xline(1);

linear_coeffs_yaw = least_squares_est(...
    [ones(1,N_data_points); amplitudes_elevator(1:end-1)], left_elevator_angle_deg(1:end-1)'...
    );

offset_elevator = linear_coeffs_yaw(1)
linear_term_elevator = linear_coeffs_yaw(2)
plot(amplitudes_elevator, offset_elevator+ amplitudes_elevator * linear_term_elevator);
title("Elevator")


% Aileron
figure
left_aileron_angle_deg = [90, 99, 107, 115, 115] - kDEG_OFFSET;
amplitudes_aileron = amplitudes - trim_roll;

plot(amplitudes_aileron, left_aileron_angle_deg, '-o'); hold on
yline(aileron_max_deg);
xline(1);

linear_coeffs_yaw = least_squares_est(...
    [ones(1,N_data_points); amplitudes_aileron(1:end-1)], left_aileron_angle_deg(1:end-1)'...
    );

offset_aileron = linear_coeffs_yaw(1)
linear_term_aileron = linear_coeffs_yaw(2)
plot(amplitudes_aileron, offset_aileron + amplitudes_aileron * linear_term_aileron);
title("Aileron")

% Seems that PX4 saturates at 1.0 input


%% See how this compares with PX4 mixing etc
mixer_elevator_to_tail = 0.8; % From PX4 mixer file
mixer_rudder_to_tail = 0.7; % From PX4 mixer file
yaw_input_to_tail_deg_calc = pitch_input_to_tail_deg / mixer_elevator_to_tail * mixer_rudder_to_tail;