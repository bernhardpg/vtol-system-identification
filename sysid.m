clc; clear all; close all;

% File locations
log_location = "logs/";
log_file = "altitude_mode_rc_flight.ulog";
temp_csv_files = 'temp/';

% Convert ulog to csv files in temp/ folder
% [status, commandOut] = system("rm " + temp_csv_files + "*");
% disp(commandOut);
% [status, commandOut] = system("ulog2csv " + log_location + log_file + " -o " + temp_csv_files);
% disp(commandOut);


%% Get output measurements from EKF2
%   State: [v_body, ang_v_body, q_attitude, flap_deflection, flap_ang_vel]
%   Outputs: [v_body, ang_v_body, q_attitude, flap_deflection, flap_ang_vel]

% Get state estimates from output predictor part of EKF2
attitude = readtable(temp_csv_files + log_file + '_' + "vehicle_attitude_0" + ".csv");
N_attitude = length(attitude.timestamp);

% attitude.timestamp(3) - attitude.timestamp(2)
% attitude.timestamp(4) - attitude.timestamp(3)
% attitude.timestamp(5) - attitude.timestamp(4)
% attitude.timestamp(6) - attitude.timestamp(5)
% attitude.timestamp(7) - attitude.timestamp(6)
freq_attitude = 1 / ((attitude.timestamp(7) - attitude.timestamp(6)) * 1e-6)

% Note: rotation from body frame to NED frame
q0_n_to_b = attitude.q_0_;
q1_n_to_b = attitude.q_1_;
q2_n_to_b = attitude.q_2_;
q3_n_to_b = attitude.q_3_;

q_n_to_b = [q0_n_to_b q1_n_to_b q2_n_to_b q3_n_to_b];
     
% Quat rotation from b to ned
q = [q0_n_to_b -q1_n_to_b -q2_n_to_b -q3_n_to_b];

local_position = readtable(temp_csv_files + log_file + '_' + "vehicle_local_position_0" + ".csv");
N_local_position = length(local_position.timestamp);
local_position.timestamp(3) - local_position.timestamp(2)
local_position.timestamp(4) - local_position.timestamp(3)
local_position.timestamp(5) - local_position.timestamp(4)
local_position.timestamp(6) - local_position.timestamp(5)
local_position.timestamp(7) - local_position.timestamp(6)
freq_local_position = 1 / ((local_position.timestamp(7) - local_position.timestamp(6)) * 1e-6)

% Note: NED frame velocity
vel_u = local_position.vx;
vel_v = local_position.vy;
vel_w = local_position.vz;
vel_ned = [vel_u vel_v vel_w];

vel = zeros(1, N_attitude);
for t = 1:N_attitude
    %vel_ned = [vel_u(t) vel_v(t) vel_w(t)];
    %vel_body = quatrotate(q(t,:), vel_ned)
    
end

angular_velocity = readtable(temp_csv_files + log_file + '_' + "vehicle_angular_velocity_0" + ".csv");
N_angular_velocity = length(angular_velocity.timestamp);
freq_ang_vel = 1 / ((angular_velocity.timestamp(7) - angular_velocity.timestamp(6)) * 1e-6)
% Angular velocities in body frame
ang_p = angular_velocity.xyz_0_;
ang_q = angular_velocity.xyz_1_;
ang_r = angular_velocity.xyz_2_;

ekf_data = readtable(temp_csv_files + log_file + '_' + "estimator_status_0" + ".csv");
freq_ekf_data = 1 / ((ekf_data.timestamp(7) - ekf_data.timestamp(6)) * 1e-6)

sensor_combined = readtable(temp_csv_files + log_file + '_' + "sensor_combined_0" + ".csv");
freq_sensor_combined = 1 / ((sensor_combined.timestamp(7) - sensor_combined.timestamp(6)) * 1e-6)