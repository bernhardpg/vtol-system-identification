clc; clear all; close all;

% File locations
log_location = "logs/";
log_file = "2021_04_18_flight_2_static_curves_no_thrust_211_roll";
csv_files_location = 'logs/csv/';
output_location = "static_curves/data/";

% Convert ulog to csv files in temp/ folder
% [status, commandOut] = system("rm " + temp_csv_files + "*");
% disp(commandOut);
% [status, commandOut] = system("ulog2csv " + log_location + log_file + " -o " + temp_csv_files);
% disp(commandOut);

% 2021_04_18_flight_1_static_curves.ulg
% 2021_04_18_flight_2_static_curves_no_thrust_211_roll.ulg


%% Load required log files
ekf_data = readtable(csv_files_location + log_file + '_' + "estimator_status_0" + ".csv");
angular_velocity = readtable(csv_files_location + log_file + '_' + "vehicle_angular_velocity_0" + ".csv");
actuator_controls_mr = readtable(csv_files_location + log_file + '_' + "actuator_controls_0_0" + ".csv");
actuator_controls_fw = readtable(csv_files_location + log_file + '_' + "actuator_controls_1_0" + ".csv");
input_rc = readtable(csv_files_location + log_file + '_' + "input_rc_0" + ".csv");
sensor_combined = readtable(csv_files_location + log_file + '_' + "sensor_combined_0" + ".csv");


%% Data settings
dt = 1 / 100;
t0 = ekf_data.timestamp(1) / 1e6;
t_end = ekf_data.timestamp(end) / 1e6;

t = t0:dt:t_end;
N = length(t);


%% Extract data from ekf2

t_ekf = ekf_data.timestamp / 1e6;

% q_NB = unit quaternion describing vector rotation from NED to Body. i.e.
% describes transformation from Body to NED frame.
% Note: This is the same as the output_predictor quaternion. Something is
% wrong with documentation

q0 = ekf_data.states_0_;
q1 = ekf_data.states_1_;
q2 = ekf_data.states_2_;
q3 = ekf_data.states_3_;
q_NB_raw = [q0 q1 q2 q3];

q_NB = interp1q(t_ekf, q_NB_raw, t');
% q_NB is the quat we are looking for for our state vector

eul = quat2eul(q_NB);

v_n = ekf_data.states_4_;
v_e = ekf_data.states_5_;
v_d = ekf_data.states_6_;
v_N_raw = [v_n v_e v_d];

v_N = interp1q(t_ekf, v_N_raw, t');

p_n = ekf_data.states_7_;
p_e = ekf_data.states_8_;
p_d = ekf_data.states_9_;
p_N_raw = [p_n p_e p_d];
p_N = interp1q(t_ekf, p_N_raw, t');

%% Extract angular velocities from output predictor

t_ang_vel = angular_velocity.timestamp / 1e6;

% Angular velocity around body axes
p = angular_velocity.xyz_0_;
q = angular_velocity.xyz_1_;
r = angular_velocity.xyz_2_;
w_B_raw = [p q r];

w_B = interp1q(t_ang_vel, w_B_raw, t');

%% Convert velocity to correct frames

% This is rotated with rotation matrices only for improved readability,
% as both the PX4 docs is ambiguous in describing q, and quatrotate() is
% pretty ambigious too.
q_BN = quatinv(q_NB);
R_BN = quat2rotm(q_BN);
v_B = zeros(N, 3);
for i = 1:N
   % Notice how the Rotation matrix has to be inverted here to get the
   % right result, indicating that q is in fact q_NB and not q_BN.
   v_B(i,:) = (R_BN(:,:,i) * v_N(i,:)')';
end

% Gives the same result, indicating that quatrotate() does not perform a
% simple quaternion product: q (x) v (x) q_inv
% v_B = quatrotate(q_NB, v_N);

%% Extract acclerations

t_acc = sensor_combined.timestamp / 1e6;
acc_B_raw = [sensor_combined.accelerometer_m_s2_0_ sensor_combined.accelerometer_m_s2_1_ sensor_combined.accelerometer_m_s2_2_];
acc_B = interp1q(t_ekf, acc_B_raw, t');

acc_x = acc_B(:,1);
acc_y = acc_B(:,2);
acc_z = acc_B(:,3);

%% Calculate intermediate values
% Calculate total airspeed

V = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);

% Calculate Angle of Attack

AoA = rad2deg(atan2(v_B(:,3),v_B(:,1)));

%% Extract input data
t_u_mr = actuator_controls_mr.timestamp / 1e6;
u_roll_mr = actuator_controls_mr.control_0_;
u_pitch_mr = actuator_controls_mr.control_1_;
u_yaw_mr = actuator_controls_mr.control_2_;
u_throttle_mr = actuator_controls_mr.control_3_;
u_mr_raw = [u_roll_mr u_pitch_mr u_yaw_mr u_throttle_mr];

u_mr = interp1q(t_u_mr, u_mr_raw, t');

t_u_fw = actuator_controls_fw.timestamp / 1e6;
u_roll_fw = actuator_controls_fw.control_0_;
u_pitch_fw = actuator_controls_fw.control_1_;
u_yaw_fw = actuator_controls_fw.control_2_;
u_throttle_fw = actuator_controls_fw.control_3_;
u_fw_raw = [u_roll_fw u_pitch_fw u_yaw_fw u_throttle_fw];

u_fw = interp1q(t_u_fw, u_fw_raw, t');


%% Extract times when sysid switch is flipped
% Extract RC sysid switch log
sysid_rc_switch_raw = input_rc.values_6_;
t_rc = input_rc.timestamp / 1e6;
RC_TRESHOLD = 1000;

% Find times when switch was switched
MAX_SYSID_MANEUVERS = 100;
sysid_times = zeros(MAX_SYSID_MANEUVERS,1);
sysid_maneuver_num = 1;
sysid_found = false;
for i = 1:length(t_rc)
  % Add time if found a new rising edge
  if sysid_rc_switch_raw(i) >= RC_TRESHOLD && not(sysid_found)
      sysid_times(sysid_maneuver_num) = t_rc(i);
      sysid_found = true;
      sysid_maneuver_num = sysid_maneuver_num + 1;
  end
  % If found a falling edge, start looking again
  if sysid_found && sysid_rc_switch_raw(i) < RC_TRESHOLD
     sysid_found = false; 
  end
end

if 0
    plot(t_rc, sysid_rc_switch_raw); hold on;
    plot(sysid_times, 1000, 'r*');
end

% Find corresponding indices in time vector
sysid_indices = round(interp1(t,1:N,sysid_times));

%% Aggregate data
time_before_maneuver = 0.1; %s
time_after_maneuver_start = 3.8; %s

indices_before_maneuver = time_before_maneuver / dt;
indices_after_maneuver_start = time_after_maneuver_start / dt;
maneuver_length_in_indices = indices_after_maneuver_start + indices_before_maneuver + 1;

% TODO change
maneuvers_to_aggregate = [6];
%maneuvers_to_aggregate = [6 7 8 9 11 12 13 14 16 17 20];

data_set_length = maneuver_length_in_indices * length(maneuvers_to_aggregate);

% state structure: [att ang_vel_B vel_B] = [q0 q1 q2 q3 p q r u v w]
% input structure: [top_rpm_1 top_rpm_2 top_rpm_3 top_rpm_4 aileron elevator rudder pusher_rpm]
%       = [nt1 nt2 nt3 nt4 np delta_a delta_e delta_r]
accelerations = zeros(data_set_length, 3);
input = zeros(data_set_length, 8);
state = zeros(data_set_length, 10);

curr_maneuver_aggregation_index = 1;
num_aggregated_maneuvers = 0;
for i = maneuvers_to_aggregate
    maneuver_start_index = sysid_indices(i) - indices_before_maneuver;
    maneuver_end_index = sysid_indices(i) + indices_after_maneuver_start;

    % Save data chunk to training and test sets
    maneuver_state = [
        q_NB(maneuver_start_index:maneuver_end_index,:) ...
        w_B(maneuver_start_index:maneuver_end_index,:) ...
        v_B(maneuver_start_index:maneuver_end_index,:) ...
        ];
    maneuver_input = [
        u_mr(maneuver_start_index:maneuver_end_index,:) ...% TODO translate these from moments and thrust to individual motor rpms.
        u_fw(maneuver_start_index:maneuver_end_index,:) ...% TODO same here ...
        ];
    maneuver_accelerations = [
        acc_x(maneuver_start_index:maneuver_end_index,:) ...
        acc_y(maneuver_start_index:maneuver_end_index,:) ...
        acc_z(maneuver_start_index:maneuver_end_index,:) ...
        ];
    
    curr_maneuver_aggregation_index = num_aggregated_maneuvers * maneuver_length_in_indices + 1;
    state(...
        curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + maneuver_length_in_indices - 1 ...
        ,:) = maneuver_state;
    input(...
        curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + maneuver_length_in_indices - 1 ...
        ,:) = maneuver_input;
    accelerations(...
        curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + maneuver_length_in_indices - 1 ...
        ,:) = maneuver_accelerations;
    
    num_aggregated_maneuvers = num_aggregated_maneuvers + 1;
    
    % plot data
    if 0
        t_maneuver = t(maneuver_start_index:maneuver_end_index);
        
        eul_deg = rad2deg(eul);
        
        figure
        subplot(5,1,1);
        plot(t_maneuver, eul_deg(maneuver_start_index:maneuver_end_index,:));
        legend('yaw', 'pitch','roll');
        title("attitude")

        subplot(5,1,2);
        plot(t_maneuver, w_B(maneuver_start_index:maneuver_end_index,:));
        legend('p','q','r');
        ylim([-3 3])
        title("ang vel body")

        subplot(5,1,3);
        plot(t_maneuver, v_B(maneuver_start_index:maneuver_end_index,:))
        legend('u','v','w');
        title("vel body")

        subplot(5,1,4);
        plot(t_maneuver, u_fw(maneuver_start_index:maneuver_end_index,:));
        legend('delta_a','delta_e','delta_r', 'T_fw');
        title("inputs")
    end
    
    % plot AoA, airspeed and pitch input
    if 0
        t_maneuver = t(maneuver_start_index:maneuver_end_index);
        
        figure
        
        subplot(5,1,1);
        plot(t_maneuver, u_fw(maneuver_start_index:maneuver_end_index,:));
        legend('delta_a','delta_e','delta_r', 'T_fw');
        title("inputs")

        subplot(5,1,2);
        plot(t_maneuver, [AoA(maneuver_start_index:maneuver_end_index,:) eul_deg(maneuver_start_index:maneuver_end_index,2)]);
        legend('AoA', 'pitch');
        title("Angle of Attack")

        subplot(5,1,3);
        plot(t_maneuver, V(maneuver_start_index:maneuver_end_index,:));
        legend('AoA');
        title("Airspeed (assuming no wind)")

        subplot(5,1,4);
        plot(t_maneuver, acc_x(maneuver_start_index:maneuver_end_index,:));
        legend('Acceleration');
        title("a_x")

        subplot(5,1,5);
        plot(t_maneuver, acc_z(maneuver_start_index:maneuver_end_index,:));
        legend('Acceleration');
        title("a_z")
        
        sgtitle("maneuver no: " + i)
    end
end


%% Save to file

output_data_in_table = table(state, input, accelerations);
writetable(output_data_in_table, output_location + 'output.csv');


%% Aggregate data as training and validation data
time_before_maneuver = 1.5; %s
time_after_maneuver_start = 5.5; %s
indices_before_maneuver = time_before_maneuver / dt;
indices_after_maneuver_start = time_after_maneuver_start / dt;
maneuver_length_in_indices = indices_after_maneuver_start + indices_before_maneuver + 1;

% TODO change
maneuvers_to_aggregate = 3:24;

num_total_good_maneuvers = length(maneuvers_to_aggregate);
num_training_maneuvers = ceil(num_total_good_maneuvers * 0.7);
num_test_maneuvers = floor(num_total_good_maneuvers * 0.3);

data_set_length = maneuver_length_in_indices * num_training_maneuvers;
test_set_length = maneuver_length_in_indices * num_test_maneuvers;

% state structure: [att ang_vel_B vel_B] = [q0 q1 q2 q3 p q r u v w]
state = zeros(data_set_length, 10);
test_state = zeros(test_set_length, 10);
% input structure: [top_rpm_1 top_rpm_2 top_rpm_3 top_rpm_4 aileron elevator rudder pusher_rpm]
%       = [nt1 nt2 nt3 nt4 np delta_a delta_e delta_r]
input = zeros(data_set_length, 8);
test_input = zeros(test_set_length, 8);

num_aggregated_maneuvers = 0;
num_aggregated_training_maneuvers = 0;
num_aggregated_test_maneuvers = 0;
for i = maneuvers_to_aggregate
    maneuver_start_index = sysid_indices(i) - indices_before_maneuver;
    maneuver_end_index = sysid_indices(i) + indices_after_maneuver_start;

    % Save data chunk to training and test sets
    maneuver_state = [
        q_NB(maneuver_start_index:maneuver_end_index,:) ...
        w_B(maneuver_start_index:maneuver_end_index,:) ...
        v_B(maneuver_start_index:maneuver_end_index,:) ...
        ];
    maneuver_input = [
        u_mr(maneuver_start_index:maneuver_end_index,:) ...% TODO translate these from moments and thrust to individual motor rpms.
        u_fw(maneuver_start_index:maneuver_end_index,:) ...% TODO same here ...
        ];
    
    % TODO consider shuffling the data randomly??
    % Take the first N maneuvers as training data
    if num_aggregated_training_maneuvers < num_training_maneuvers
        curr_maneuver_aggregation_index = num_aggregated_training_maneuvers * maneuver_length_in_indices + 1;
        state(...
            curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + maneuver_length_in_indices - 1 ...
            ,:) = maneuver_state;
        input(...
            curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + maneuver_length_in_indices - 1 ...
            ,:) = maneuver_input;
        
        num_aggregated_training_maneuvers = num_aggregated_training_maneuvers + 1;
    % Take the rest as test data  
    else
        curr_index_test = num_aggregated_test_maneuvers * maneuver_length_in_indices + 1;
        test_state(...
            curr_index_test:curr_index_test + maneuver_length_in_indices - 1 ...
            ,:) = maneuver_state;
        test_input(...
            curr_index_test:curr_index_test + maneuver_length_in_indices - 1 ...
            ,:) = maneuver_input;
        
        num_aggregated_test_maneuvers = num_aggregated_test_maneuvers + 1;
    end
    
    % plot data
    if 0
        t_maneuver = t(maneuver_start_index:maneuver_end_index);
        
        figure
        subplot(5,1,1);
        plot(t_maneuver, rad2deg(eul(maneuver_start_index:maneuver_end_index,:)));
        legend('yaw','pitch','roll');
        title("attitude")

        subplot(5,1,2);
        plot(t_maneuver, w_B(maneuver_start_index:maneuver_end_index,:));
        legend('p','q','r');
        ylim([-3 3])
        title("ang vel body")

        subplot(5,1,3);
        plot(t_maneuver, v_B(maneuver_start_index:maneuver_end_index,:));
        legend('u','v','w');
        title("vel body")

        subplot(5,1,4);
        plot(t_maneuver, u_fw(maneuver_start_index:maneuver_end_index,:));
        legend('delta_a','delta_e','delta_r', 'T_fw');
        title("inputs")
    end
end


%% Save to file

writematrix(state, 'aggregated_data/training_state.csv');
writematrix(input, 'aggregated_data/training_input.csv');
writematrix(test_state, 'aggregated_data/test_state.csv');
writematrix(test_input, 'aggregated_data/test_input.csv');


