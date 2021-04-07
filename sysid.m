clc; clear all; close all;

% File locations
log_location = "logs/";
log_file = "day_2";
temp_csv_files = 'temp/';

% Convert ulog to csv files in temp/ folder
% [status, commandOut] = system("rm " + temp_csv_files + "*");
% disp(commandOut);
% [status, commandOut] = system("ulog2csv " + log_location + log_file + " -o " + temp_csv_files);
% disp(commandOut);


%% Load required log files
ekf_data = readtable(temp_csv_files + log_file + '_' + "estimator_status_0" + ".csv");
angular_velocity = readtable(temp_csv_files + log_file + '_' + "vehicle_angular_velocity_0" + ".csv");
actuator_controls_mr = readtable(temp_csv_files + log_file + '_' + "actuator_controls_0_0" + ".csv");
actuator_controls_fw = readtable(temp_csv_files + log_file + '_' + "actuator_controls_1_0" + ".csv");
input_rc = readtable(temp_csv_files + log_file + '_' + "input_rc_0" + ".csv");
sensor_combined = readtable(temp_csv_files + log_file + '_' + "sensor_combined_0" + ".csv");


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

w_B = interp1q(t_ekf, w_B_raw, t');

%% Convert data to correct frames

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

%% Calculate total airspeed

V = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);

%% Calculate Angle of Attack

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
sysid_switch = input_rc.values_6_;
t_rc = input_rc.timestamp / 1e6;

sysid_times = zeros(100,1);
sysid_maneuver_num = 1;
sysid_found = false;
for i = 1:length(t_rc)
  % Add time if found a new rising edge
  if sysid_switch(i) > 1900 && not(sysid_found)
      sysid_times(sysid_maneuver_num) = t_rc(i);
      sysid_found = true;
      sysid_maneuver_num = sysid_maneuver_num + 1;
  end
  % If found a falling edge, start looking again
  if sysid_found && sysid_switch(i) < 1900
     sysid_found = false; 
  end
end

if 0
    plot(t_rc, sysid_switch); hold on;
    plot(sysid_times, 1000, 'r*');
end

%% Plot states at input switch
indices_before_maneuver = 2; % seconds
indices_after_maneuver_start = 8; % seconds
dt = 1 / 100; % 100 hz

for i = 30:30
    start_index = sysid_times(i) - indices_before_maneuver;
    t = start_index:dt:start_index + indices_after_maneuver_start;
    
    % States
    q_NB_interpolated = interp1q(t_ekf, q_NB, t');
    eul = quat2eul(q_NB_interpolated);
    v_B_interpolated = interp1q(t_ekf, v_B, t');
    w_B_interpolated = interp1q(t_ang_vel, w_B, t');
    
    figure
    subplot(4,1,1);
    plot(t, rad2deg(eul));
    legend('yaw','pitch','roll');
    title("attitude")
    
    subplot(4,1,2);
    plot(t, w_B_interpolated);
    legend('w_x','w_y','w_z');
    title("ang vel body")
    
    subplot(4,1,3);
    plot(t, v_B_interpolated);
    legend('v_x','v_y','v_z');
    title("vel body")
    
    % Inputs
    u_fw_interpolated = interp1q(t_u_fw, u_fw, t');
       
    subplot(4,1,4);
    plot(t, u_fw_interpolated);
    legend('delta_a','delta_e','delta_r', 'T_fw');
    title("inputs")
    
    AoA_intepolated = rad2deg(atan2(v_B_interpolated(:,3),v_B_interpolated(:,1)));
    figure
    plot(t, AoA_intepolated)
    title("Angle of Attack")
end




%% STATIC CURVES
%%%%%%%%

%% Model parameters

% TODO fix these
m = 5.6+3*1.27+2*0.951; % kg 

g = 9.81; % m/s^2
rho = 1.225; % kg / m^3
b = 2.5;
S = 0.75; % TODO: not exact
AR = b^2 / S;

%% Extract aerodynamic forces

t_acc = sensor_combined.timestamp / 1e6;
acc_B_raw = [sensor_combined.accelerometer_m_s2_0_ sensor_combined.accelerometer_m_s2_1_ sensor_combined.accelerometer_m_s2_2_];
acc_B = interp1q(t_ekf, acc_B_raw, t');

F_x = acc_B(:,1) / m;
F_z = acc_B(:,3) / m;

L = zeros(N,1);
D = zeros(N,1);

for i = 1:N
   alpha = deg2rad(AoA(i));
   R_BS = [cos(alpha) -sin(alpha);
           sin(alpha) cos(alpha)];
   R_SB = inv(R_BS);
   F_B = [F_x(i); F_z(i)]; % Body frame forces
   F_S = R_SB * F_B; % Rotated to stability frame
   D(i) = -F_S(1);
   L(i) = -F_S(2);
end

dynamic_pressure = 0.5 * rho * V.^2;
c_L = L ./ dynamic_pressure;
c_D = D ./ dynamic_pressure;
 
%% Find times for static curves

static_sysid_indices = zeros(100,1);
static_sysid_maneuver_num = 1;
static_sysid_found = false;
for i = 1:N
  % Add time if found a new rising edge
  if u_fw(i,2) >= 1 && not(static_sysid_found)
      static_sysid_indices(static_sysid_maneuver_num) = i;
      static_sysid_found = true;
      static_sysid_maneuver_num = static_sysid_maneuver_num + 1;
  end
  % If found a falling edge, start looking again
  if static_sysid_found && u_fw(i,2) < 1
     static_sysid_found = false; 
  end
end

%% Plot static curves
indices_before_maneuver = -2 / dt;
indices_after_maneuver_start = 5 / dt;
maneuver_N = maneuver_end_index - maneuver_start_index + 1;

% 5: good 6, 2
% 5: good -12, 16. Some SD card error. However, the thrust is here.
% 6: useless
% 7: useless, data dropout
% 8: useless, strange data. Lift goes down with increasing AoA
% 9 is useless
% 10: good 4, 1
% 10: good, -2 5

for i = 5:5
    maneuver_start_index = static_sysid_indices(i) - indices_before_maneuver;
    maneuver_end_index = static_sysid_indices(i) + indices_after_maneuver_start;
    t_maneuver = t(maneuver_start_index:maneuver_end_index);
    
    % Lift coefficient
    % Construct regressors
    phi = [ones(maneuver_N,1) AoA(maneuver_start_index:maneuver_end_index)]';
    % Least Squares Estimation
    P = (phi * phi')^-1;
    theta = P * phi * c_L(maneuver_start_index:maneuver_end_index);
    % Extract coefficients
    c_L_0 = theta(1);
    c_L_alpha = theta(2);
    
    c_L_hat = c_L_0 + c_L_alpha * AoA(maneuver_start_index:maneuver_end_index);
    
    % Drag coefficient
    % LSE
    phi = [ones(maneuver_N,1) c_L_hat.^2 / (AR * pi)]';
    P = (phi * phi')^-1;
    theta = P * phi * c_D(maneuver_start_index:maneuver_end_index);
    c_D_p = theta(1);
    e = 1/theta(2); % Oswalds efficiency factor
    
    c_D_hat = c_D_p + c_L_hat.^2 / (pi * e * AR);
   
    % Plotting
    figure
    subplot(7,1,1);
    plot(t_maneuver, rad2deg(eul(maneuver_start_index:maneuver_end_index,2:3)));
    legend('pitch','roll');
    title("attitude")

    subplot(7,1,2);
    plot(t_maneuver, w_B(maneuver_start_index:maneuver_end_index,:));
    legend('w_x','w_y','w_z');
    title("ang vel body")

    subplot(7,1,3);
    plot(t_maneuver, v_B(maneuver_start_index:maneuver_end_index,:));
    legend('v_x', 'v_y', 'v_z');
    title("vel body")

    subplot(7,1,4);
    plot(t_maneuver, u_fw(maneuver_start_index:maneuver_end_index,:));
    legend('delta_a','delta_e','delta_r', 'T_fw');
    title("inputs")

    subplot(7,1,5);
    plot(t_maneuver, AoA(maneuver_start_index:maneuver_end_index));
    title("Angle of Attack")

    subplot(7,1,6);
    plot(t_maneuver, L(maneuver_start_index:maneuver_end_index));
    legend('L');
    title("Lift force")
    
    subplot(7,1,7);
    plot(t_maneuver, D(maneuver_start_index:maneuver_end_index));
    legend('D');
    title("Drag force")
    
    figure
    subplot(2,1,1);
    plot(AoA(maneuver_start_index:maneuver_end_index), c_L_hat); hold on;
    scatter(AoA(maneuver_start_index:maneuver_end_index), c_L(maneuver_start_index:maneuver_end_index)); hold on;
    xlabel("AoA")
    ylabel("c_L")
    ylim([0 max(c_L(maneuver_start_index:maneuver_end_index))*1.2])
    
    subplot(2,1,2);
    plot(AoA(maneuver_start_index:maneuver_end_index), c_D_hat); hold on;
    scatter(AoA(maneuver_start_index:maneuver_end_index), c_D(maneuver_start_index:maneuver_end_index)); hold on;
    xlabel("AoA")
    ylabel("c_D")
    ylim([min([0 c_D(maneuver_start_index:maneuver_end_index)']) max(c_D(maneuver_start_index:maneuver_end_index))*1.2])
end




