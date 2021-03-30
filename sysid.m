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

q_NB = [q0 q1 q2 q3];
eul = quat2eul(q_NB);

if 0
    figure
    subplot(3,1,1)
    plot(t_ekf, eul(:,1))
    title("roll")
    subplot(3,1,2)
    plot(t_ekf, eul(:,2))
    title("pitch")
    subplot(3,1,3)
    plot(t_ekf, eul(:,3))
    title("yaw")
    sgtitle("vehicle attitude")
end

v_n = ekf_data.states_4_;
v_e = ekf_data.states_5_;
v_d = ekf_data.states_6_;

v_N = [v_n v_e v_d];

p_n = ekf_data.states_7_;
p_e = ekf_data.states_8_;
p_d = ekf_data.states_9_;

% Plot position
if 0
    plot3(p_n, p_e, -p_d);
end

% Plot wind data
if 0
    w_n = ekf_data.states_22_;
    w_e = ekf_data.states_23_;

    wind = readtable(temp_csv_files + log_file + '_' + "wind_estimate_0" + ".csv");
    w_n = wind.windspeed_north;
    w_e = wind.windspeed_east;
    t = wind.timestamp / 1e6;

    subplot(2,1,1);
    plot(t, w_n);
    subplot(2,1,2);
    plot(t, w_e);
end

%% Extract angular velocities from output predictor

% Angular velocity around body axes
p = angular_velocity.xyz_0_;
q = angular_velocity.xyz_1_;
r = angular_velocity.xyz_2_;
w_B = [p q r];
t_ang_vel = angular_velocity.timestamp / 1e6;

if 0
    subplot(3,1,1)
    plot(t_ang_vel, p);
    title("p");
    subplot(3,1,2)
    plot(t_ang_vel, q);
    title("q");
    subplot(3,1,3)
    plot(t_ang_vel, r);
    title("r");
    sgtitle("Angular velocities [rad/s]")
end

%% Convert data to correct frames
% Desired data:
%   State: [v_body, ang_v_body, q_attitude, flap_deflection, flap_ang_vel]
%   Outputs: [v_body, ang_v_body, q_attitude, flap_deflection, flap_ang_vel]

% This is rotated with rotation matrices only for improved readability,
% as both the PX4 docs is ambiguous in describing q, and quatrotate() is
% pretty ambigious too.
q_BN = quatinv(q_NB);
R_BN = quat2rotm(q_BN);
v_B = zeros(length(v_N), 3);
for i=1:length(R_BN)
   % Notice how the Rotation matrix has to be inverted here to get the
   % right result, indicating that q is in fact q_NB and not q_BN.
   v_B(i,:) = (R_BN(:,:,i) * v_N(i,:)')';
end

% q_NB is the quat we are looking for for our state vector

% Gives the same result, indicating that quatrotate() does not perform a
% simple quaternion product: q (x) v (x) q_inv
%v_B = quatrotate(q_NB, v_N);

if 0
    subplot(3,1,1)
    plot(t_ekf, v_B(:,1))
    title("v_x")
    subplot(3,1,2)
    plot(t_ekf, v_B(:,2))
    title("v_y")
    subplot(3,1,3)
    plot(t_ekf, v_B(:,3))
    title("v_z")
    sgtitle("Body velocities")
end

%% Calculate Angle of Attack

AoA = rad2deg(atan2(v_B(:,3),v_B(:,1)));


%% Extract input data
t_u_mr = actuator_controls_mr.timestamp / 1e6;
u_roll_mr = actuator_controls_mr.control_0_;
u_pitch_mr = actuator_controls_mr.control_1_;
u_yaw_mr = actuator_controls_mr.control_2_;
u_throttle_mr = actuator_controls_mr.control_3_;

t_u_fw = actuator_controls_fw.timestamp / 1e6;
u_roll_fw = actuator_controls_fw.control_0_;
u_pitch_fw = actuator_controls_fw.control_1_;
u_yaw_fw = actuator_controls_fw.control_2_;
u_throttle_fw = actuator_controls_fw.control_3_;
u_fw = [u_roll_fw u_pitch_fw u_yaw_fw u_throttle_fw];

% Todo: convert inputs to actual deflection angle and RPM

%% Extract acceleration data
t_acc = sensor_combined.timestamp / 1e6;
acc_B = [sensor_combined.accelerometer_m_s2_0_ sensor_combined.accelerometer_m_s2_1_ sensor_combined.accelerometer_m_s2_2_];

F_z = acc_B(:,3);
F_x = acc_B(:,1);
 
%% Find times for static curves

static_sysid_times = zeros(100,1);
static_sysid_maneuver_num = 1;
static_sysid_found = false;
for i = 1:length(t_u_fw)
  % Add time if found a new rising edge
  if u_pitch_fw(i) >= 1 && not(static_sysid_found)
      static_sysid_times(static_sysid_maneuver_num) = t_u_fw(i);
      static_sysid_found = true;
      static_sysid_maneuver_num = static_sysid_maneuver_num + 1;
  end
  % If found a falling edge, start looking again
  if static_sysid_found && u_pitch_fw(i) < 1
     static_sysid_found = false; 
  end
end


if 0
    plot(t_u_fw, u_pitch_fw); hold on;
    plot(static_sysid_times, 0, 'r*');
end

%% Plot static curves
maneuver_before = 10; % seconds
maneuver_duration = 20; % seconds
dt = 1 / 100; % 100 hz

m = 5.6+3*1.27+2*0.951;
g = 9.81;
rho = 1.225;
V = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);

for i = 5:10
    start_time = static_sysid_times(i) - maneuver_before;
    t = start_time:dt:start_time + maneuver_duration;
    
    % States
    q_NB_interpolated = interp1q(t_ekf, q_NB, t');
    eul = quat2eul(q_NB_interpolated);
    V_interpolated = interp1q(t_ekf, V, t');
    w_B_interpolated = interp1q(t_ang_vel, w_B, t');
    
    % Total airspeed
    v_B_interpolated = interp1q(t_ekf, v_B, t');
    
    % Acceleration
    a_B_interpolated = interp1q(t_acc, acc_B, t');
    L_B_interpolated = interp1q(t_acc, L_B, t');
    
    % Angle of attack
    AoA_interpolated = rad2deg(atan2(v_B_interpolated(:,3),v_B_interpolated(:,1)));
    
    % Construct regressors
    dynamic_pressure = 0.5 * rho * V_interpolated.^2;
    c_L_measured = L_B_interpolated ./ dynamic_pressure;
    phi = [ones(length(AoA_interpolated),1) AoA_interpolated]';
    
    % Least Squares Estimation
    P = (phi * phi')^-1;
    theta = P * phi * c_L_measured;

    % Extract coefficients
    c_L_0 = theta(1);
    c_L_alpha = theta(2);
    
    c_L_hat = c_L_0 + c_L_alpha * AoA_interpolated;
    plot(AoA_interpolated, c_L_hat); hold on;
    scatter(AoA_interpolated, c_L_measured); hold on;
    xlabel("AoA")
    ylabel("c_L")
       
    % Plotting
    if 0
        figure
        subplot(6,1,1);
        plot(t, rad2deg(eul(:,2:3)));
        legend('pitch','roll');
        title("attitude")

        subplot(6,1,2);
        plot(t, w_B_interpolated);
        legend('w_x','w_y','w_z');
        title("ang vel body")

        subplot(6,1,3);
        plot(t, V_interpolated);
        legend('V');
        title("Airspeed")

        % Inputs
        u_fw_interpolated = interp1q(t_u_fw, u_fw, t');

        subplot(6,1,4);
        plot(t, u_fw_interpolated);
        legend('delta_a','delta_e','delta_r', 'T_fw');
        title("inputs")

        subplot(6,1,5);
        plot(t, AoA_interpolated)
        title("Angle of Attack")

        subplot(6,1,6);
        plot(t, L_B_interpolated);
        legend('L');
        title("Lift force")
    end
    
end




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
maneuver_before = 2; % seconds
maneuver_duration = 8; % seconds
dt = 1 / 100; % 100 hz

for i = 30:30
    start_time = sysid_times(i) - maneuver_before;
    t = start_time:dt:start_time + maneuver_duration;
    
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
