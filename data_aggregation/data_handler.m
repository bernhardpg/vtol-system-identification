clc; clear all; close all;

% File location
log_file = "06_31_21";
csv_files_location = 'data/logs/csv/';
csv_log_file_location = csv_files_location + log_file;

% Output data
save_output_data = false;
save_plot = true;
show_plot = false;
plot_output_location = "data/plots/static_stability/00_full_maneuvers/experiment_2/";
data_output_location = "data/static_stability/experiment_2/";

% Set common data time resolution
dt = 1/100;

% Load aircraft parameters
aircraft_properties;

% Read data
[t, state, input] = read_state_and_input_from_log(csv_log_file_location, dt);
q_NB = state(:,1:4);
w_B = state(:,5:7);
v_B = state(:,8:10);
u_mr = input(:,1:4);
u_fw = input(:,5:8);
sysid_indices = get_sysid_indices(csv_log_file_location, t);
num_maneuvers = length(sysid_indices);
[acc_B_unfiltered, acc_B_raw, t_acc_raw, bias_acc] = read_accelerations(csv_log_file_location, t);

%%

generate_plot_of_all_maneuvers(t, dt, acc_B_raw, t_acc_raw, acc_B_unfiltered, state, bias_acc, input, sysid_indices, mass_kg, g, show_plot, save_plot, plot_output_location)


%%



filepath = 'data/metadata.csv';
%[maneuvers_to_aggregate, metadata] = read_metadata(filepath);
maneuvers_to_aggregate = 1:20;

% Cutoff frequency for acceleration data
f_cutoff = 25;

% Process data
[acc_B] = filter_accelerations(t, acc_B_raw, t_acc_raw, f_cutoff);
acc_N = calculate_acc_N(acc_B, q_NB);

% Calculate Angle of Attack
AoA_rad = atan2(v_B(:,3),v_B(:,1));
AoA_deg = rad2deg(AoA_rad);

% Calculate lift and drag
v_A = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);
[L, D, c_L, c_D] = calculate_lift_and_drag(state, input, AoA_rad, v_A, acc_B, mass_kg, g);

%%%
% Aggregate data
% Set aggregation parameters
maneuver_trim_start = -1; %s
maneuver_trim_end = -1; %s
sweep_maneuver_length = 16; % s
sweep_maneuver_length_in_indices = round((sweep_maneuver_length - maneuver_trim_start - maneuver_trim_end) / dt);

% Initialize empty output variables
maneuver_start_indices = [];
t_aggregated = [];
input_aggregated = [];
state_aggregated = [];
c_D_aggregated = [];
c_L_aggregated = [];
AoA_aggregated = [];

% Iterate through maneuvers
num_aggregated_maneuvers = 0;
for i = maneuvers_to_aggregate
    maneuver_metadata = metadata(string(i));
    maneuver_start_index = round((maneuver_metadata(1) - t(1)) / dt);
    maneuver_start_indices = [maneuver_start_indices maneuver_start_index];
    maneuver_end_index = round((maneuver_metadata(2) - t(1)) / dt);

    % Extract only maneuver data
    t_maneuver = t(maneuver_start_index:maneuver_end_index)';
    state_maneuver = state(maneuver_start_index:maneuver_end_index,:);
    input_maneuver = input(maneuver_start_index:maneuver_end_index,:);
    AoA_deg_maneuver = AoA_deg(maneuver_start_index:maneuver_end_index);
    AoA_rad_maneuver = AoA_rad(maneuver_start_index:maneuver_end_index);
    acc_B_maneuver = acc_B(maneuver_start_index:maneuver_end_index,:);
    acc_B_unfiltered_maneuver = acc_B_unfiltered(maneuver_start_index:maneuver_end_index,:);
    acc_N_maneuver = acc_N(maneuver_start_index:maneuver_end_index,:);
    bias_acc_maneuver = bias_acc(maneuver_start_index:maneuver_end_index);
    L_maneuver = L(maneuver_start_index:maneuver_end_index);
    D_maneuver = D(maneuver_start_index:maneuver_end_index);
    c_L_maneuver = c_L(maneuver_start_index:maneuver_end_index);
    c_D_maneuver = c_D(maneuver_start_index:maneuver_end_index);

    % Plot data
    plot_trajectory(i, t_maneuver, state_maneuver, input_maneuver, show_plot, save_plot, plot_output_location);
    plot_trajectory_static_details(i, t_maneuver, state_maneuver, input_maneuver,...
        AoA_deg_maneuver, acc_B_unfiltered_maneuver, acc_B_maneuver, bias_acc_maneuver,...
        acc_N_maneuver, show_plot, save_plot, plot_output_location);
    plot_scatter_lift_drag(i, t_maneuver, AoA_deg_maneuver, L_maneuver, D_maneuver, c_L_maneuver, c_D_maneuver, show_plot, save_plot, plot_output_location);

    % Store data in aggregated data matrices
    t_aggregated = [t_aggregated;
                    t_maneuver];
    state_aggregated = [state_aggregated;
                        state_maneuver];
    input_aggregated = [input_aggregated;
                        input_maneuver];
    c_L_aggregated = [c_L_aggregated;
                      c_L_maneuver];
    c_D_aggregated = [c_D_aggregated;
                      c_D_maneuver];
    AoA_aggregated = [AoA_aggregated;
                      AoA_rad_maneuver];

    num_aggregated_maneuvers = num_aggregated_maneuvers + 1;

end

disp("Succesfully aggregated " + num_aggregated_maneuvers + " maneuvers");
disp(maneuvers_to_aggregate);

% Save to files
if save_output_data
    writematrix(state_aggregated, data_output_location + 'state.csv');
    writematrix(input_aggregated, data_output_location + 'input.csv');
    writematrix(t_aggregated, data_output_location + 't.csv');
    writematrix(AoA_aggregated, data_output_location + 'aoa_rad.csv');
    writematrix(c_D_aggregated, data_output_location + 'cd.csv');
    writematrix(c_L_aggregated, data_output_location + 'cl.csv');
    writematrix(dt, data_output_location + 'dt.csv');
    writematrix(maneuver_start_indices, data_output_location + 'maneuver_start_indices.csv');
end


function [] = generate_plot_of_all_maneuvers(t, dt, acc_B_raw, t_acc_raw, acc_B_unfiltered, state, bias_acc, input, sysid_indices, mass_kg, g, show_plot, save_plot, plot_output_location)
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);


    maneuvers_to_aggregate = 1:length(sysid_indices);
    
    % Cutoff frequency for acceleration data
    f_cutoff = 25;

    % Process data
    [acc_B] = filter_accelerations(t, acc_B_raw, t_acc_raw, f_cutoff);
    acc_N = calculate_acc_N(acc_B, q_NB);

    % Calculate Angle of Attack
    AoA_rad = atan2(v_B(:,3),v_B(:,1));
    AoA_deg = rad2deg(AoA_rad);

    % Calculate lift and drag
    v_A = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);
    [L, D, c_L, c_D] = calculate_lift_and_drag(state, input, AoA_rad, v_A, acc_B, mass_kg, g);

    %%%
    % Aggregate data
    % Set aggregation parameters
    maneuver_trim_start = -1; %s
    maneuver_trim_end = -1; %s
    sweep_maneuver_length = 16; % s
    sweep_maneuver_length_indices = round(sweep_maneuver_length / dt);

    % Initialize empty output variables
    maneuver_start_indices = [];
    t_aggregated = [];
    input_aggregated = [];
    state_aggregated = [];
    c_D_aggregated = [];
    c_L_aggregated = [];
    AoA_aggregated = [];

    % Iterate through maneuvers
    num_aggregated_maneuvers = 0;
    for i = maneuvers_to_aggregate
        maneuver_start_index = max([sysid_indices(i) + round(maneuver_trim_start / dt) 1]);
        maneuver_end_index = max([maneuver_start_index + sweep_maneuver_length / dt + round(maneuver_trim_end/ dt) 1]);
        maneuver_start_indices = [maneuver_start_indices maneuver_start_index];

        % Extract only maneuver data
        t_maneuver = t(maneuver_start_index:maneuver_end_index)';
        state_maneuver = state(maneuver_start_index:maneuver_end_index,:);
        input_maneuver = input(maneuver_start_index:maneuver_end_index,:);
        AoA_deg_maneuver = AoA_deg(maneuver_start_index:maneuver_end_index);
        AoA_rad_maneuver = AoA_rad(maneuver_start_index:maneuver_end_index);
        acc_B_maneuver = acc_B(maneuver_start_index:maneuver_end_index,:);
        acc_B_unfiltered_maneuver = acc_B_unfiltered(maneuver_start_index:maneuver_end_index,:);
        acc_N_maneuver = acc_N(maneuver_start_index:maneuver_end_index,:);
        bias_acc_maneuver = bias_acc(maneuver_start_index:maneuver_end_index);
        L_maneuver = L(maneuver_start_index:maneuver_end_index);
        D_maneuver = D(maneuver_start_index:maneuver_end_index);
        c_L_maneuver = c_L(maneuver_start_index:maneuver_end_index);
        c_D_maneuver = c_D(maneuver_start_index:maneuver_end_index);

        % Plot data
        plot_trajectory(i, t_maneuver, state_maneuver, input_maneuver, show_plot, save_plot, plot_output_location);
        plot_trajectory_static_details(i, t_maneuver, state_maneuver, input_maneuver,...
            AoA_deg_maneuver, acc_B_unfiltered_maneuver, acc_B_maneuver, bias_acc_maneuver,...
            acc_N_maneuver, show_plot, save_plot, plot_output_location);
        plot_scatter_lift_drag(i, t_maneuver, AoA_deg_maneuver, L_maneuver, D_maneuver, c_L_maneuver, c_D_maneuver, show_plot, save_plot, plot_output_location);

    end
end

%%

function [] = read_state_input_data_in_time_interval(start_time_s, end_time_s, csv_log_file_location, dt, save_plot, show_plot, plot_output_location, save_output_data, data_output_location)
    % Read data
    [t, state, input] = read_state_and_input_from_log(csv_log_file_location, dt);

    % Cut data


    % Find indexes in t corresponding to start and end time
    start_index = round(interp1(t, 1:length(t), start_time_s));
    end_index = round(interp1(t, 1:length(t), end_time_s));

    % Cut data
    t = t(start_index:end_index);
    state = state(start_index:end_index,:);
    input = input(start_index:end_index,:);

    % Plot data
    name = "freehand";
    plot_state_input(t, state, input, show_plot, save_plot, plot_output_location, name)   

    % Save to files
    if save_output_data
        writematrix(t, data_output_location + 't.csv');
        writematrix(state, data_output_location + 'state.csv');
        writematrix(input, data_output_location + 'input.csv');
    end

end


%%

function [] = read_static_curves_data()
    % File location
    static_curves_log_file = "2021_04_18_flight_2_static_curves_no_thrust_211_roll";
    csv_files_location = 'logs/csv/';
    csv_log_file_location = csv_files_location + log_file;

    % Output data
    save_output = false;
    %output_location = "static_curves/data/";
    plot_output_location = "plots/maneuver_plots/";

    % Plotting options
    show_plots = false;
    save_plots = false;

    % Set common data time resolution
    dt = 0.01;

    % Aircraft parameters
    aircraft_properties

    % Read data
    [t, state, input] = read_state_and_input_from_log(csv_log_file_location, dt);
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    sysid_indices = get_sysid_indices(csv_log_file_location, t);
    [acc_B_unfiltered, acc_B_raw, t_acc_raw, bias_acc] = read_accelerations(csv_log_file_location, t);
    [V_a_measured, V_a_measured_validated] = read_airspeed(csv_log_file_location, t);
    wind_B = get_wind_body_frame(csv_log_file_location, state, t);
    v_airspeed_B = v_B - wind_B;
    V_a_calculated = sqrt(v_airspeed_B(:,1).^2 + v_airspeed_B(:,2).^2 + v_airspeed_B(:,3).^2);


    %%% Analyze and aggregate data
    % Data filtering
    AIRSPEED_TRESHOLD_MIN = 21; % m/s
    AIRSPEED_TRESHOLD_MAX = 25; % m/s

    % Cutoff frequency for acceleration data
    f_cutoff = 15;

    %maneuvers_to_aggregate = [2:9 11:27 29:31]; % All maneuvers without dropout
    %maneuvers_to_aggregate = 1:31; % All maneuvers without dropout
    %maneuvers_to_aggregate = [2:7 9 11:13 15 17 20:23 29:31];
    static_curves_maneuvers_to_aggregate = [9 11:13 20:23 31];

    % Process data
    [acc_B] = filter_accelerations(t, acc_B_raw, t_acc_raw, f_cutoff);
    acc_N = calculate_acc_N(acc_B, q_NB);

    % Calculate Angle of Attack
    AoA_rad = atan2(v_airspeed_B(:,3),v_airspeed_B(:,1));
    %AoA_rad = atan2(v_B(:,3),v_B(:,1));
    AoA_deg = rad2deg(AoA_rad);

    % Calculate lift and drag
    [L, D, c_L, c_D] = calculate_lift_and_drag(state, input, AoA_rad, V_a_calculated, acc_B, mass_kg, g);

    %%%
    % Aggregate data
    % Set aggregation meta data
    maneuver_trim_start = 0.5; %s
    maneuver_trim_end = 2.8; %s
    sweep_maneuver_period = 2; % s
    total_sweep_maneuver_length = 2 * sweep_maneuver_period;
    sweep_maneuver_length_in_indices = round((total_sweep_maneuver_length - maneuver_trim_start - maneuver_trim_end) / dt);

    % Initialize empty output variables
    data_set_length = sweep_maneuver_length_in_indices * length(static_curves_maneuvers_to_aggregate);
    lift_dra = zeros(data_set_length, 3);
    input_output = zeros(data_set_length, 8);
    state_output = zeros(data_set_length, 10);
    c_D_output = zeros(data_set_length, 1);
    c_L_output = zeros(data_set_length, 1);
    AoA_rad_output = zeros(data_set_length, 1);

    % Iterate through maneuvers
    curr_maneuver_aggregation_index = 1;
    num_aggregated_maneuvers = 0;
    aggregated_maneuvers = zeros(100);
    for i = static_curves_maneuvers_to_aggregate
        % Find exact maneuver index interval
        maneuver_start_index = find_exact_start_index_sweep(u_fw(:,2), sysid_indices(i), dt, sweep_maneuver_period);

        % Trim maneuver by padding
        maneuver_start_index = maneuver_start_index + maneuver_trim_start / dt;
        maneuver_end_index = maneuver_start_index + sweep_maneuver_length_in_indices;

        % Extract only maneuver data
        t_maneuver = t(maneuver_start_index:maneuver_end_index);
        state_maneuver = state(maneuver_start_index:maneuver_end_index,:);
        input_maneuver = input(maneuver_start_index:maneuver_end_index,:);
        AoA_deg_maneuver = AoA_deg(maneuver_start_index:maneuver_end_index);
        AoA_rad_maneuver = AoA_rad(maneuver_start_index:maneuver_end_index);
        V_a_measured_maneuver = V_a_measured(maneuver_start_index:maneuver_end_index);
        V_a_measured_validated_maneuver = V_a_measured_validated(maneuver_start_index:maneuver_end_index);
        V_a_calculated_maneuver = V_a_calculated(maneuver_start_index:maneuver_end_index);
        acc_B_maneuver = acc_B(maneuver_start_index:maneuver_end_index,:);
        acc_B_unfiltered_maneuver = acc_B_unfiltered(maneuver_start_index:maneuver_end_index,:);
        acc_N_maneuver = acc_N(maneuver_start_index:maneuver_end_index,:);
        bias_acc_maneuver = bias_acc(maneuver_start_index:maneuver_end_index,:);
        v_airspeed_B_maneuver = v_airspeed_B(maneuver_start_index:maneuver_end_index,:);
        wind_B_maneuver = wind_B(maneuver_start_index:maneuver_end_index,:);
        L_maneuver = L(maneuver_start_index:maneuver_end_index,:);
        D_maneuver = D(maneuver_start_index:maneuver_end_index,:);
        c_L_maneuver = c_L(maneuver_start_index:maneuver_end_index,:);
        c_D_maneuver = c_D(maneuver_start_index:maneuver_end_index,:);

        % Plot data
        plot_trajectory(i, t_maneuver, state_maneuver, input_maneuver, ...
            v_airspeed_B_maneuver, 0.5, show_plots, save_plots, plot_output_location);
        plot_trajectory_static_details(i, t_maneuver, state_maneuver, input_maneuver,...
            AoA_deg_maneuver, acc_B_unfiltered_maneuver, acc_B_maneuver, bias_acc_maneuver,...
            acc_N_maneuver, V_a_measured_maneuver, V_a_calculated_maneuver, show_plots, save_plots, plot_output_location);
        plot_scatter_lift_drag(i, t_maneuver, AoA_deg_maneuver, L_maneuver, D_maneuver, c_L_maneuver, c_D_maneuver, show_plots, save_plots, plot_output_location);

    %     % Check if data passes tests
    %     if ~does_data_pass_validation_tests(state_maneuver, V_a_measured_maneuver, i, AIRSPEED_TRESHOLD_MIN, AIRSPEED_TRESHOLD_MAX)
    %         continue
    %     end

        % Store data in aggregated data matrices
        curr_maneuver_aggregation_index = num_aggregated_maneuvers * sweep_maneuver_length_in_indices + 1;
        state_output(...
            curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + sweep_maneuver_length_in_indices ...
            ,:) = state_maneuver;
        input_output(...
            curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + sweep_maneuver_length_in_indices ...
            ,:) = input_maneuver;
        c_L_output(...
            curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + sweep_maneuver_length_in_indices ...
            ,:) = c_L_maneuver;
        c_D_output(...
            curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + sweep_maneuver_length_in_indices ...
            ,:) = c_D_maneuver;
        AoA_rad_output(...
            curr_maneuver_aggregation_index:curr_maneuver_aggregation_index + sweep_maneuver_length_in_indices ...
            ,:) = AoA_rad_maneuver;

        num_aggregated_maneuvers = num_aggregated_maneuvers + 1;
        aggregated_maneuvers(num_aggregated_maneuvers) = i;

    end

    % Trim data
    state_output = state_output(1:num_aggregated_maneuvers * sweep_maneuver_length_in_indices,:);
    input_output = input_output(1:num_aggregated_maneuvers * sweep_maneuver_length_in_indices,:);
    AoA_rad_output = AoA_rad_output(1:num_aggregated_maneuvers * sweep_maneuver_length_in_indices,:);
    c_D_output = c_D_output(1:num_aggregated_maneuvers * sweep_maneuver_length_in_indices,:);
    c_L_output = c_L_output(1:num_aggregated_maneuvers * sweep_maneuver_length_in_indices,:);

    aggregated_maneuvers = aggregated_maneuvers(1:num_aggregated_maneuvers);
    display("aggregated " + num_aggregated_maneuvers + " maneuvers");
    disp(aggregated_maneuvers);

    % Save to files
    if save_output
        output_data_in_table = table(state_output, input_output);
        writetable(output_data_in_table, output_location + 'output.csv');
        writematrix(AoA_rad_output, output_location + 'AoA_rad.csv');
        writematrix(c_D_output, output_location + 'c_D.csv');
        writematrix(c_L_output, output_location + 'c_L.csv');
        writematrix(sweep_maneuver_length_in_indices, output_location + 'maneuver_length.csv');
        writematrix(dt, output_location + 'dt.csv');
        writematrix(aggregated_maneuvers, output_location + 'aggregated_maneuvers.csv');
    end
    
end

%% Functions
function [t, state, input] = read_state_and_input_from_log(csv_log_file_location, dt)
    ekf_data = readtable(csv_log_file_location + '_' + "estimator_status_0" + ".csv");
    angular_velocity = readtable(csv_log_file_location + '_' + "vehicle_angular_velocity_0" + ".csv");
    actuator_controls_mr = readtable(csv_log_file_location + '_' + "actuator_controls_0_0" + ".csv");
    actuator_controls_fw = readtable(csv_log_file_location + '_' + "actuator_controls_1_0" + ".csv");
    
    %%%
    % Common time vector
    
    t0 = ekf_data.timestamp(1) / 1e6;
    t_end = ekf_data.timestamp(end) / 1e6;

    t = t0:dt:t_end;
    N = length(t);
    
    %%%
    % Extract data from ekf2

    t_ekf = ekf_data.timestamp / 1e6;

    % q_NB = unit quaternion describing vector rotation from NED to Body. i.e.
    % describes transformation from Body to NED frame.
    % Note: This is the same as the output_predictor quaternion. Something is
    % wrong with documentation

    q0_raw = ekf_data.states_0_;
    q1_raw = ekf_data.states_1_;
    q2_raw = ekf_data.states_2_;
    q3_raw = ekf_data.states_3_;
    q_NB_raw = [q0_raw q1_raw q2_raw q3_raw];
    

    % Extrapolate to correct time
    q_NB = interp1q(t_ekf, q_NB_raw, t');
    % q_NB is the quat we are looking for for our state vector
    q0 = q_NB(:,1);
    q1 = q_NB(:,2);
    q2 = q_NB(:,3);
    q3 = q_NB(:,4);

    v_n = ekf_data.states_4_;
    v_e = ekf_data.states_5_;
    v_d = ekf_data.states_6_;
    v_N_raw = [v_n v_e v_d];

    % Extrapolate to correct time
    v_N = interp1q(t_ekf, v_N_raw, t');

    p_n = ekf_data.states_7_;
    p_e = ekf_data.states_8_;
    p_d = ekf_data.states_9_;
    p_N_raw = [p_n p_e p_d];
    p_N = interp1q(t_ekf, p_N_raw, t');

    %%%
    % Extract angular velocities from output predictor

    t_ang_vel = angular_velocity.timestamp / 1e6;

    % Angular velocity around body axes
    p_raw = angular_velocity.xyz_0_;
    q_raw = angular_velocity.xyz_1_;
    r_raw = angular_velocity.xyz_2_;
    w_B_raw = [p_raw q_raw r_raw];
    
    % Extrapolate to correct time
    w_B = interp1q(t_ang_vel, w_B_raw, t');
    p = w_B(:,1);
    q = w_B(:,2);
    r = w_B(:,3);

    %%%
    % Convert velocity to correct frames

    % This is rotated with rotation matrices only for improved readability,
    % as both the PX4 docs is ambiguous in describing q, and quatrotate() is
    % pretty ambigious too.
    R_NB = quat2rotm(q_NB);

    q_BN = quatinv(q_NB);
    R_BN = quat2rotm(q_BN);

    v_B = zeros(N, 3);
    for i = 1:N
       % Notice how the Rotation matrix has to be inverted here to get the
       % right result, indicating that q is in fact q_NB and not q_BN.
       v_B(i,:) = (R_BN(:,:,i) * v_N(i,:)')';
    end
    u = v_B(:,1);
    v = v_B(:,2);
    w = v_B(:,3);
    
    % The following line gives the same result, indicating that quatrotate() does not perform a
    % simple quaternion product: q (x) v (x) q_inv
    % v_B = quatrotate(q_NB, v_N);
    
    %%%
    % Extract input data
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
    
    
    %%%
    % Create state and input
    %   state structure: [att ang_vel_B vel_B] = [q0 q1 q2 q3 p q r u v w]
    %   input structure: [top_rpm_1 top_rpm_2 top_rpm_3 top_rpm_4 aileron elevator rudder pusher_rpm]
    %                       = [nt1 nt2 nt3 nt4 np delta_a delta_e delta_r]
    
    state = [q0 q1 q2 q3 p q r u v w];
    input = [u_mr u_fw];
    
end

function [sysid_indices] = get_sysid_indices(csv_log_file_location, t)
    input_rc = readtable(csv_log_file_location + '_' + "input_rc_0" + ".csv");
    
    % Extract RC sysid switch log
    sysid_rc_switch_raw = input_rc.values_6_; % sysid switch mapped to button 6
    t_rc = input_rc.timestamp / 1e6;
    RC_TRESHOLD = 1800;

    % Find times when switch was switched
    MAX_SYSID_MANEUVERS = 500;
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
    
    % Plot
    if 0
        plot(t_rc, sysid_rc_switch_raw); hold on;
        plot(sysid_times, 1000, 'r*');
    end

    % Find corresponding indices in time vector
    sysid_indices = round(interp1(t,1:length(t),sysid_times));
    sysid_indices = sysid_indices(1:sysid_maneuver_num - 1);
end

function [wind_B] = get_wind_body_frame(csv_log_file_location, state, t)
    % Load data
    wind_data = readtable(csv_log_file_location + '_' + "wind_estimate_1" + ".csv");
    
     % Read raw sensor data
    t_wind = wind_data.timestamp / 1e6;
    wind_N_raw = [wind_data.windspeed_north wind_data.windspeed_east zeros(size(wind_data.windspeed_north))];
    
    wind_N = interp1q(t_wind, wind_N_raw, t');

    % Rotate to body frame
    q_NB = state(:,1:4);

    q_BN = quatinv(q_NB);
    R_BN = quat2rotm(q_BN);
    wind_B = zeros(size(wind_N));
    for i = 1:length(wind_N)
       % Notice how the Rotation matrix has to be inverted here to get the
       % right result, indicating that q is in fact q_NB and not q_BN.
       wind_B(i,:) = (R_BN(:,:,i) * wind_N(i,:)')';
    end 
end



function [acc_B, acc_B_raw, t_acc, bias_acc] = read_accelerations(csv_log_file_location, t)
    % Load data
    sensor_combined = readtable(csv_log_file_location + '_' + "sensor_combined_0" + ".csv");
    sensor_bias = readtable(csv_log_file_location + '_' + "estimator_sensor_bias_0" + ".csv");

    % Read raw sensor data
    t_acc = sensor_combined.timestamp / 1e6;
    acc_B_raw = [sensor_combined.accelerometer_m_s2_0_ sensor_combined.accelerometer_m_s2_1_ sensor_combined.accelerometer_m_s2_2_];

    % Check if significant bias
    t_sensor_bias = sensor_bias.timestamp / 1e6;
    bias_acc_raw = [sensor_bias.accel_bias_0_ sensor_bias.accel_bias_1_ sensor_bias.accel_bias_2_];
    bias_acc = interp1q(t_sensor_bias, bias_acc_raw, t');

    if 0
        figure
        subplot(2,1,1)
        plot(t, acc_B(:,1)); hold on
        plot(t, bias_acc(:,1));
        legend('acc x','bias')

        subplot(2,1,2)
        plot(t, acc_B(:,3)); hold on
        plot(t, bias_acc(:,3));
        legend('acc z','bias')
    end

    % Fuse to common time horizon
    acc_B = interp1q(t_acc, acc_B_raw, t');
end

function [acc_B_filtered] = filter_accelerations(t, acc_B_raw, t_acc_raw, f_cutoff)
    % Filter data
    T_c = 1/f_cutoff;
    temp = filloutliers(t_acc_raw(2:end) - t_acc_raw(1:end-1), 'linear');
    dt_acc = mean(temp);
    alpha = dt_acc / (T_c + dt_acc);

    acc_B_raw_filtered = zeros(size(acc_B_raw));
    acc_B_raw_filtered(1,:) = acc_B_raw(1,:);
    for i = 2:length(acc_B_raw)
       acc_B_raw_filtered(i,:) = alpha * acc_B_raw(i,:) + (1 - alpha) * acc_B_raw_filtered(i-1,:);
    end

    % Frequency analysis
    if 0
        plot_fft(acc_B_raw, dt);
        plot_fft(acc_B_raw_filtered, dt);
    end

    % Fuse to common time horizon
    acc_B_filtered = interp1q(t_acc_raw, acc_B_raw_filtered, t');
end

function [V_a, V_a_validated] = read_airspeed(csv_log_file_location, t)
    airspeed_data = readtable(csv_log_file_location + '_' + "airspeed_0" + ".csv");
    airspeed_validated_data = readtable(csv_log_file_location + '_' + "airspeed_validated_0" + ".csv");

    t_airspeed = airspeed_data.timestamp / 1e6;
    V_a_raw = airspeed_data.true_airspeed_m_s;
    V_a = interp1q(t_airspeed, V_a_raw, t');

    t_airspeed_validated = airspeed_validated_data.timestamp / 1e6;
    V_a_validated_raw = airspeed_validated_data.true_airspeed_m_s;
    V_a_validated = interp1q(t_airspeed_validated, V_a_validated_raw, t');
end

function [] = plot_state_input(t, state, input, show_plot, save_plot, plot_output_location, name)        
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    
    eul = quat2eul(q_NB);
    eul_deg = rad2deg(eul);

    fig = figure;
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 1000];
    num_plots = 9; 
   
    subplot(num_plots,1,1);
    plot(t, eul_deg(:,2:3));
    legend('pitch','roll');
    title("attitude")
    
    subplot(num_plots,1,2);
    plot(t, eul_deg(:,1));
    legend('yaw');
    title("attitude")

    subplot(num_plots,1,3);
    plot(t, w_B);
    legend('p','q','r');
    max_ang_rate = max(max(max(abs(state(:,5:7)))), 1); % Never let ang rate axis be smaller than 1
    ylim([-max_ang_rate max_ang_rate])
    title("ang vel body")

    subplot(num_plots,1,4);
    plot(t, v_B(:,1));
    legend('u');
    title("vel body")
    
    subplot(num_plots,1,5);
    plot(t, v_B(:,2));
    legend('v');
    title("vel body")
    
    subplot(num_plots,1,6);
    plot(t, v_B(:,3));
    legend('w');
    title("vel body")

    subplot(num_plots,1,7);
    plot(t, u_mr);
    legend('delta_a','delta_e','delta_r', 'T_mr');
    title("mr inputs")
    
    subplot(num_plots,1,8);
    plot(t, u_fw(:,1:3));
    legend('delta_a','delta_e','delta_r');
    title("fw inputs")
    
    subplot(num_plots,1,9);
    plot(t, u_fw(:,4));
    legend('T_fw');
    title("fw inputs")
    
    figure_title = "state and input: " + name;
    sgtitle(figure_title)

    if save_plot
        filename = name + "_state_input";
        saveas(fig, plot_output_location + filename, 'epsc')
        savefig(plot_output_location + filename + '.fig')
    end
end

function [] = plot_trajectory(maneuver_index, t, state, input, show_plot, save_plot, plot_output_location)        
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    
    eul = quat2eul(q_NB);
    eul_deg = rad2deg(eul);

    fig = figure;
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 1000];
    num_plots = 6; 
   
    subplot(num_plots,1,1);
    plot(t, eul_deg(:,2:3));
    legend('pitch','roll');
    title("attitude")
    
    subplot(num_plots,1,2);
    plot(t, eul_deg(:,1));
    legend('yaw');
    title("attitude")

    subplot(num_plots,1,3);
    plot(t, w_B);
    legend('p','q','r');
    max_ang_rate = 0.5;
    ylim([-max_ang_rate max_ang_rate])
    title("ang vel body")

    subplot(num_plots,1,4);
    plot(t, v_B(:,1)); hold on
    legend('u');
    title("vel body")
    
    subplot(num_plots,1,5);
    plot(t, v_B(:,2:3)); hold on;
    legend('v','w');
    title("vel body")

    subplot(num_plots,1,6);
    plot(t, u_fw);
    legend('delta_a','delta_e','delta_r', 'T_fw');
    title("inputs")
    
    figure_title = "state and input, maneuver: " + maneuver_index;
    sgtitle(figure_title)

    if save_plot
        filename = maneuver_index + "_state_input";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end

function [] = plot_trajectory_static_details(maneuver_index, t, state, input, AoA, ...
    acc_B, acc_B_filtered, bias_acc, acc_N, ...
    show_plot, save_plot, plot_output_location)
    
% Figure details
    fig = figure; 
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 1000];
    num_plots = 5; 

    % Extract all state values
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    V = sqrt(sum(v_B.^2'));

    eul = quat2eul(q_NB);
    eul_deg = rad2deg(eul);
    
    subplot(num_plots,1,1);
    plot(t, AoA);
    legend('AoA');
    title("Angle of Attack")

    subplot(num_plots,1,2);
    plot(t, V);
    legend('V body');
    title("Airspeed (assuming no wind)")

    subplot(num_plots,1,3);
    plot(t, acc_B(:,1)); hold on
    plot(t, acc_B_filtered(:,1)); hold on
    plot(t, bias_acc(:,1));
    legend('acc B', 'acc B filtered', 'bias');
    title("a_x")

    subplot(num_plots,1,4);
    plot(t, acc_B(:,3)); hold on
    plot(t, acc_B_filtered(:,3));
    plot(t, bias_acc(:,3));
    legend('acc B', 'acc B filtered', 'bias');
    title("a_z")

    subplot(num_plots,1,5);
    g = 9.81;
    plot(t, abs(acc_N(:,3) + g));
    title("acc z compared to 1 g")
    
    figure_title = "Lift and drag details, maneuver: " + maneuver_index;
    sgtitle(figure_title)
    
    if save_plot
        filename = maneuver_index + "_details";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end

function [] = plot_wind(maneuver_index, t, wind_B, ...
    show_plot, save_plot, plot_output_location)
    
    % Figure details
    fig = figure; 
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 600];

    plot(t, wind_B);
    legend('wind_x', 'wind_y', 'wind_z');
    
    figure_title = "estimated wind, maneuver: " + maneuver_index;
    title(figure_title)
    
    if save_plot
        filename = maneuver_index + "_wind";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end

function [] = plot_scatter_lift_drag(maneuver_index, t, AoA_deg, L, D, c_L, c_D, show_plot, save_plot, plot_output_location)
    % Figure details
    fig = figure; 
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 1000];

    subplot(2,2,1)
    scatter(AoA_deg, L, [], t)
    xlabel('AoA [deg]')
    ylabel('L')
    title("Lift")
    colorbar
    
    subplot(2,2,2)
    scatter(AoA_deg, D, [], t)
    xlabel('AoA [deg]')
    ylabel('D')
    title("Drag")
    colorbar
    
    subplot(2,2,3)
    scatter(AoA_deg, c_L, [], t)
    xlabel('AoA [deg]')
    ylabel('c_L')
    title("Lift coeff")
    colorbar
    
    subplot(2,2,4)
    scatter(AoA_deg, c_D, [], t)
    xlabel('AoA [deg]')
    ylabel('c_D')
    title("Drag coeff")
    colorbar
    
    figure_title = "static curves, maneuver: " + maneuver_index;
    sgtitle(figure_title)
    
    if save_plot
        filename = maneuver_index + "_static_curves";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end


function [maneuver_start_index] = find_exact_start_index_sweep_maneuver(input, guess_sysid_index, dt, maneuver_period)
    % Find correct maneuver start index
    time_to_search_before_maneuver = 1; %s
    time_to_search_after_maneuver_start = 3; %s

    indices_to_search_before_maneuver = time_to_search_before_maneuver / dt;
    indices_to_search_after_maneuver_start = time_to_search_after_maneuver_start / dt;
    max_start_index = guess_sysid_index - indices_to_search_before_maneuver;
    max_end_index = guess_sysid_index + indices_to_search_after_maneuver_start;
    % Move to correct start index
    for j = 1:maneuver_period/dt
        [~, maneuver_top_index] = max(input(max_start_index:max_end_index));
        maneuver_top_index = max_start_index + maneuver_top_index; % What is this line doing??
    end
    
    maneuver_start_index = maneuver_top_index - maneuver_period / dt;
end


% TODO
function [maneuver_passed_data_test] = does_data_pass_validation_tests(state_maneuver, airspeed_maneuver, maneuver_index, AIRSPEED_TRESHOLD_MIN, AIRSPEED_TRESHOLD_MAX)  
    q_NB = state_maneuver(:,1:4);
    w_B = state_maneuver(:,5:7);
    v_B = state_maneuver(:,8:10);
    u_mr = state_maneuver(:,1:4);
    u_fw = state_maneuver(:,5:8);
    
    maneuver_passed_data_test = true;    
    if (airspeed_maneuver(1) < AIRSPEED_TRESHOLD_MIN)
       display("skipping maneuver " + maneuver_index + ": airspeed too low");
       maneuver_passed_data_test = false;
    end
    if (airspeed_maneuver(1) > AIRSPEED_TRESHOLD_MAX)
       display("skipping maneuver " + maneuver_index + ": airspeed too high");
       maneuver_passed_data_test = false;
    end
    
    % Calculate data cleanliness
    std_ang_rates = std(w_B);
    std_body_vel = std(v_B);
    total_std = std_ang_rates(1) + std_ang_rates(3 ) + std_body_vel(2);
    
    if (total_std > 0.7)
       display("skipping maneuver " + maneuver_index + ": total std: " + total_std);
       maneuver_passed_data_test = false;
    end
end

function [acc_N] = calculate_acc_N(acc_B, q_NB)
    R_NB = quat2rotm(q_NB);
    % Calculate NED acceleration
    acc_N = zeros(size(acc_B));
    for i = 1:length(acc_B)
       acc_N(i,:) = (R_NB(:,:,i) * acc_B(i,:)')';
    end
end

function plot_fft(data, dt)
    % Frequency analysis
    figure
    Y = fft(data);
    L = length(data);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    Fs = 1/dt;
    f = Fs*(0:(L/2))/L;
    plot(f,P1)
    title('Single-Sided Amplitude Spectrum of X(t)')
    xlabel('f (Hz)')
    ylabel('|P1(f)|')
end


