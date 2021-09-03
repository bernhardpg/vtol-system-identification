clc; clear all; close all;

% The script will automoatically load all maneuvers in "data_raw/experiments/*"

% This script takes in raw maneuver data from csv files and calculates all
% the required signals and their derivatives. This includes states, their
% derivatives, coefficients, and inputs.

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

disp("Setting random seed to default to guarantee reproducability");
rng default % Always shuffle the maneuvers in the same way for reproducability

% Load metadata
metadata_filename = "data/flight_data/metadata.json";
metadata = read_metadata(metadata_filename);

% How much data to keep for validation
val_ratio = 0.25;

% Plot settings
save_raw_maneuver_plot = true;
save_maneuver_plot = false;
show_maneuver_plot = false;

% Maneuver settings
maneuver_types = [
   "roll_211",...
   "pitch_211",...
   "yaw_211",...
   "roll_211_no_throttle",...
   "pitch_211_no_throttle",...
   "yaw_211_no_throttle",...
   "freehand",...
    ];

maneuvers_to_skip = {};
maneuvers_to_skip.("roll_211") = [];
maneuvers_to_skip.("roll_211_no_throttle") = [];
maneuvers_to_skip.("pitch_211") = [];
maneuvers_to_skip.("pitch_211_no_throttle") = [];
maneuvers_to_skip.("yaw_211") = [];
maneuvers_to_skip.("yaw_211_no_throttle") = [];
maneuvers_to_skip.("freehand") = [];

% maneuvers_to_skip = {};
% maneuvers_to_skip.("roll_211") = [12 13 14 15 16 17 40 41 42 44 45 54 58 59 60 64];
% maneuvers_to_skip.("roll_211_no_throttle") = [];
% maneuvers_to_skip.("pitch_211") = [1]; % dropout that went unoticed by automated test
% maneuvers_to_skip.("pitch_211_no_throttle") = [];
% maneuvers_to_skip.("yaw_211") = [1 4 7];
% maneuvers_to_skip.("yaw_211_no_throttle") = [];
% maneuvers_to_skip.("sweep") = [];

for maneuver_type = maneuver_types
    maneuvers_to_skip_for_curr_type = maneuvers_to_skip.(maneuver_type);
    disp("Processing " + maneuver_type + " maneuvers.");
    
    % Plot settings
    plot_location = "data/flight_data/maneuver_plots/" + maneuver_type + "/";

    % Set data params
    dt = 1 / 50;

    % Read data recorded from logs
    [t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw] ...
        = read_experiment_data(metadata, maneuver_type);

    % Shuffle maneuvers
    [t_recorded, eul_recorded, phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded, t_u_fw_recorded, u_fw_recorded]...
        = shuffle_maneuvers(maneuvers_to_skip_for_curr_type, t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw);
    
    % Calculate body velocities from recorded data
    [u_recorded, v_recorded, w_recorded] = calc_body_vel(phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded);
    
        
    %%%%
    % CONTINUE HERE
    %%%%
    % Figure out why all maneuvers are not loaded
    % Plot unprocessed data from maneuvers
    
    
    % Split up that crazy collect_data_from_all_maneuvers function!
    

    
    % Calculate states and their derivatives using splines
    [t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a_sp, delta_e_sp, delta_r_sp, delta_a, delta_e, delta_r, n_p, maneuver_start_indices]...
        = collect_data_from_all_maneuvers(maneuvers_to_skip_for_curr_type, dt, t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw,...
            save_maneuver_plot, show_maneuver_plot, plot_location);

    [c_X, c_Y, c_Z, c_L, c_D] = calc_force_coeffs(u, v, w, a_x, a_y, a_z, n_p); % For now, lift and drag coeff is not used for anything
    [c_l, c_m, c_n] = calc_moment_coeffs(p, q, r, u, v, w, p_dot, q_dot, r_dot, n_p);

    % Collect data
    data = [t phi theta psi p q r u v w a_x a_y a_z p_dot q_dot r_dot delta_a_sp delta_e_sp delta_r_sp delta_a delta_e delta_r n_p c_X c_Y c_Z c_l c_m c_n c_L c_D];

    % Divide in training and validation data
    total_num_maneuvers = length(maneuver_start_indices);
    num_val_maneuvers = floor(total_num_maneuvers * val_ratio);
    num_train_maneuvers = total_num_maneuvers - num_val_maneuvers;

    maneuver_start_indices_train = maneuver_start_indices(1:num_train_maneuvers);
    maneuver_start_indices_val = maneuver_start_indices(num_train_maneuvers + 1:end);
    data_train = data(1:maneuver_start_indices_val(1) - 1,:);
    data_val = data(maneuver_start_indices_val(1):end,:);
    maneuver_start_indices_val = maneuver_start_indices_val - maneuver_start_indices_val(1) + 1; % Make this start at 1 for logged data

    % Save all data
    output_data_path = "data/flight_data/aggregated_data/" + maneuver_type + "/";
    mkdir(output_data_path)
    writematrix(data_train, output_data_path + "data_train.csv");
    writematrix(data_val, output_data_path + "data_val.csv");
    writematrix(maneuver_start_indices_train, output_data_path + "maneuver_start_indices_train.csv");
    writematrix(maneuver_start_indices_val, output_data_path + "maneuver_start_indices_val.csv");

    disp("Saved " + num_train_maneuvers + " maneuvers for training");
    disp("Saved " + num_val_maneuvers + " maneuvers for validation");
end

