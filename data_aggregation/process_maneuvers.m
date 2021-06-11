clc; clear all; close all;

% This script takes in raw maneuver data from csv files and calculates all
% the required signals and their derivatives. This includes states, their
% derivatives, coefficients, and inputs.

disp("Setting random seed to default to guarantee reproducability");
rng default % Always shuffle the maneuvers in the same way for reproducability

% Load metadata
metadata_filename = "data/flight_data/metadata.json";
metadata = read_metadata(metadata_filename);

% How much data to keep for validation
val_ratio = 0.2;

% Maneuver settings
maneuver_types = ["roll_211"];
maneuvers_to_skip = {};
maneuvers_to_skip.("roll_211") = [12 13 14 15 16 17 40 41 42 44 45 54 58 59 60 64];
maneuvers_to_skip.("pitch_211") = [1]; % dropout that went unoticed by automatic test
maneuvers_to_skip.("yaw_211") = [1 4 7];
save_maneuver_plot = false;
show_maneuver_plot = false;

for maneuver_type = maneuver_types
    disp("Processing " + maneuver_type + " maneuvers.");
    
    % Plot settings
    plot_location = "data/flight_data/maneuver_plots/" + maneuver_type + "/";

    % Set data params
    dt = 1 / 50;

    % Read data recorded from logs
    [t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw] ...
        = read_experiment_data(metadata, maneuver_type);

    % Calculate states and their derivatives using splines
    [t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a_sp, delta_e_sp, delta_r_sp, delta_a, delta_e, delta_r, n_p, maneuver_start_indices]...
        = collect_data_from_all_maneuvers(maneuvers_to_skip.(maneuver_type), dt, t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw,...
            save_maneuver_plot, show_maneuver_plot, plot_location);

    [c_X, c_Y, c_Z] = calc_force_coeffs(u, v, w, a_x, a_y, a_z, n_p);
    [c_l, c_m, c_n] = calc_moment_coeffs(p, q, r, u, v, w, p_dot, q_dot, r_dot);

    % Collect data
    data = [t phi theta psi p q r u v w a_x a_y a_z p_dot q_dot r_dot delta_a_sp delta_e_sp delta_r_sp delta_a delta_e delta_r n_p c_X c_Y c_Z c_l c_m c_n];

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

