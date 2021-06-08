clc; clear all; close all;

% This script takes in raw maneuver data from csv files and calculates all
% the required signals and their derivatives. This includes states, their
% derivatives, coefficients, and inputs.

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Maneuver settings
maneuver_type = "pitch_211";

% Plot settings
plot_location = "data/man euver_plots/" + maneuver_type + "/";
save_maneuver_plot = true;
show_maneuver_plot = false;

% Set data params
dt = 1 / 50;

% Read data recorded from logs
[t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw] ...
    = read_experiment_data(metadata, maneuver_type);

% Calculate states and their derivatives using splines
% TODO: Save maneuers for validation
% TODO: Remove bad maneuvers

maneuvers_to_skip = [1];

maneuvers_val = rand_pick_n_indices(choose_num_maneuvers, total_num_maneuvers);

[t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a, delta_e, delta_r, n_p, maneuver_start_indices]...
    = collect_data_from_all_maneuvers(maneuvers_to_skip, dt, t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw,...
        save_maneuver_plot, show_maneuver_plot, plot_location);

% TODO: I need to find actual PWM to RPM scale.
%T = calc_propeller_force(n_p);
[c_X, c_Y, c_Z] = calc_force_coeffs(u, v, w, a_x, a_y, a_z);
[c_l, c_m, c_n] = calc_moment_coeffs(p, q, r, u, v, w, p_dot, q_dot, r_dot);

% Save all data
all_data = [t phi theta psi p q r u v w a_x a_y a_z p_dot q_dot r_dot delta_a delta_e delta_r n_p c_X c_Y c_Z c_l c_m c_n];
output_data_path = "data/aggregated_data/" + maneuver_type + "/";
mkdir(output_data_path)
writematrix(all_data, output_data_path + "data.csv");
writematrix(maneuver_start_indices, output_data_path + "maneuver_start_indices.csv");

