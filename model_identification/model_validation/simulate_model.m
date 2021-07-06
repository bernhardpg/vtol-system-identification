clc; clear all; close all;

% Load validation data
data_type = "val";
maneuver_types = ["yaw_211"];
load_data; % Loads all states and inputs

% Load parameters
equation_error_results_lon;
x_lon = [c_D_0 c_D_alpha c_D_alpha_sq c_D_q c_D_delta_e c_L_0 c_L_alpha c_L_alpha_sq c_L_q_hat c_L_delta_e c_m_0 c_m_alpha c_m_q c_m_delta_e c_m_delta_e_sq];
maneuvers_to_test = 1:num_maneuvers;
save_plot = true;
show_plot = false;
evaluate_full_model(maneuvers_to_test, maneuver_types, x, save_plot, show_plot,...
    maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_vl_sp, delta_vr_sp, delta_a, delta_vl, delta_vr, n_p);

