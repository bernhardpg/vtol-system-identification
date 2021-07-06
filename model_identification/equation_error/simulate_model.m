clc; clear all; close all;

% Load validation data
data_type = "val";
maneuver_types = ["yaw_211"];
load_data; % Loads all states and inputs

% Load parameters
equation_error_results_lat;
equation_error_results_lon;
%x = [c_D_0 c_D_alpha c_D_q c_D_alpha_sq c_D_delta_e c_L_0 c_L_alpha c_L_delta_e c_m_0 c_m_alpha c_m_q c_m_delta_e c_m_delta_e_sq c_m_alpha_sq c_Y_0 c_Y_beta c_Y_p c_Y_delta_a c_Y_delta_r c_l_0 c_l_delta_a c_l_p c_l_beta c_l_r c_n_0 c_n_beta c_n_p c_n_r c_n_delta_a c_n_delta_r];
x = [0.1246    0.2767    8.5167    1.8681    0.1412    0.5135    4.0559    0.3482    0.0062   -1.0687  -13.3937   -0.6828 -0.5180 0.8074 c_Y_0 c_Y_beta c_Y_p c_Y_delta_a c_Y_delta_r c_l_0 c_l_delta_a c_l_p c_l_beta c_l_r c_n_0 c_n_beta c_n_p c_n_r c_n_delta_a c_n_delta_r];
maneuvers_to_test = [1 2];
save_plot = false;
show_plot = true;
evaluate_full_model(maneuvers_to_test, maneuver_types, x, save_plot, show_plot,...
    maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_vl_sp, delta_vr_sp, delta_a, delta_vl, delta_vr, n_p);

