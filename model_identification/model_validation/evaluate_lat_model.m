% Load initial guesses
load_const_params;
equation_error_results_lon;
x0_lon = [  c_X_0, c_X_q, c_X_u, c_X_w, c_X_w_sq, c_X_delta_e,...
        c_Z_0, c_Z_w, c_Z_delta_e,...
        c_m_0, c_m_q, c_m_w, c_m_delta_e, c_m_delta_e_sq, c_m_delta_r_sq];
equation_error_results_lat;
x0_lat = [
     c_Y_0, c_Y_p, c_Y_v, c_Y_delta_a, c_Y_delta_r,...
     c_l_0, c_l_p, c_l_r, c_l_v, c_l_delta_a,...
     c_n_0, c_n_p, c_n_r, c_n_v, c_n_delta_r,...
     ];
 
 
 x = [x0_lon x0_lat];
 
% Test model
data_type = "val";
maneuver_types = ["roll_211" "pitch_211" "yaw_211"];
load_data; % Loads all states and inputs
maneuvers_to_test = [1 2 10 11 21 22];
save_plot = false;
show_plot = true;
evaluate_full_model(maneuvers_to_test, maneuver_types, x, save_plot, show_plot,...
    maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_e_sp, delta_r_sp, delta_a, delta_e, delta_r, n_p);
