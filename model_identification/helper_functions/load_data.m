% Maneuver settings
maneuver_type = "pitch_211";
data_path = "data/aggregated_data/" + maneuver_type + "/";

% Load training data
data = readmatrix(data_path + "data_train.csv");
maneuver_start_indices = readmatrix(data_path + "maneuver_start_indices_train.csv");

[t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a_sp, delta_e_sp, delta_r_sp, delta_a, delta_e, delta_r, n_p, c_X, c_Y, c_Z, c_l, c_m, c_n]...
    = extract_variables_from_data(data);
dt = t(2) - t(1);

maneuver_indices = [maneuver_start_indices; length(t)]; % Add end index to this
num_maneuvers = length(maneuver_indices) - 1;

% Create explanatory variables for equation-error
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);