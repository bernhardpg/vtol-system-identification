% Script that loads maneuver data for desired maneuver types into workspace
% Set maneuver_types and data_type before running.

data = [];
maneuver_start_indices = [];

for maneuver_type = maneuver_types
    % Maneuver settings
    data_path = "data/flight_data/aggregated_data/" + maneuver_type + "/";

    % Load training data
    data_for_man_type = readmatrix(data_path + "data_" + data_type + ".csv");
    maneuver_start_indices_for_man_type = readmatrix(data_path + "maneuver_start_indices_" + data_type + ".csv");
    maneuver_start_indices_for_man_type = maneuver_start_indices_for_man_type + length(data); % Make indices continue

    data = [data;
            data_for_man_type];
        
    maneuver_start_indices = [maneuver_start_indices;
                              maneuver_start_indices_for_man_type];
        
end

[t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a_sp, delta_vl_sp, delta_vr_sp, delta_a, delta_vl, delta_vr, n_p, c_X, c_Y, c_Z, c_l, c_m, c_n, c_L, c_D]...
    = extract_variables_from_data(data);
dt = t(2) - t(1);
aoa_alpha = atan2(w, u);
V = sqrt(u.^2 + v.^2 + w.^2);
beta = asin(v ./ V);

delta_e = 0.5 * (delta_vl + delta_vr);
delta_r = 0.5 * (-delta_vl + delta_vr);
delta_e_sp = 0.5 * (delta_vl_sp + delta_vr_sp);
delta_r_sp = 0.5 * (-delta_vl_sp + delta_vr_sp);


maneuver_indices = [maneuver_start_indices; length(t)]; % Add end index to maneuver indices to make handling of last maneuver easier
num_maneuvers = length(maneuver_indices) - 1;
disp("Loaded " + num_maneuvers + " maneuvers.")