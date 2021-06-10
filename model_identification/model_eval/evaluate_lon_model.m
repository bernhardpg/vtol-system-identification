clc; clear all; close all;

maneuver_types = "pitch_211";
data_type = "val";
load_data;
load_const_params;

% Load initial guesses
equation_error_results_lon;
x0 = [
     c_X_0, c_X_w, c_X_w_sq, c_X_q, c_X_q_sq, c_X_delta_e,...
     c_Z_0, c_Z_w, c_Z_w_sq, c_Z_delta_e,...
     c_m_0, c_m_w, c_m_q, c_m_delta_e, c_m_delta_e_sq,...
     ];
x = x0; % TODO: Replace with real params

all_params = [const_params;
              x'];

% Collect recorded data
t_seq = t;
y_lon_seq = [theta q u w];
y_lat_seq = [phi, psi, p, r, v];
input_seq = [delta_a, delta_e, delta_r, n_p]; % Actuator dynamics simulated beforehand

% Get data for desired maneuver
maneuver_i = 1;
[t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_sp_m, delta_e_sp_m, delta_r_sp_m, delta_a_m, delta_e_m, delta_r_m, n_p_m]...
    = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_e_sp, delta_r_sp, delta_a, delta_e, delta_r, n_p);
input_seq_m = [delta_a_m delta_e_m delta_r_m n_p_m];
lat_state_seq_m = [phi_m, psi_m, p_m, r_m, v_m];
maneuver_seq_m = [t_m input_seq_m lat_state_seq_m];

% Integrate dynamics
y0 = [theta_m(1) q_m(1) u_m(1) w_m(1)];
tspan = [t_m(1) t_m(end)];

% Integrate dynamics
[t_pred, y_pred] = ode45(@(t,y) lon_dynamics_c(t, y, maneuver_seq_m, all_params), tspan, y0);
y_pred = interp1(t_pred, y_pred, tspan(1):dt:tspan(2));

plot_maneuver("maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_e_m, delta_r_m,  delta_a_sp_m, delta_e_sp_m, delta_r_sp_m, n_p_m,...
    t_m, y_pred,...
    false, true, "");



xs = readmatrix("lon_params.txt");
xs = rmoutliers(xs);
chosen_params = median(xs);
param_mads = mad(xs);

%%

figure
num_params = length(x0);
for i = 1:num_params
    subplot(3,5,i)
    errorbar(0,chosen_params(i),param_mads(i),'-s','MarkerSize',10,...
        'MarkerEdgeColor','red','MarkerFaceColor','red')
    title(param_names(i));
end
