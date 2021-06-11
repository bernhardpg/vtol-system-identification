clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

maneuver_types = ["pitch_211"];
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

% Process parameters found from output-error
xs = readmatrix("lon_params.txt");
xs = rmoutliers(xs);
x = median(xs);
param_mads = mad(xs);

all_params = [const_params;
              x'];

% Collect recorded data
t_seq = t;
y_lon_seq = [theta q u w];
y_lat_seq = [phi, psi, p, r, v];
input_seq = [delta_a, delta_e, delta_r, n_p]; % Actuator dynamics simulated beforehand

% Generate plot of all validation maneuvers
plot_output_location = "model_identification/model_validation/validation_plots/lon_model/";
save_plot = true;
show_plot = false;

R_sq = zeros(num_maneuvers, 4);
for maneuver_i = 8
    % Get data for desired maneuver
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

    R_sq_theta = calc_R_sq(theta_m, y_pred(:,1));
    R_sq_q = calc_R_sq(q_m, y_pred(:,2));
    R_sq_u = calc_R_sq(u_m, y_pred(:,3));
    R_sq_w = calc_R_sq(w_m, y_pred(:,4));
    R_sq_m = [R_sq_theta R_sq_q R_sq_u R_sq_w];
    R_sq(maneuver_i,:) = R_sq_m;
    
    if save_plot || show_plot
        plot_maneuver("val_maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_e_m, delta_r_m,  delta_a_sp_m, delta_e_sp_m, delta_r_sp_m, n_p_m,...
            t_m, y_pred,...
            save_plot, false, plot_output_location, R_sq_m);
    end
end


%%
xs = readmatrix("lon_params.txt");
xs = rmoutliers(xs);
x = median(xs);
param_mads = mad(xs);

%%

figure
num_params = length(x0);
for i = 1:num_params
    subplot(3,5,i)
    errorbar(0,x(i),param_mads(i),'-s','MarkerSize',10,...
        'MarkerEdgeColor','red','MarkerFaceColor','red')
    title(param_names(i));
end

function [R_sq] = calc_R_sq(z, y_hat)
    % Calculate total Sum of Squares
    z_bar = mean(z);
    SS_T = (z - z_bar)' * (z - z_bar);
    SS_E = (z - y_hat)' * (z - y_hat); % Residual Sum of Squares
    
    % Coefficient of Determination
    R_sq = (1 - SS_E/SS_T) * 100;
end