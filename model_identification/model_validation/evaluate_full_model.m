clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

maneuver_types = ["roll_211" "yaw_211"];
data_type = "train";
load_data;
load_const_params;

% Load initial guesses
equation_error_results_lat;
x_lon = [-0.1074    0.1720    2.5516  -10.0212  535.1918   -0.0493   -0.4894   -5.4394    3.3249   -0.5300    0.0115   -1.3611  -22.6064   -0.8852   -0.5934];

x0_lat = [
     c_Y_0, c_Y_p, c_Y_v, c_Y_delta_a, c_Y_delta_r,...
     c_l_0, c_l_p, c_l_r, c_l_v, c_l_delta_a,...
     c_n_0, c_n_p, c_n_r, c_n_v, c_n_delta_r,...
     ];
x_lat = [0.0502    1.5157   -0.7445   -0.1535    0.1872   -0.0075   -0.2793    0.2030   -0.0532    0.1762    0.0001   -0.0851   -0.0987    0.0681   -0.0772];
 
all_params = [const_params;
              x_lon';
              x_lat'];

% Collect recorded data
t_seq = t;
input_seq = [delta_a, delta_e, delta_r, n_p]; % Actuator dynamics simulated beforehand

% Generate plot of all validation maneuvers
plot_output_location = "model_identification/model_validation/validation_plots/full_model/";
save_plot = true;
show_plot = false;

R_sq = zeros(num_maneuvers, 4);
for maneuver_i = 1
    % Get data for desired maneuver
    [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_sp_m, delta_e_sp_m, delta_r_sp_m, delta_a_m, delta_e_m, delta_r_m, n_p_m]...
        = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_e_sp, delta_r_sp, delta_a, delta_e, delta_r, n_p);
    input_seq_m = [t_m delta_a_m delta_e_m delta_r_m n_p_m];

    % Integrate dynamics
    y0 = [phi_m(1) theta_m(1) psi_m(1) p_m(1) q_m(1) r_m(1) u_m(1) v_m(1) w_m(1)];
    tspan = [t_m(1) t_m(end)];

    % Integrate dynamics
    [t_pred, y_pred] = ode45(@(t,y) full_dynamics_c(t, y, input_seq_m, all_params), tspan, y0);
    y_pred = interp1(t_pred, y_pred, tspan(1):dt:tspan(2));

    R_sq_m = zeros(1,4);
    
    if save_plot || show_plot
        plot_maneuver_full("val_maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_e_m, delta_r_m,  delta_a_sp_m, delta_e_sp_m, delta_r_sp_m, n_p_m,...
            t_m, y_pred,...
            false, true, "", R_sq_m);
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
