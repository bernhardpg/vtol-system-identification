clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

maneuver_types = ["yaw_211"];
data_type = "val";
load_data;
load_const_params;

% Generate plot of all validation maneuvers
plot_output_location = "model_identification/model_validation/validation_plots/lon_model/yaw_maneuvers/";
save_plot = true;
show_plot = false;

test_initial = false;

if test_initial
    % Load initial guesses
    equation_error_results_lon;
    x_lon = [c_D_0 c_D_alpha c_D_alpha_sq c_D_q c_D_delta_e c_L_0 c_L_alpha c_L_alpha_sq c_L_q c_L_delta_e c_m_0 c_m_alpha c_m_q c_m_delta_e c_m_delta_e_sq];
else
    xs = readmatrix("lon_params_ga.txt");
    %xs = rmoutliers(xs);
    x_lon = median(xs);
    param_mads = mad(xs);
end

%% Plot distributions
n_bins = 40;

[~, n_params] = size(xs);
figure
for i = 1:n_params
    subplot(5,round(n_params/5),i)
    histogram(xs(:,i), n_bins);
    xlim(calc_bounds(x_lon(i), 0.5));
    %title(param_names(i));
end

%% Plot error bars
figure

param_i = 1;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{D0}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 2;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{D\alpha}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 3;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{D\alpha^2}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 4;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{Dq}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 5;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{D\delta_e}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 6;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{L0}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 7;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{L\alpha}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 8;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{L\alpha^2}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 9;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{Lq}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 10;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{L\delta_e}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 11;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{m0}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 12;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{m\alpha}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 13;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{mq}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 14;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{m\delta_e}");
ylim(calc_bounds(x_lon(param_i), 0.5));

param_i = 15;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{m\delta_e^2}");
ylim(calc_bounds(x_lon(param_i), 0.5));

%% Generate trajectory plots on validation data
all_params = [const_params;
              x_lon'];

% Collect recorded data
t_seq = t;
y_lon_seq = [theta q u w];
y_lat_seq = [phi, psi, p, r, v];
input_seq = [delta_a, delta_vr, delta_vl, n_p]; % Actuator dynamics simulated beforehand

R_sq = zeros(num_maneuvers, 4);
for maneuver_i = 1:num_maneuvers
    % Get data for desired maneuver
    [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, delta_a_m, delta_vl_m, delta_vr_m, n_p_m]...
        = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_vl_sp, delta_vr_sp, delta_a, delta_vl, delta_vr, n_p);
    input_seq_m = [delta_a_m delta_vl_m delta_vr_m n_p_m];
    lat_state_seq_m = [phi_m, psi_m, p_m, r_m, v_m];
    maneuver_seq_m = [t_m input_seq_m lat_state_seq_m];

    % Integrate dynamics
    y0 = [theta_m(1) q_m(1) u_m(1) w_m(1)];
    tspan = [t_m(1) t_m(end)];

    % Integrate dynamics
    disp("Simulating dynamics for maneuver " + maneuver_i);
    tic
    [t_pred, y_pred] = ode23s(@(t,y) lon_dynamics_liftdrag_c(t, y, maneuver_seq_m, all_params), tspan, y0);
    toc
    y_pred = interp1(t_pred, y_pred, t_m);

    R_sq_theta = calc_R_sq(theta_m, y_pred(:,1));
    R_sq_q = calc_R_sq(q_m, y_pred(:,2));
    R_sq_u = calc_R_sq(u_m, y_pred(:,3));
    R_sq_w = calc_R_sq(w_m, y_pred(:,4));
    R_sq_m = [R_sq_theta R_sq_q R_sq_u R_sq_w];
    R_sq(maneuver_i,:) = R_sq_m;
    
    if save_plot || show_plot
        plot_maneuver("val_maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_vl_m, delta_vr_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, n_p_m,...
            t_m, y_pred,...
            save_plot, show_plot, plot_output_location, R_sq_m);
    end
end


%% TODO remove this
xs = readmatrix("lon_params_free.txt");
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

function [bounds] = calc_bounds(param, allowed_param_change)
    LB = min([param * (1 - allowed_param_change); param * (1 + allowed_param_change)]);
    UB = max([param * (1 - allowed_param_change); param * (1 + allowed_param_change)]);
    bounds = [LB UB];
end
