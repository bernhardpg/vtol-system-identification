clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

maneuver_types = ["yaw_211"];
data_type = "val";
load_data;
load_const_params;

% Generate plot of all validation maneuvers
plot_output_location = "model_identification/model_validation/validation_plots/lat_model";
save_plot = false;
show_plot = true;
plot_height = 1.0;

test_initial = true;

if test_initial
    % Load initial guesses
    equation_error_results_lat;
    x_lat = [c_Y_0 c_Y_beta c_Y_p c_Y_delta_a c_Y_delta_r c_l_0 c_l_beta c_l_p c_l_r c_l_delta_a c_n_0 c_n_beta c_n_p c_n_r c_n_delta_r];
else
%     xs = readmatrix("lon_params_ga.txt");
%     %xs = rmoutliers(xs);
%     x_lon = median(xs);
%     param_mads = mad(xs);
%     writematrix(x_lon, "lon_params_medians.txt");
%     x_lon = [x_lon -0.3];
end

%% Plot distributions
n_bins = 40;

[~, n_params] = size(xs);
figure
for i = 1:n_params
    subplot(5,round(n_params/5),i)
    histogram(xs(:,i), n_bins);
    xlim(calc_bounds(x_lon(i), plot_height));
    %title(param_names(i));
end

%% Plot error bars
avl_stability_derivatives;

figure

param_i = 1;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{D0}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 2;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{D\alpha}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 3;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{D\alpha^2}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 4;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{Dq}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 5;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{D\delta_e}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 6;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
hold on
scatter(1, avl_c_L_0, 'x');
xlim([-1 2])
set(gca,'xticklabel',{[]})
title("c_{L0}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 7;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red');
hold on
scatter(1, avl_c_L_alpha, 'x');
xlim([-1 2])
set(gca,'xticklabel',{[]})
title("c_{L\alpha}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 8;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{L\alpha^2}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 9;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{Lq}");
scatter(1, avl_c_L_q, 'x');
xlim([-1 2])
set(gca,'xticklabel',{[]})
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 10;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
hold on
scatter(1, avl_c_L_delta_e, 'x');
xlim([-1 2])
set(gca,'xticklabel',{[]})
title("c_{L\delta_e}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 11;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{m0}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 12;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
hold on
scatter(1, avl_c_m_alpha, 'x');
xlim([-1 2])
set(gca,'xticklabel',{[]})
title("c_{m\alpha}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 13;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
hold on
scatter(1, avl_c_m_q, 'x');
xlim([-1 2])
set(gca,'xticklabel',{[]})
title("c_{mq}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 14;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
hold on
scatter(1, avl_c_m_delta_e, 'x');
xlim([-1 2])
set(gca,'xticklabel',{[]})
title("c_{m\delta_e}");
ylim(calc_bounds(x_lon(param_i), plot_height));

param_i = 15;
subplot(3,5,param_i)
errorbar(0,x_lon(param_i),param_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red')
title("c_{m\delta_e^2}");
ylim(calc_bounds(x_lon(param_i), plot_height));

%% Generate trajectory plots on validation data
all_params = [const_params;
              x_lat'];

% Collect recorded data
t_seq = t;
y_lon_seq = [theta q u w];
input_seq = [delta_a, delta_vr, delta_vl, n_p]; % Actuator dynamics simulated beforehand

num_variables_to_pred = 5;
R_sq = zeros(num_maneuvers, num_variables_to_pred);
for maneuver_i = 1
    % Get data for desired maneuver
    [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, delta_a_m, delta_vl_m, delta_vr_m, n_p_m, p_dot_m, q_dot_m, r_dot_m]...
    = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_vl_sp, delta_vr_sp, delta_a, delta_vl, delta_vr, n_p, p_dot, q_dot, r_dot);
    input_seq_m = [delta_a_m delta_vl_m delta_vr_m n_p_m];
    lon_state_seq_m = [theta_m q_m u_m w_m];
    maneuver_seq_m = [t_m input_seq_m lon_state_seq_m];

    % Integrate dynamics
    y0 = [phi_m(1) psi_m(1) p_m(1) r_m(1) v_m(1)];
    tspan = [t_m(1) t_m(end)];

    % Integrate dynamics
    disp("Simulating dynamics for maneuver " + maneuver_i);
    tic
    [t_pred, y_pred] = ode45(@(t,y) lat_dynamics_liftdrag_c(t, y, maneuver_seq_m, all_params), tspan, y0);
    y_pred = interp1(t_pred, y_pred, t_m);
    acc = calc_acc_lat(y_pred, maneuver_seq_m, all_params);
    toc
    

    R_sq_phi = calc_R_sq(phi_m, y_pred(:,1));
    R_sq_psi = calc_R_sq(psi_m, y_pred(:,2));
    R_sq_p = calc_R_sq(p_m, y_pred(:,3));
    R_sq_r = calc_R_sq(r_m, y_pred(:,4));
    R_sq_v = calc_R_sq(v_m, y_pred(:,4));
    R_sq_man = [R_sq_phi R_sq_psi R_sq_p R_sq_r R_sq_v];
    R_sq(maneuver_i,:) = R_sq_man;
    

    if save_plot || show_plot
        plot_maneuver_lat("traj_val_maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_vl_m, delta_vr_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, n_p_m, a_x_m, a_y_m, a_z_m, p_dot_m, q_dot_m, r_dot_m,...
            t_m, [y_pred acc],...
            save_plot, show_plot, plot_output_location, R_sq_man);
        plot_coeffs_lat("coeffs_val_maneuver" + maneuver_i, x_lat, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_vl_m, delta_vr_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, n_p_m, a_x_m, a_y_m, a_z_m, p_dot_m, q_dot_m, r_dot_m,...
            t_m, [y_pred acc],...
            save_plot, show_plot, plot_output_location, R_sq_man);
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

function [bounds] = calc_bounds(param, allowed_param_change)
    LB = min([param * (1 - allowed_param_change); param * (1 + allowed_param_change)]);
    UB = max([param * (1 - allowed_param_change); param * (1 + allowed_param_change)]);
    bounds = [LB UB];
end
