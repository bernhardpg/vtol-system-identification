clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

maneuver_types = ["roll_211"];
data_type = "val";
load_data;
load_const_params;

% Generate plot of all validation maneuvers
plot_output_location = "model_identification/model_validation/validation_plots/lat_model_step/";
traj_plot_output_location = plot_output_location + "roll1/";
save_plot = true;
show_plot = true;
plot_height = 1;

validation_type = "steps";

if validation_type == "initial"
    % Load initial guesses
    equation_error_results_lat;
    x_lat_roll = [c_Y_0 c_Y_beta c_Y_p c_Y_r c_Y_delta_a c_Y_delta_r c_l_0 c_l_beta c_l_p c_l_r c_l_delta_a c_l_delta_r c_n_0 c_n_beta c_n_p c_n_r c_n_delta_a c_n_delta_r];
elseif validation_type == "deviations"
    % Load all parameters from roll maneuvers to plot deviations
    params_location = "model_identification/output_error/results/lat/";
    xs_roll = readmatrix(params_location + "lat_params_roll.txt");
    %xs_roll = rmoutliers(xs_roll);
    x_lat_roll = median(xs_roll);
    x_lat_roll_mads = mad(xs_roll);
    writematrix(x_lat_roll, params_location + "lat_params_roll_medians.txt");
    
    % Load all parameters from yaw maneuvers to plot deviations
    params_location = "model_identification/output_error/results/lat/";
    xs_yaw = readmatrix(params_location + "lat_params_yaw.txt");
    %xs_yaw = rmoutliers(xs_yaw);
    x_lat_yaw = median(xs_yaw);
    x_lat_yaw_mads = mad(xs_yaw);
    writematrix(x_lat_yaw, params_location + "lat_params_yaw_medians.txt");
elseif validation_type == "steps"
    %%%%%%%
    %%% Step 1:
    %%%%%%%
    % Load parameters from roll maneuvers
    % c_Y_delta_a c_l_0 c_l_p c_l_r c_l_delta_a c_n_p c_n_delta_a
    params_location = "model_identification/output_error/results/lat/";
    params_type = "lat_params_step_roll";
    xs = readmatrix(params_location + params_type + ".txt");
    xs = rmoutliers(xs);
    x = median(xs);
    x_mads = mad(xs);
    writematrix(x, params_location + params_type + "_medians.txt");
    
    equation_error_results_lat;
    x_lat_roll = [c_Y_0 c_Y_beta c_Y_p c_Y_r x(1) c_Y_delta_r...
                  x(2) c_l_beta x(3) x(4) x(5) c_l_delta_r...
                  c_n_0 c_n_beta x(6) c_n_r x(7) c_n_delta_r];
    
    x_lat_roll_mads = [0 0 0 0 x_mads(1) 0 ...
                       x_mads(2) 0 x_mads(3) x_mads(4) x_mads(5) 0 ...
                       0 0 x_mads(6) 0 x_mads(7) 0];
    x_lat_yaw = x_lat_roll;
    x_lat_yaw_mads = x_lat_roll_mads;
end

param_names = ["c_Y_0" "c_Y_beta" "c_Y_p" "c_Y_r" "c_Y_delta_a" "c_Y_delta_r"...
               "c_l_0" "c_l_beta" "c_l_p" "c_l_r" "c_l_delta_a" "c_l_delta_a"...
               "c_n_0" "c_n_beta" "c_n_p" "c_n_r" "c_n_delta_a" "c_n_delta_r"];


%% Plot distributions
n_bins = 40;

% Plot params from roll maneuvers
[~, n_params] = size(xs_roll);
fig = figure;
for i = 1:n_params
    subplot(5,round(n_params/5),i)
    histogram(xs_roll(:,i), n_bins);
    xlim(calc_bounds(x_lat_roll(i), plot_height));
    title(param_names(i));
end
sgtitle("Parameter Distributions from Roll maneuvers")
if save_plot
    filename = "param_dists_roll";
    mkdir(plot_output_location);
    saveas(fig, plot_output_location + filename, 'epsc')
end

% Plot params from roll maneuvers
[~, n_params] = size(xs_yaw);
fig = figure;
for i = 1:n_params
    subplot(5,round(n_params/5),i)
    histogram(xs_yaw(:,i), n_bins);
    xlim(calc_bounds(x_lat_roll(i), plot_height));
    title(param_names(i));
end
sgtitle("Parameter Distributions from Yaw maneuvers")
if save_plot
    filename = "param_dists_yaw";
    mkdir(plot_output_location);
    saveas(fig, plot_output_location + filename, 'epsc')
end
%% Plot error bars for roll vs yaw
avl_stability_derivatives;

fig = figure;
if ~show_plot
    fig.Visible = 'off';
end
fig.Position = [100 100 800 800];

param_i = 1;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{Y0}");
xlim([-1 3])
set(gca,'xticklabel',{[]})
ylim(calc_bounds(x_lat_roll(param_i), plot_height));

param_i = 2;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{Y\beta}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
scatter(2, avl_c_Y_beta, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})

param_i = 3;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{Yp}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
scatter(2, avl_c_Y_p, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})

param_i = 4;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{Yr}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
scatter(2, avl_c_Y_r, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})

param_i = 5;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{Y\delta_a}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
scatter(2, avl_c_Y_delta_a, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})

param_i = 6;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{Y\delta_r}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
scatter(2, avl_c_Y_delta_r, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})

param_i = 7;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
xlim([-1 3])
set(gca,'xticklabel',{[]})
title("c_{l0}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));

param_i = 8;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
scatter(2, avl_c_l_beta, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})
title("c_{l\beta}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));

param_i = 9;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{lp}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
scatter(2, avl_c_l_p, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})

param_i = 10;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{lr}");
xlim([-1 3])
set(gca,'xticklabel',{[]})
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
scatter(2, avl_c_l_r, 'x');

param_i = 11;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
hold on
scatter(2, avl_c_l_delta_a, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})
title("c_{l\delta_a}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));

param_i = 12;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
hold on
scatter(2, avl_c_l_delta_r, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})
title("c_{l\delta_r}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));

param_i = 13;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{n0}");
xlim([-1 3])
set(gca,'xticklabel',{[]})
ylim(calc_bounds(x_lat_roll(param_i), plot_height) * 5);

param_i = 14;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
xlim([-1 3])
set(gca,'xticklabel',{[]})
title("c_{n\beta}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
scatter(2, avl_c_n_beta, 'x');

param_i = 15;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
xlim([-1 3])
set(gca,'xticklabel',{[]})
title("c_{np}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
scatter(2, avl_c_n_p, 'x');

param_i = 16;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
scatter(2, avl_c_n_r, 'x');
xlim([-1 3])
set(gca,'xticklabel',{[]})
title("c_{nr}");
ylim(calc_bounds(x_lat_roll(param_i), plot_height));

param_i = 17;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on;
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{n\delta_a}");
scatter(2, avl_c_n_delta_a, 'x');
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
xlim([-1 3])
set(gca,'xticklabel',{[]})

param_i = 18;
subplot(3,6,param_i)
errorbar(0,x_lat_roll(param_i),x_lat_roll_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on;
errorbar(1,x_lat_yaw(param_i),x_lat_yaw_mads(param_i),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red','MarkerFaceColor','red'); hold on
title("c_{n\delta_r}");
scatter(2, avl_c_n_delta_r, 'x');
ylim(calc_bounds(x_lat_roll(param_i), plot_height));
xlim([-1 3])
set(gca,'xticklabel',{[]})

legend("Roll 2-1-1","Yaw 2-1-1","AVL",'Location','southeastoutside')

sgtitle("Lateral Parameters Deviations")

if save_plot
    filename = "param_deviations";
    mkdir(plot_output_location);
    saveas(fig, plot_output_location + filename, 'epsc')
end

%% Generate trajectory plots on validation data
all_params = [const_params;
              x_lat_roll'];

% Collect recorded data
t_seq = t;
y_lon_seq = [theta q u w];
input_seq = [delta_a, delta_vr, delta_vl, n_p]; % Actuator dynamics simulated beforehand

num_variables_to_pred = 5;
R_sq = zeros(num_maneuvers, num_variables_to_pred);
for maneuver_i = 1:num_maneuvers
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
    R_sq_v = calc_R_sq(v_m, y_pred(:,5));
    R_sq_man = [R_sq_phi R_sq_psi R_sq_p R_sq_r R_sq_v];
    R_sq(maneuver_i,:) = R_sq_man;
    
    if save_plot || show_plot
        plot_maneuver_lat("traj_val_maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_vl_m, delta_vr_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, n_p_m, a_x_m, a_y_m, a_z_m, p_dot_m, q_dot_m, r_dot_m,...
            t_m, [y_pred acc],...
            save_plot, show_plot, traj_plot_output_location, R_sq_man);
        plot_coeffs_lat("coeffs_val_maneuver" + maneuver_i, x_lat_roll, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_vl_m, delta_vr_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, n_p_m, a_x_m, a_y_m, a_z_m, p_dot_m, q_dot_m, r_dot_m,...
            t_m, [y_pred acc],...
            save_plot, show_plot, traj_plot_output_location, R_sq_man);
    end
end


%% TODO remove this
xs_roll = readmatrix("lon_params_free.txt");
xs_roll = rmoutliers(xs_roll);
x = median(xs_roll);
x_lat_roll_mads = mad(xs_roll);

%%

figure
num_params = length(x0);
for i = 1:num_params
    subplot(3,6,i)
    errorbar(0,x(i),x_lat_roll_mads(i),'-s','MarkerSize',10,...
        'MarkerEdgeColor','red','MarkerFaceColor','red')
    title(param_names(i));
end

function [bounds] = calc_bounds(param, allowed_param_change)
    LB = min([param * (1 - allowed_param_change); param * (1 + allowed_param_change)]);
    UB = max([param * (1 - allowed_param_change); param * (1 + allowed_param_change)]);
    bounds = [LB UB];
end
