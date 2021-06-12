clc; clear all; close all;
% Load training data
maneuver_types = ["roll_211" "yaw_211"];
data_type = "train";
load_data;

% Create explanatory variables
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
t_plot = 0:dt:length(t)*dt-dt;

% Basis regressors
regr = [p_hat r_hat v_hat delta_a delta_r]; % Basis regressors
regr_names = ["p" "r" "v" "delta_a" "delta_r"];

% Dependent variables
zs = [c_Y c_l c_n];

% Load validation data
maneuver_types = ["roll_211" "yaw_211"];
data_type = "val";
load_data;

[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);

regr_val = [p_hat r_hat v_hat delta_a delta_r]; % Basis regressors
% Dependent variables
zs_val = [c_Y c_l c_n];



%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%


min_r_sq_change = 1; % Demand at least 2% improvement to add a regressor

clc
z = zs(:,1);
z_val = zs_val(:,1);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_Y", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_Y")


z = zs(:,2);
z_val = zs_val(:,2);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_l", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_l")

z = zs(:,3);
z_val = zs_val(:,3);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_n", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_n")
