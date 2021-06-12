clc; clear all; close all;
% Load training data
maneuver_types = ["pitch_211" "yaw_211"];
data_type = "train";
load_data;

% Create explanatory variables
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
t_plot = 0:dt:length(t)*dt-dt;

% Basis regressors
regr = [u_hat w_hat q_hat delta_e delta_r]; % Include delta_r as lon model is actually affect by this due to aircraft design
regr_names = ["u" "w" "q" "delta_e" "delta_r"];

% Dependent variables
zs = [c_X c_Z c_m];

% Load validation data
maneuver_types = ["pitch_211" "yaw_211"];
data_type = "val";
load_data;

[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);

regr_val = [u_hat w_hat q_hat delta_e delta_r];
zs_val = [c_X c_Z c_m];

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

min_r_sq_change = 2; % Demand at least 2% improvement to add a regressor

clc
z = zs(:,1);
z_val = zs_val(:,1);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_X", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_X")


z = zs(:,2);
z_val = zs_val(:,2);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_Z", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_Z")

z = zs(:,3);
z_val = zs_val(:,3);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_m", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_m")
