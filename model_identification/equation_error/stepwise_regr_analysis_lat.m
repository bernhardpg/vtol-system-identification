clc; clear all; close all;

%%%
% This script calculates the best model for lateral coeffs using
% stepwise-regression. This is only based on the data. If this method gives
% a simpler model, this model is preferred!

%%%
% Load training data
%%%
maneuver_types = ["roll_211" "yaw_211"];
data_type = "train";
load_data;

% Create explanatory variables
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);

% Basis regressors
regr = [p_hat r_hat beta delta_a delta_r]; % Basis regressors
regr_names = ["p" "r" "beta" "delta_a" "delta_r"];

nonlin_regr = [beta.^2]; %sign(p_hat).*p_hat.^2 sign(r_hat).*r_hat.^2 sign(delta_a).*delta_a.^2 sign(delta_r).*delta_r.^2];
nonlin_regr_names = ["beta_sq"]; % "p_sq" "r_sq" "delta_a_sq" "delta_r_sq"];

% Dependent variables
zs = [c_Y c_l c_n];

%%%
% Load validation data
%%%
data_type = "val";
load_data;

[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);

delta_r = (-delta_vl + delta_vr) / 2;
regr_val = [p_hat r_hat beta delta_a delta_r]; % Basis regressors
nonlin_regr_val = [beta.^2];

% Dependent variables
zs_val = [c_Y c_l c_n];
t_plot_val = 0:dt:length(t)*dt-dt;



%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

min_r_sq_change = 0.5; % Demand at least 2% improvement to add a regressor

z = zs(:,1);
z_val = zs_val(:,1);
[th_hat, th_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_Y", th_hat, th_names);

figure
subplot(3,1,1)
plot(t_plot_val, z_val, t_plot_val, y_hat_val); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_Y: " + "R^2 = " + R_sq_val + "%")


z = zs(:,2);
z_val = zs_val(:,2);
[th_hat, th_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_l", th_hat, th_names);

subplot(3,1,2)
plot(t_plot_val, z_val, t_plot_val, y_hat_val); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_l: " + "R^2 = " + R_sq_val + "%")

z = zs(:,3);
z_val = zs_val(:,3);
[th_hat, th_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_n", th_hat, th_names);

subplot(3,1,3)
plot(t_plot_val, z_val, t_plot_val, y_hat_val); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_n: " + "R^2 = " + R_sq_val + "%")

sgtitle("Equation-error Lat coeffs")