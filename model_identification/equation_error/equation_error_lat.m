clc; clear all; close all;
maneuver_types = ["roll_211" "yaw_211"];
data_type = "train";
load_data;

% Create explanatory variables
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
t_plot = 0:dt:length(t)*dt-dt;

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

regr = [p_hat r_hat v_hat delta_a delta_r]; % Basis regressors
regr_names = ["p" "r" "v" "delta_a" "delta_r"];
use_cross_terms = false;

clc
z = c_Y; % output (= dependent variable)
[th_hat, th_names, y_hat] = stepwise_regression(z, regr, regr_names, use_cross_terms);
print_eq_error_params("c_Y", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_Y")

z = c_l; % output (= dependent variable)
[th_hat, th_names, y_hat] = stepwise_regression(z, regr, regr_names, use_cross_terms);
print_eq_error_params("c_l", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_l")

z = c_n; % output (= dependent variable)
[th_hat, th_names, y_hat] = stepwise_regression(z, regr, regr_names, use_cross_terms);
print_eq_error_params("c_n", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_n")