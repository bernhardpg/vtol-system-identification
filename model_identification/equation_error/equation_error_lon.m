clc; clear all; close all;
maneuver_types = ["pitch_211"];
data_type = "train";
load_data;

% Create explanatory variables
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
t_plot = 0:dt:length(t)*dt-dt;

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

regr = [u_hat w_hat q_hat delta_e]; % Basis regressors
regr_names = ["u" "w" "q" "delta_e"];
use_cross_terms = false;

clc
z = c_X; % output (= dependent variable)
[th_hat, th_names, y_hat, s_sq] = stepwise_regression(z, regr, regr_names, use_cross_terms);
print_eq_error_params("c_X", th_hat, th_names);
disp("s_sq = " + s_sq);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_X")

z = c_Z; % output (= dependent variable)
[th_hat, th_names, y_hat, s_sq] = stepwise_regression(z, regr, regr_names, use_cross_terms);
print_eq_error_params("c_Z", th_hat, th_names);
disp("s_sq = " + s_sq);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_Z")

z = c_m; % output (= dependent variable)
[th_hat, th_names, y_hat, s_sq] = stepwise_regression(z, regr, regr_names, use_cross_terms);
print_eq_error_params("c_m", th_hat, th_names);
disp("s_sq = " + s_sq);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_m")
