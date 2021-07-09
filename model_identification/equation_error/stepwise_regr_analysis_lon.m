clear all;
% Load training data
maneuver_types = ["pitch_211"];
data_type = "train";
load_data;

% Basis regressors
regr = [V_hat aoa_alpha q_hat delta_e];
regr_names = ["V" "alpha" "q" "delta_e"];

nonlin_regr = [aoa_alpha.^2];% sign(q_hat).*q_hat.^2 sign(delta_e).*delta_e.^2];
nonlin_regr_names = ["alpha_sq"];% "q_sq" "delta_e_sq"];

% Dependent variables
zs = [c_D c_L c_m];

% Load validation data
data_type = "val";
load_data;

regr_val = [V_hat aoa_alpha q_hat delta_e];
nonlin_regr_val = [aoa_alpha.^2];% sign(q_hat).*q_hat.^2 sign(delta_e).*delta_e.^2];
zs_val = [c_D c_L c_m];
t_plot_val = 0:dt:length(t)*dt-dt;

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

min_r_sq_change = 0.5; % Demand at least 2% improvement to add a regressor

z = zs(:,1);
z_val = zs_val(:,1);
[th_hat, th_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_D", th_hat, th_names);

figure
subplot(3,1,1)
plot(t_plot_val, z_val, t_plot_val, y_hat_val); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_D: " + "R^2 = " + R_sq_val + "%")


z = zs(:,2);
z_val = zs_val(:,2);
[th_hat, th_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_L", th_hat, th_names);

subplot(3,1,2)
plot(t_plot_val, z_val, t_plot_val, y_hat_val); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_L: " + "R^2 = " + R_sq_val + "%")

z = zs(:,3);
z_val = zs_val(:,3);
[th_hat, th_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_m", th_hat, th_names);

subplot(3,1,3)
plot(t_plot_val, z_val, t_plot_val, y_hat_val); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_m: " + "R^2 = " + R_sq_val + "%")

sgtitle("Stepwise-Regression Equation-Error Longitudinal Model")