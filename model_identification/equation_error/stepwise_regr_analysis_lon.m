clc; close all; clear all;

%%%
% This script calculates the best model for lateral coeffs using
% stepwise-regression. This is only based on the data. If this method gives
% a simpler model, this model is preferred!

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lon.mat");

% Collect data from multiple experiments into one long dataset
% Note: Ordering does not matter for equation error

min_r_sq_change = 2; % Demand at least X% improvement to add a regressor

maneuver_types = [
    "pitch_211",...
    ];

dt = fpr_data_lon.training.pitch_211(1).Dt;

dependent_variable_names = ["C_L" "C_D" "C_m"];
independent_variable_names = ["Alpha" "AngQHat" "DeltaE"];
regr_names = ["alpha" "q_hat" "delta_e"];
nonlin_regr_names = ["alpha_sq" "alpha_q_hat" "alpha_delta_e"];

%%%
% Load training data
%%%

zs = {};
for data_type = dependent_variable_names
    zs.(data_type) = collect_data_from_multiple_maneuvers(fpr_data_lon.training, maneuver_types, data_type);
end

regr = [];
for data_type = independent_variable_names
    regr = [regr collect_data_from_multiple_maneuvers(fpr_data_lon.training, maneuver_types, data_type)];
end

[nonlin_regr] = create_nonlin_regressors_lon(regr);

%%%
% Load validation data
%%%
zs_val = {};
for data_type = dependent_variable_names
    zs_val.(data_type) = collect_data_from_multiple_maneuvers(fpr_data_lon.validation, maneuver_types, data_type);
end

regr_val = [];
for data_type = independent_variable_names
    regr_val = [regr_val collect_data_from_multiple_maneuvers(fpr_data_lon.validation, maneuver_types, data_type)];
end

[nonlin_regr_val] = create_nonlin_regressors_lon(regr_val);

%%%%%
% Options for plotting
maneuver_end_indices = get_maneuver_end_indices(fpr_data_lon.validation, maneuver_types);
first_maneuver_index = 1;
last_maneuver_index = 3;
total_maneuvers_length = (maneuver_end_indices(last_maneuver_index+1) - maneuver_end_indices(first_maneuver_index));
t_plot = 0:dt:total_maneuvers_length*dt-dt;

% For coefficient storage
std_regr_order_lon = ["bias" "alpha" "alpha_sq" "q_hat" "delta_e" "alpha_delta_e"];


%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

z = zs.C_D;
z_val = zs_val.C_D;
[th_hat, th_names, c_D_hat, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_D", th_hat, th_names);

% Store coeff values
c_D = create_coeff_vector(std_regr_order_lon, th_hat, th_names);

z = zs.C_L;
z_val = zs_val.C_L;
[th_hat, th_names, c_L_hat, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("C_L", th_hat, th_names);

% Store coeff values
c_L = create_coeff_vector(std_regr_order_lon, th_hat, th_names);

z = zs.C_m;
z_val = zs_val.C_m;
[th_hat, th_names, c_m_hat, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("C_m", th_hat, th_names);

% Store coeff values
c_m = create_coeff_vector(std_regr_order_lon, th_hat, th_names);

% Save coeffs to file
equation_error_coeffs_lon = [c_D c_L c_m];

save("model_identification/equation_error/results/equation_error_coeffs_lon.mat", "equation_error_coeffs_lon");

% Plot coefficients
coeff_names = ["$c_D$" "$c_L$" "$c_m$"];
recorded_values = [zs_val.C_D zs_val.C_L zs_val.C_m];
predicted_values = [c_D_hat c_L_hat c_m_hat];
plot_coeffs(t_plot, predicted_values, recorded_values, first_maneuver_index, last_maneuver_index, ...
    maneuver_end_indices, coeff_names,"Longitudinal Equation-Error One-Step Coefficient Prediction (Elevator Deflections)",...
    "longitudinal")

function [nonlin_regr] = create_nonlin_regressors_lon(regr)
    nonlin_regr = [regr(:,1).^2 regr(:,1).*regr(:,2) regr(:,1).*regr(:,3)];
end