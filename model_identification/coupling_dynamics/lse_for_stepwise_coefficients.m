clc; close all; clear all;

format long

% For convenience, this script is very manual, as it only calculates one
% parameter. It essentially orthogonalizes c_m with the final decoupled aerodynamic
% model, and finds the nonlinear regressor r = f(delta_r) that is the most
% correlated with the orthogonal output. Next, it computes the coefficient
% based on c_m. Data from yaw_211 is used to find this coupling.


% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lat.mat");

min_r_sq_change = 2; % Demand at least X% improvement to add a regressor

maneuver_types = [
    "yaw_211",...
    ];

dt = fpr_data_lat.training.roll_211(1).Dt;

dependent_variable_names = ["C_m"];
independent_variable_names = ["DeltaR"];
regr_names = ["delta_r"];
nonlin_regr_names = ["delta_r_abs" "delta_r_sq" "delta_r_abs_sqrt"];

%%%
% Load training data
%%%

% Orthogonalize c_m with previously found model
c_m = collect_data_from_multiple_maneuvers(fpr_data_lat.training, maneuver_types, "C_m");
independent_variable_names = ["Alpha" "AngQHat" "DeltaE"];
regr = ones(length(c_m),1);
for data_type = independent_variable_names
    regr = [regr collect_data_from_multiple_maneuvers(fpr_data_lat.training, maneuver_types, data_type)];
end
aerodynamic_coeffs;
coeffs = [c_m_0 c_m_alpha c_m_q_hat c_m_delta_e];

% Predict c_m with model and remove all data which can be predicted
c_m_hat = regr * coeffs';
z = c_m - c_m_hat;

% Load new regressors
delta_r = collect_data_from_multiple_maneuvers(fpr_data_lat.training, maneuver_types, "DeltaR");
regr = [delta_r];
nonlin_regr = [abs(delta_r) delta_r.^2 sqrt(abs(delta_r))];


% Validation data
% Orthogonalize c_m with previously found model
c_m_val = collect_data_from_multiple_maneuvers(fpr_data_lat.validation, maneuver_types, "C_m");
independent_variable_names = ["Alpha" "AngQHat" "DeltaE"];
regr_val = ones(length(c_m_val),1);
for data_type = independent_variable_names
    regr_val = [regr_val collect_data_from_multiple_maneuvers(fpr_data_lat.validation, maneuver_types, data_type)];
end
aerodynamic_coeffs;
coeffs = [c_m_0 c_m_alpha c_m_q_hat c_m_delta_e];

% Predict c_m with model and remove all data which can be predicted
c_m_hat_val = regr_val * coeffs';
z_val = c_m_val - c_m_hat_val;

% Load new regressors
delta_r_val = collect_data_from_multiple_maneuvers(fpr_data_lat.validation, maneuver_types, "DeltaR");
regr_val = [delta_r_val];
nonlin_regr_val = [abs(delta_r_val) delta_r_val.^2 sqrt(abs(delta_r_val))];

%%%%%
% Options for plotting
maneuver_end_indices = get_maneuver_end_indices(fpr_data_lat.validation, maneuver_types);
first_maneuver_index = 1;
last_maneuver_index = 3;
total_maneuvers_length = (maneuver_end_indices(last_maneuver_index+1) - maneuver_end_indices(first_maneuver_index));
t_plot = 0:dt:total_maneuvers_length*dt-dt;

% Find most correlated regressor with orthogonalized output
all_regressors = [regr nonlin_regr];
all_regressor_names = [regr_names nonlin_regr_names];
regressor_correlations = calc_corr_coeff(all_regressors, z);
disp("Correlations:")
disp(regressor_correlations)
disp("delta_r_abs and delta_r_sq are the most correlated, with similar magnitudes.")
disp("Pick the simplest regressor which is delta_r_sq")

% Calculate regressor coefficient
c_m_delta_r_sq = LSE(delta_r.^2, z);
disp("c_m_delta_r_sq = ");
c_m_delta_r_sq;


plot(z); hold on
plot(c_m_delta_r_sq * delta_r.^2)
%% Function borrowed from stepwise_regression.m

function [r] = calc_corr_coeff(X, z)
    X_bar = mean(X);
    z_bar = mean(z);
    
    cov_Xz = (X - X_bar)' * (z - z_bar);
    var_X = diag((X - X_bar)' * (X - X_bar));
    var_z = diag((z - z_bar)' * (z - z_bar));
    r = cov_Xz ./ sqrt(var_X * var_z);
end