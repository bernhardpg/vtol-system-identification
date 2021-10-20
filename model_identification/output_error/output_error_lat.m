clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lat.mat");
% 
% % Load equation_error parameters which will be used as initial guesses
% load("model_identification/equation_error/results/equation_error_coeffs_lat.mat");
% 
% % Load parameters from AVL
% load("avl_model/avl_results/avl_coeffs_lat.mat");

maneuver_types = [
    "roll_211",...
    "yaw_211",...
    ];

%%
%%%%%%%%%%%
% MODEL 1
% Model from equation-error with Equation-Error as initial guess
%%%%%%%%%%%

% Load equation_error parameters which will be used as initial guesses
load("model_identification/equation_error/results/equation_error_coeffs_lat.mat");

lat_model = NonlinearModel(zeros(5,3), equation_error_coeffs_lat);
params_to_update = [1:3 5:6 7:11 13:16 18];

lambda = 0;
weights = eye(4);
opt_problem = OutputErrorProblem("lateral-directional", fpr_data_lat, lat_model, maneuver_types, params_to_update, lambda, weights);

opt_problem = opt_problem.solve();

output_error_lat_coeffs = opt_problem.ModelSpecific.curr_params;
save("model_identification/output_error/results/output_error_lat_coeffs.mat", "output_error_lat_coeffs");

output_error_lat_cr_bounds = zeros(18,1);
for i = 1:length(params_to_update)
    output_error_lat_cr_bounds(params_to_update(i)) = opt_problem.OptData.CramerRaoLowerBound(i);
end
save("model_identification/output_error/results/output_error_lat_cr_bounds.mat", "output_error_lat_cr_bounds");
