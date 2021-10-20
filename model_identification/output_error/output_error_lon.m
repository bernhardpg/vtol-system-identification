clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lon.mat");

maneuver_types = "pitch_211";

%%
%%%%%%%%%%%
% MODEL 1
% Model from equation-error with Equation-Error as initial guess
%%%%%%%%%%%

% Load equation_error parameters which will be used as initial guesses
load("model_identification/equation_error/results/equation_error_coeffs_lon.mat");

lon_model = NonlinearModel(equation_error_coeffs_lon, zeros(6,3));
regularization = 0;
state_sizes = [21 4 1.2 0.4];
weights = diag(state_sizes)^-1;
params_to_update = [1:6 7:9 11 13 14 16 17];
opt_problem = OutputErrorProblem("longitudinal", fpr_data_lon, lon_model, maneuver_types, params_to_update, regularization, weights);
opt_problem = opt_problem.solve();

output_error_lon_coeffs = opt_problem.ModelSpecific.curr_params;
save("model_identification/output_error/results/output_error_lon_coeffs.mat", "output_error_lon_coeffs");
output_error_lon_cr_bounds = zeros(18,1);
for i = 1:length(params_to_update)
    output_error_lon_cr_bounds(params_to_update(i)) = opt_problem.OptData.CramerRaoLowerBound(i);
end
save("model_identification/output_error/results/output_error_lon_cr_bounds.mat", "output_error_lon_cr_bounds");
