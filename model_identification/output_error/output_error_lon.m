clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lon.mat");

% Load equation_error parameters which will be used as initial guesses
load("model_identification/equation_error/results/equation_error_coeffs_lon.mat");

maneuver_types = "pitch_211";

lon_model = NonlinearModel(equation_error_coeffs_lon, zeros(6,3));
%params_to_update = [1:5 6 7 10 11 12 14 15];
params_to_update = [1:15];
opt_problem = OutputErrorProblem("longitudinal", fpr_data_lon, lon_model, maneuver_types, params_to_update);

tic
opt_problem = opt_problem.solve();
toc

output_error_coeffs_lon_all_free = opt_problem.ModelSpecific.curr_params;
save("model_identification/output_error/results/output_error_coeffs_lon_all_free.mat", "output_error_coeffs_lon_all_free");

output_error_lon_variances = opt_problem.OptData.CramerRaoLowerBound;
save("model_identification/output_error/results/output_error_lon_variances.mat", "output_error_lon_variances");

variances = zeros(18,1);
for i = 1:length(params_to_update)
    variances(params_to_update(i)) = output_error_lon_variances(i);
end

plot(variances)