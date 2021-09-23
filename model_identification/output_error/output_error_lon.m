clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lon.mat");

% Load equation_error parameters which will be used as initial guesses
load("model_identification/equation_error/results/equation_error_coeffs_lon.mat");

% Load some parameters from AVL
load("avl_model/avl_results/avl_coeffs_lon.mat");

maneuver_types = "pitch_211";
%%
%%%%%%%%%%%
% MODEL 1
% Model from equation-error with Equation-Error as initial guess
%%%%%%%%%%%

lon_model = NonlinearModel(equation_error_coeffs_lon, zeros(6,3));
params_to_update = [1:5 6 7 10 11 12 14 15];
opt_problem = OutputErrorProblem("longitudinal", fpr_data_lon, lon_model, maneuver_types, params_to_update);
opt_problem = opt_problem.solve();

output_error_lon_coeffs = opt_problem.ModelSpecific.curr_params;
save("model_identification/output_error/results/output_error_lon_coeffs.mat", "output_error_lon_coeffs");
output_error_lon_cr_bounds = zeros(15,1);
for i = 1:length(params_to_update)
    output_error_lon_cr_bounds(params_to_update(i)) = opt_problem.OptData.CramerRaoLowerBound(i);
end
save("model_identification/output_error/results/output_error_lon_cr_bounds.mat", "output_error_lon_cr_bounds");
%%
%%%%%%%%%%%
% MODEL 2
% Model with initial guess from Equation-Error but with model structure
% free
%%%%%%%%%%%

lon_model = NonlinearModel(equation_error_coeffs_lon, zeros(6,3));
params_to_update = 1:15;
opt_problem = OutputErrorProblem("longitudinal", fpr_data_lon, lon_model, maneuver_types, params_to_update);
opt_problem = opt_problem.solve();

output_error_lon_all_free_coeffs = opt_problem.ModelSpecific.curr_params;
save("model_identification/output_error/results/output_error_lon_all_free_coeffs.mat", "output_error_lon_all_free_coeffs");
output_error_lon_all_free_cr_bounds = zeros(15,1);
for i = 1:length(params_to_update)
    output_error_lon_all_free_cr_bounds(params_to_update(i)) = opt_problem.OptData.CramerRaoLowerBound(i);
end
save("model_identification/output_error/results/output_error_lon_all_free_cr_bounds.mat", "output_error_lon_all_free_cr_bounds");
%%
%%%%%%%%%%%
% MODEL 3
% Model with selective parameters from equation-error and AVL, and some
% allowed to vary
%%%%%%%%%%%

% Keep some coeffs from AVL
initial_coeffs = equation_error_coeffs_lon;
coeffs_to_keep_from_avl = [6 7 9]; % c_L_0, c_L_alpha, c_L_q
for i = coeffs_to_keep_from_avl
   initial_coeffs(i) = avl_coeffs_lon(i);
end
lon_model = NonlinearModel(initial_coeffs, zeros(6,3));
params_to_update = [1  2  3     5 ... % All drag parameters
                                10 ... % c_L_delta_e
                    11 12    14 15]; % c_m_0 c_m_alpha c_m_q_hat c_m_delta_e
opt_problem = OutputErrorProblem("longitudinal", fpr_data_lon, lon_model, maneuver_types, params_to_update);
opt_problem = opt_problem.solve();


output_error_coeffs_lon_final_coeffs = opt_problem.ModelSpecific.curr_params;
save("model_identification/output_error/results/output_error_coeffs_lon_final_coeffs.mat", "output_error_coeffs_lon_final_coeffs");
output_error_coeffs_lon_final_cr_bounds = zeros(15,1);
for i = 1:length(params_to_update)
    output_error_coeffs_lon_final_cr_bounds(params_to_update(i)) = opt_problem.OptData.CramerRaoLowerBound(i);
end
save("model_identification/output_error/results/output_error_coeffs_lon_final_cr_bounds.mat", "output_error_coeffs_lon_final_cr_bounds");