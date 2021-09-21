clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lat.mat");

% Load equation_error parameters which will be used as initial guesses
load("model_identification/equation_error/results/equation_error_coeffs_lat.mat");
%load("avl_model/avl_results/avl_coeffs_lat.mat");

% equation_error_coeffs_lat(1) = 0;
% equation_error_coeffs_lat(7) = 0;
% equation_error_coeffs_lat(13) = 0;

% initial_coeffs = equation_error_coeffs_lat;
% coeffs_to_keep_from_avl = [];
% for i = coeffs_to_keep_from_avl
%    initial_coeffs(i) = avl_coeffs_lat(i);
% end

% Lateral system
% State = [v p r phi]
% Input = [delta_a delta_r]

maneuver_types = [
    "roll_211",...
    "yaw_211",...
    ];

curr_coeffs_lat = equation_error_coeffs_lat;
lat_model = NonlinearModel(zeros(5,3), equation_error_coeffs_lat);
params_to_update = [1:18];
opt_problem = OutputErrorProblem("lateral-directional", fpr_data_lat, lat_model, maneuver_types, params_to_update);

tic
opt_problem = opt_problem.solve();
toc

output_error_coeffs_lat = opt_problem.OptData.params.CoeffsLat;
save("model_identification/output_error/results/output_error_coeffs_lat.mat", "output_error_coeffs_lat");

output_error_variances = opt_problem.OptData.CramerRaoLowerBound;
save("model_identification/output_error/results/output_error_variances.mat", "output_error_variances");

variances = zeros(18,1);
for i = 1:length(params_to_update)
    variances(params_to_update(i)) = output_error_variances(i);
end

plot(variances)