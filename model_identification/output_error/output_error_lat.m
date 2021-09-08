clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lat.mat");

% Load equation_error parameters which will be used as initial guesses
load("model_identification/equation_error/results/equation_error_coeffs_lat.mat");

% Lateral system
% State = [v p r phi]
% Input = [delta_a delta_r]

maneuver_types = [
    "roll_211",...
    "yaw_211",...
    ];

curr_coeffs_lat = equation_error_coeffs_lat;
lat_model = NonlinearModel(zeros(5,3), equation_error_coeffs_lat);
opt_problem = OutputErrorProblem(fpr_data_lat, lat_model, maneuver_types);

tic
opt_problem = opt_problem.solve();
toc

output_error_coeffs_lat = opt_problem.OptData.params.CoeffsLat;
save("model_identification/output_error/results/output_error_coeffs_lat.mat", "output_error_coeffs_lat");
