clc; clear all; close all;

% Load airframe properties
aircraft_properties;

metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);
maneuver_type = "yaw_211_no_throttle";
%maneuver_type = "roll_211_no_throttle";

[full_state, full_input, t, maneuver_start_indices] = read_experiment_data(metadata, maneuver_type);

% Create ID data from each experiment
[data_full_state] = create_iddata(t, full_state, full_input, maneuver_start_indices, "full");
[data_lat] = create_iddata(t, full_state, full_input, maneuver_start_indices, "lat");

num_experiments = length(data_lat.Experiment);

num_states_lat = 9;
num_outputs_lat = 7;
num_inputs_lat = 4; % [delta_a_sp, delta_r_sp, u, w]

%% Load previous model parameters
model_number_to_load = 6;
model_load_path = "nlgr_models/lateral_models/" + "model_" + model_number_to_load + "/";
load(model_load_path + "model.mat");

old_parameters = nlgr_model.Parameters;

%% Create new nlgr object
parameters = create_param_struct("lat");

% Create model path
model_number = 6;
model_path = "nlgr_models/lateral_models/" + "model_" + model_number + "/";

experiments_to_use = 1:2;
initial_states = create_initial_states_struct(data_lat, num_states_lat, num_outputs_lat, experiments_to_use, "lat");

[nlgr_model] = create_nlgr_object(num_states_lat, num_outputs_lat, num_inputs_lat, parameters, initial_states, "lat");

%% Load parameters from old model
[nlgr_model] = load_parameters_into_model(nlgr_model, old_parameters);

%% Print parameters
%print_parameters(nlgr_model.Parameters, "all");
print_parameters(nlgr_model.Parameters, "free")

%% Plot response of model
sim_responses(experiments_to_use, nlgr_model, data_lat(:,:,:,experiments_to_use), data_full_state(:,:,:,experiments_to_use), model_path, true, "lat");
%compare(data_lat(:,:,:,experiments_to_use), nlgr_model)

%% Specify optimization options
opt = nlgreyestOptions('Display', 'on');
opt.SearchOptions.MaxIterations = 100;

% Prediction error weight
% Only weigh states p, r, v
output_weights = diag([0 0 0 0 10 10 1]);
opt.OutputWeight = output_weights;

opt.Regularization.Lambda = 1;

%% Regularization
opt.Regularization.Lambda = 100;
% Specify that the second parameter better known than the first.

rolling_motion_regularization = [
    100,...% c_Y_p
    100,...% c_Y_r
    10,...% c_Y_delta_a
    10,...% c_Y_delta_r
    0.1,...% c_l_p
    100,...% c_l_r
    0.1,...% c_l_delta_a
    100,...% c_l_delta_r
    0.1,...% c_n_p
    100,...% c_n_r
    100,...% c_n_delta_a
    100,...% c_n_delta_r
];

opt.Regularization.R = rolling_motion_regularization;

% Specify initial guess as nominal parameter values. 
opt.Regularization.Nominal = 'model';

%% Fix parameters
rudder_params = [];
nlgr_model = fix_parameters(params_to_fix, nlgr_model, true);


%% Estimate NLGR model
nlgr_model = nlgreyest(data_lat(:,:,:,experiments_to_use), nlgr_model, opt);
parameters = nlgr_model.Parameters;
print_parameters(nlgr_model.Parameters, "all");

%% Save model
mkdir(model_path)
save(model_path + "model.mat", 'nlgr_model');