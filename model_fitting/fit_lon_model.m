clc; clear all; close all;

% Based on this guide:
% https://se.mathworks.com/help/ident/ug/industrial-three-degrees-of-freedom-robot-c-mex-file-modeling-of-mimo-system-using-vector-matrix-parameters.html

metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);
maneuver_type = "pitch_211_no_throttle";
[full_state, full_input, t, maneuver_start_indices] = read_experiment_data(metadata, maneuver_type);

% Load airframe properties
aircraft_properties;

% Create ID data from each experiment
[data_full_state] = create_iddata(t, full_state, full_input, maneuver_start_indices, "full");
[data_lon] = create_iddata(t, full_state, full_input, maneuver_start_indices, "lon");

num_states_lon = 6;
num_outputs_lon = 5;
num_inputs_lon = 2;
num_experiments = length(data_lon.Experiment);

%% Load previous model parameters
model_number_to_load = 8;
model_load_path = "nlgr_models/longitudinal_models/" + "model_" + model_number_to_load + "/";
load(model_load_path + "model.mat");

old_parameters = nlgr_model.Parameters;
%% Create new nlgr object
parameters = create_param_struct("lon");

% Create model path
model_number = 9;
model_path = "nlgr_models/" + "model_" + model_number + "/";

experiments_to_use = [1:10];
initial_states = create_initial_states_struct(data_lon, num_states_lon, num_outputs_lon, experiments_to_use, "lon");

[nlgr_model] = create_nlgr_object(num_states_lon, num_outputs_lon, num_inputs_lon, parameters, initial_states, "lon");

%% Load parameters from old model
[nlgr_model] = load_parameters_into_model(nlgr_model, old_parameters);

%% Fix parameters
params_to_fix = [1:24];
nlgr_model = fix_parameters(params_to_fix, nlgr_model, true);

%% Unfix Parameters
params_to_unfix = [12:24];
nlgr_model = fix_parameters(params_to_unfix, nlgr_model, false);

%% Reset static curves
nlgr_model = reset_static_curve_params(nlgr_model, 0);

%% Set optimization options

opt = nlgreyestOptions('Display', 'on');
opt.SearchOptions.MaxIterations = 100;

% Prediction error weight
% Only weigh states q, u, w
state_weights = diag([0 0 1 1 1]);
opt.OutputWeight = state_weights;

% Regularization
opt.Regularization.Lambda = 200;
% Specify that the second parameter better known than the first.
opt.Regularization.R = [
        1,... % g,                  ...
        1,... % half_rho_planform, ...
        1,... % mass_kg,					...
        1,... % mean_chord_length,              ...
        1,... % wingspan,					...
        1,... %nondim_constant_lon
        1,... % lam,				...
        1,... % Jyy, ...
        1,... % servo_time_const,...
        1,... % servo_rate_lim_rad_s,...
        1,... % elevator_trim_rad
        1,... % c_L_0,				...
        1,... % c_L_alpha,      	...
        0.1,... % c_L_q,          	...
        0,... % c_L_delta_e,    	...
        1,... % c_D_p,				...
        1,... % c_D_alpha,          ...
        1,... % c_D_alpha_sq,          ...
        0.1,... % c_D_q,          	...
        0,... % c_D_delta_e,    	...
        0,... % c_m_0,				...
        0,... % c_m_alpha,          ...
        0,... % c_m_q,				...
        0,... % c_m_delta_e,		...
];

opt.Regularization.R = [
        1,... % c_L_0,				...
        1,... % c_L_alpha,      	...
        0,... % c_L_q,          	...
        0,... % c_L_delta_e,    	...
        1,... % c_D_p,				...
        1,... % c_D_alpha,          ...
        1,... % c_D_alpha_sq,          ...
        0,... % c_D_q,          	...
        0,... % c_D_delta_e,    	...
        0,... % c_m_0,				...
        0,... % c_m_alpha,          ...
        0,... % c_m_q,				...
        0,... % c_m_delta_e,		...
];

% Specify initial  guess as Nominal. 
opt.Regularization.Nominal = 'model';

%% Estimate NLGR model
nlgr_model = nlgreyest(data_lon(:,:,:,experiments_to_use), nlgr_model, opt);
parameters = nlgr_model.Parameters;
print_parameters(nlgr_model.Parameters);


%% Evaluate performance of model
print_parameters(nlgr_model.Parameters, "all");
print_parameters(nlgr_model.Parameters, "free")
sim_responses(experiments_to_use, nlgr_model, data_lon(:,:,:,experiments_to_use), data_full_state(:,:,:,experiments_to_use), model_path, true, "lon");


%compare(nlgr_model, data);
compare(data_lon(:,:,:,experiments_to_use), nlgr_model)
%% Save model

mkdir(model_path)
save(model_path + "model.mat", 'nlgr_model');

%% Local functions

function [nlgr_model] = reset_static_curve_params(nlgr_model, fix)
    lift_drag_properties;
    nlgr_model.Parameters(12).Value = c_L_0;
    nlgr_model.Parameters(13).Value = c_L_alpha;
    nlgr_model.Parameters(14).Value = c_L_q;
    
    if fix
        nlgr_model.Parameters(12).Fixed = 1;
        nlgr_model.Parameters(13).Fixed = 1;
        nlgr_model.Parameters(14).Fixed = 1;
        nlgr_model.Parameters(16).Fixed = 1;
        nlgr_model.Parameters(17).Fixed = 1;
        nlgr_model.Parameters(18).Fixed = 1;
        nlgr_model.Parameters(19).Fixed = 1;
    end
    
    nlgr_model.Parameters(16).Value = c_D_p;
    nlgr_model.Parameters(17).Value = c_D_alpha;
    nlgr_model.Parameters(18).Value = c_D_alpha_sq;
    nlgr_model.Parameters(19).Value = c_D_q;

end