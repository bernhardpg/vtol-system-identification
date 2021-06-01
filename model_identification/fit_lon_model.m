clc; clear all; close all;

% Load airframe properties
aircraft_properties;

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Create data for sysid
maneuver_types = ["pitch_211_no_throttle"];
maneuver_quantities = [12];

model_type = "lon";
[data_lon, data_full_state] = create_combined_iddata(metadata, maneuver_types, maneuver_quantities, model_type);

num_states_lon = 6;
num_outputs_lon = 5;
num_inputs_lon = 2;
num_experiments = length(data_lon.Experiment);

%% Load previous model parameters
model_name_to_load = "final3";
model_load_path = "fitted_models/longitudinal_models/" + "model_" + model_name_to_load + "/";
load(model_load_path + "model.mat");

old_parameters = nlgr_model.Parameters;
%% Create new nlgr object
initial_parameters = create_param_struct("lon");

% Create model path
model_name = "test_low_drag_3";
model_path = "fitted_models/longitudinal_models/" + "model_" + model_name + "/";

experiments_to_use = [1:12];
initial_states = create_initial_states_struct(data_lon, num_states_lon, num_outputs_lon, experiments_to_use, "lon");

[nlgr_model] = create_nlgr_object(num_states_lon, num_outputs_lon, num_inputs_lon, initial_parameters, initial_states, "lon");

%% Load parameters from old model
[nlgr_model] = load_parameters_into_model(nlgr_model, old_parameters);

%% Print parameters
print_parameters(nlgr_model.Parameters, "all");
print_parameters(nlgr_model.Parameters, "free")

%% Plot response of model
close all;
save_plot = true;
show_plot = true;
sim_responses(...
    experiments_to_use, nlgr_model, data_lon(:,:,:,experiments_to_use), data_full_state(:,:,:,experiments_to_use), model_path, ...,
    save_plot, "lon", show_plot);
print_parameters(nlgr_model.Parameters, "free")
%compare(data_lon(:,:,:,experiments_to_use), nlgr_model)

%% Reset static curves
nlgr_model = reset_static_curve_params(nlgr_model, 0);

%% Fix params
params_to_fix = [1:27];
params_to_unfix = [15:27];

nlgr_model = fix_parameters(params_to_fix, nlgr_model, true);
nlgr_model = fix_parameters(params_to_unfix, nlgr_model, false);
print_parameters(nlgr_model.Parameters, "all")
print_parameters(nlgr_model.Parameters, "free")

%% Reset parameters to their initial values
params_to_reset = [15 16];
[nlgr_model] = reset_parameters(nlgr_model, params_to_reset, initial_parameters);
print_parameters(nlgr_model.Parameters, "all");
print_parameters(nlgr_model.Parameters, "free")

%% Specify optimization options
opt = nlgreyestOptions('Display', 'on');
opt.SearchOptions.MaxIterations = 100;

% Prediction error weight
% Only weigh states p, r, v

%opt.OutputWeight = diag([1 1 1 0.01 0.01]);
opt.OutputWeight = diag([1 1 10 1 1]);
opt.Regularization.Lambda = 100;
opt.Regularization.Nominal = 'model';
%opt.Regularization.R = [1 1 1 1 1];
%opt.Regularization.R = [1 1 0.01 0];
%opt.Regularization.R = [50 50 0.1 0.1 0.1 0.1 0.1 0.1 0.1];
%opt.Regularization.R = [50 2000 1 1 1 1 1 1 1];
%opt.Regularization.Nominal = 'model';
% opt.Regularization.R = [
%         100,... % c_L_0,				...
%         100,... % c_L_alpha,      	...
%         0,... % c_L_q,          	...
%         0,... % c_L_delta_e,    	...
%         0,... % c_D_p,				...
%         0,... % c_D_alpha,          ...
%         0,... % c_D_alpha_sq,          ...
%         0,... % c_D_q,          	...
%         0,... % c_D_delta_e,    	...
%         0,... % c_m_0,				...
%         0,... % c_m_alpha,          ...
%         0,... % c_m_q,				...
%         0,... % c_m_delta_e,		...
%];

% opt.Advanced.ErrorThreshold = 1.6;


%% Estimate NLGR model
print_parameters(nlgr_model.Parameters, "free");
nlgr_model = nlgreyest(data_lon(:,:,:,experiments_to_use), nlgr_model, opt);
parameters = nlgr_model.Parameters;
print_parameters(nlgr_model.Parameters, "free");

%% Save model
mkdir(model_path)
save(model_path + "model.mat", 'nlgr_model');
disp("Saved " + model_name);
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