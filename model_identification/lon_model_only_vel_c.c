clc; clear all; close all;

% Model type
model_type = "full_lat_fixed";
num_states = 10; % [e0 e1 e2 e3 q u w delta_a delta_e delta_r]
num_outputs = 7; % [e0 e1 e2 e3 q u w]
num_inputs = 11; % [nt1 nt2 nt3 nt4 delta_a_sp delta_e_sp delta_r_sp np p r v]

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Select how to compose datasets
maneuver_types = [...
    "roll_211_no_throttle", "pitch_211_no_throttle", "yaw_211_no_throttle",...
    "roll_211", "pitch_211", "yaw_211"
    ];
maneuver_quantities = [0 1 0 0 1 0];

num_models_to_create = 5;

[data, data_full_state] = create_combined_iddata(metadata, maneuver_types, maneuver_quantities, model_type);



%% Load previous model parameters
model_name_to_load = "1";
model_load_path = "fitted_models/full_state_models/" + "model_" + model_name_to_load + "/";
load(model_load_path + "model.mat");

old_parameters = nlgr_model.Parameters;
%% Create new nlgr object
% Load params from other models
model_paths_to_load = [];
[collected_params] = create_collected_params(model_paths_to_load);

% Create model path
model_name = "wip_1";
model_path = "fitted_models/full_state_models/" + "model_" + model_name + "/";

experiments_to_use = 1:sum(maneuver_quantities);
initial_states = create_initial_states_struct(data, num_states, num_outputs, experiments_to_use, model_type);

[nlgr_model] = create_nlgr_object(num_states, num_outputs, num_inputs, collected_params, initial_states, model_type);

%% Print parameters
print_parameters(nlgr_model.Parameters, "all");
print_parameters(nlgr_model.Parameters, "free")

%% Plot response of model
close all;
save_plot = true;
show_plot = true;
sim_responses(...
    experiments_to_use, nlgr_model, data(:,:,:,experiments_to_use), data_full_state(:,:,:,experiments_to_use), model_path, ...,
    save_plot, model_type, show_plot);
print_parameters(nlgr_model.Parameters, "free")

%% Reset static curves
nlgr_model = reset_static_curve_params(nlgr_model, 0);

%% Fix params
params_to_fix = [1:42];
params_to_unfix = [24:27];

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

opt.OutputWeight = diag([0 0 0 0 100 1 1]);
%opt.Regularization.Lambda = 1;
%opt.Regularization.R = [0 100 1 1 1 0 0 1 1 0 0 0 0];
%opt.Regularization.R = [1 1 0.01 0];
%opt.Regularization.R = [];
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
nlgr_model = nlgreyest(data(:,:,:,experiments_to_use), nlgr_model, opt);
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