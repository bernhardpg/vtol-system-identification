clc; clear all; close all;

% Load airframe properties
aircraft_properties;

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Create data for sysid
maneuver_types = ["roll_211_no_throttle", "yaw_211_no_throttle"];
maneuver_quantities = [25, 25];

model_type = "lat";
[data_lat, data_full_state] = create_combined_iddata(metadata, maneuver_types, maneuver_quantities, model_type);

num_states_lat = 9;
num_outputs_lat = 7;
num_inputs_lat = 4; % [delta_a_sp, delta_r_sp, u, w]

%% Load previous model parameters
model_name_to_load = "final3";
model_load_path = "fitted_models/lateral_models/" + "model_" + model_name_to_load + "/";
load(model_load_path + "model.mat");

old_parameters = nlgr_model.Parameters;

%% Create new nlgr object
initial_parameters = create_param_struct("lat");

% Create model path
model_name = "final3";
model_path = "fitted_models/lateral_models/" + "model_" + model_name + "/";

%experiments_to_use = 1:sum(maneuver_quantities);

% TODO: Fix experiment number 19
% TODO: Fix experiment number 42
experiments_to_use = [1:5 26:30];
%experiments_to_use = 26:27;
initial_states = create_initial_states_struct(data_lat, num_states_lat, num_outputs_lat, experiments_to_use, "lat");

[nlgr_model] = create_nlgr_object(num_states_lat, num_outputs_lat, num_inputs_lat, initial_parameters, initial_states, "lat");

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
    experiments_to_use, nlgr_model, data_lat(:,:,:,experiments_to_use), data_full_state(:,:,:,experiments_to_use), model_path, ...,
    save_plot, "lat", show_plot);
print_parameters(nlgr_model.Parameters, "free")
%compare(data_lat(:,:,:,experiments_to_use), nlgr_model)

%%
clf
compare(data_lat, nlgr_model); % simulate the model and compare the response to measured values

%%
figure;
h_gcf = gcf;
Pos = h_gcf.Position;
h_gcf.Position = [Pos(1), Pos(2)-Pos(4)/2, Pos(3), Pos(4)*1.5];
pe(data_lat, nlgr_model);

%% Fix params
params_to_fix = [1:29];
params_to_unfix = [15:29];

nlgr_model = fix_parameters(params_to_fix, nlgr_model, true);
nlgr_model = fix_parameters(params_to_unfix, nlgr_model, false);
print_parameters(nlgr_model.Parameters, "all")
print_parameters(nlgr_model.Parameters, "free")

%% Reset parameters to their initial values
params_to_reset = [12 14];
[nlgr_model] = reset_parameters(nlgr_model, params_to_reset, initial_parameters);
print_parameters(nlgr_model.Parameters, "all");
print_parameters(nlgr_model.Parameters, "free")

%% Specify optimization options
opt = nlgreyestOptions('Display', 'on');
opt.SearchOptions.MaxIterations = 100;

% Prediction error weight
% Only weigh states p, r, v

opt.OutputWeight = diag([0 0 0 0 1 1 1]);
opt.Regularization.Lambda = 100;
opt.Regularization.Nominal = 'model';
% opt.Regularization.R = [
%     1,...% c_Y_beta
%     1000,...% c_Y_p
%     1000,...% c_Y_r
%     1,...% c_Y_delta_a
%     1,...% c_Y_delta_r
%     1,...% c_l_beta
%     1,...% c_l_p
%     1,...% c_l_r
%     1000,...% c_l_delta_a
%     1,...% c_l_delta_r
%     1000,...% c_n_beta
%     1,...% c_n_p
%     1000,...% c_n_r
%     1,...% c_n_delta_a
%     1000,...% c_n_delta_r
% ];

% opt_type = "neutral";
% opt.Regularization.Lambda = 10;
% opt.Regularization.R = create_reg_matrix("yaw");
% % Specify initial guess as nominal parameter values
% opt.Regularization.Nominal = 'model';
% opt.Advanced.ErrorThreshold = 1.6;
% opt.OutputWeight = create_cost_fn(opt_type);


%% Estimate NLGR model
print_parameters(nlgr_model.Parameters, "free");
nlgr_model = nlgreyest(data_lat(:,:,:,experiments_to_use), nlgr_model, opt);
parameters = nlgr_model.Parameters;
print_parameters(nlgr_model.Parameters, "free");

%% Save model
mkdir(model_path)
save(model_path + "model.mat", 'nlgr_model');
disp("Saved " + model_name);

%% Functions

function [rolling_motion_regularization] = create_reg_matrix(opt_type)
    if opt_type == "roll"
        rolling_motion_regularization = [
            0,...% c_Y_p
            100,...% c_Y_r
            0,...% c_Y_delta_a
            100,...% c_Y_delta_r
            0,...% c_l_p
            100,...% c_l_r
            0,...% c_l_delta_a
            100,...% c_l_delta_r
            0,...% c_n_p
            100,...% c_n_r
            0,...% c_n_delta_a
            100,...% c_n_delta_r
        ];
    elseif opt_type == "yaw"
        rolling_motion_regularization = [
            100,...% c_Y_p
            0,...% c_Y_r
            100,...% c_Y_delta_a
            0,...% c_Y_delta_r
            100,...% c_l_p
            0,...% c_l_r
            100,...% c_l_delta_a
            0,...% c_l_delta_r
            100,...% c_n_p
            0,...% c_n_r
            100,...% c_n_delta_a
            0,...% c_n_delta_r
        ];
    end
end

function [output_weights] = create_cost_fn(opt_type)
    if opt_type == "roll"
        output_weights = diag([0 0 0 0 100 10 1]);
    elseif opt_type == "yaw"
        output_weights = diag([0 0 0 0 10 100 1]);
    elseif opt_type == "neutral"
        output_weights = diag([0 0 0 0 10 10 1]);
    end
end

