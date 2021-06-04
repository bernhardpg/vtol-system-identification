clc; clear all; close all;

% Model type
model_type = "full_lat_fixed";
num_states = 10; % [e0 e1 e2 e3 q u w delta_a delta_e delta_r]
num_outputs = 7; % [e0 e1 e2 e3 q u w]
num_inputs = 11; % [nt1 nt2 nt3 nt4 delta_a_sp delta_e_sp delta_r_sp np p r v]

% Create model path
model_name = "wip_1";
model_path = "fitted_models/full_state_models/" + "model_" + model_name + "/";

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Select how to compose datasets
maneuver_types = [...
    "roll_211_no_throttle", "pitch_211_no_throttle", "yaw_211_no_throttle",...
    "roll_211", "pitch_211", "yaw_211"
    ];

num_models_to_create = 1;
trained_params = {};

%val_maneuver_quantities = [2 2 2 2 2 2];
%[data_val, data_full_state_val] = create_combined_iddata(metadata, maneuver_types, val_maneuver_quantities, model_type);
% Train different models on different data
maneuver_quantities = [0 0 0 0 29 0];
[data, data_full_state] = create_combined_iddata(metadata, maneuver_types, maneuver_quantities, model_type);
maneuver_quantities = [0 0 0 0 29 0];
[data_val, data_full_state_val] = create_combined_iddata(metadata, maneuver_types, maneuver_quantities, model_type);

for i = 1:num_models_to_create
    disp(" ")
    disp("====================")
    disp("====== Model:" + i)

    model_params = fit_lon_model(data, num_states, num_outputs, num_inputs, model_type);
%     fit = evaluate_performance(model_params, data_val, num_states, num_inputs, num_outputs, model_type);
%     disp("Model " + i + " fit:");
%     fprintf(['  ' repmat('%3.1f%%, ',1,size(fit,2)) '\n'], fit);
%     trained_params(i) = {model_params};

    % Print and save model
    initial_states = create_initial_states_struct(data_val, num_states, num_outputs, model_type);
    nlgr_model = create_nlgr_object(num_states, num_outputs, num_inputs, model_params, initial_states, model_type);
    print_parameters(nlgr_model.Parameters, "free")

    model_path = "fitted_models/multiple_models/" + "model_" + i + "/";
    save_plot = true;
    show_plot = false;
    sim_responses(nlgr_model, data_val, data_full_state_val, model_path, save_plot, model_type, show_plot);

    save("results/trained_models", 'trained_params')
end



%%
% Create plots of validation data
for i = 1:length(trained_params)


    params = trained_params{i};
    initial_states = create_initial_states_struct(data_val, num_states, num_outputs, model_type);
    nlgr_model = create_nlgr_object(num_states, num_outputs, num_inputs, params, initial_states, model_type);
    %print_parameters(nlgr_model.Parameters, "all");
    print_parameters(nlgr_model.Parameters, "free")
    

    model_path = "fitted_models/multiple_models/" + "model_" + i + "/";
    save_plot = true;
    show_plot = false;
    sim_responses(nlgr_model, data_val, data_full_state_val, model_path, save_plot, model_type, show_plot);
end

disp("test");
save("results/trained_models", 'trained_params')


%% Print parameters
print_parameters(nlgr_model.Parameters, "all");
print_parameters(nlgr_model.Parameters, "free")

%% Plot response of model
close all;
save_plot = true;
show_plot = true;
sim_responses(...
    nlgr_model, data, data_full_state, model_path, ...,
    save_plot, model_type, show_plot);
print_parameters(nlgr_model.Parameters, "free")

%% Save model
mkdir(model_path)
save(model_path + "model.mat", 'nlgr_model');
disp("Saved " + model_name);
%% Local functions
function [new_params] = fit_lon_model(data, num_states, num_outputs, num_inputs, model_type)
    % Create params from initial values, but augment randomly
    params = create_collected_params([]);
    augment_percentage = 100;
    lon_params = 22:34;
    params = rand_augment_params(lon_params, params, augment_percentage);
    
    % Create model with params and initial states for the experiments
    initial_states = create_initial_states_struct(data, num_states, num_outputs, model_type);
    nlgr_model = create_nlgr_object(num_states, num_outputs, num_inputs, params, initial_states, model_type);
    disp("Initial params:")
    print_parameters(nlgr_model.Parameters, "free")
    
    % Fix lon params
    params_to_fix = 1:length(params);
    params_to_unfix = lon_params;
    nlgr_model = fix_parameters(params_to_fix, nlgr_model, true);
    nlgr_model = fix_parameters(params_to_unfix, nlgr_model, false);

    % Optimization options
    opt = nlgreyestOptions('Display', 'on');
    opt.SearchOptions.MaxIterations = 40;
    opt.OutputWeight = diag([0 0 0 0 10 1 1]);

    % Estimate model
    %nlgr_model = nlgreyest(data, nlgr_model, opt);
    new_params = nlgr_model.Parameters();
end