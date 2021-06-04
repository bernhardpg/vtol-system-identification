clc; clear all; close all;

% Model type
model_type = "full_lat_fixed";
num_states = 10; % [e0 e1 e2 e3 q u w delta_a delta_e delta_r]
num_outputs = 7; % [e0 e1 e2 e3 q u w]
num_inputs = 11; % [nt1 nt2 nt3 nt4 delta_a_sp delta_e_sp delta_r_sp np p r v]

% Create model path
models_path = "fitted_models/" + string(today('datetime')) + "/" + model_type + "/";

% Find run number of the day
run_number = 1;
curr_run_path = models_path + "run_" + run_number;
while isfolder(curr_run_path)
   run_number = run_number + 1;
   curr_run_path = models_path + "run_" + run_number;
end
mkdir(curr_run_path)

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Select how to compose datasets
maneuver_types = [...
    "roll_211", "roll_211_no_throttle",...
    "pitch_211", "pitch_211_no_throttle",...
    "yaw_211", "yaw_211_no_throttle",...
    ];

num_models_to_create = 5;
trained_params = {};

% Create empty params table
params = create_collected_params([]);
[params_table] = create_param_table(params);

fit_table = table('RowNames', {'e0','e1','e2','e4','p','u','w',});

% Validation data
val_maneuver_quantities = [2 2 2 2 2 2];
[data_val, data_full_state_val] = create_combined_iddata(metadata, maneuver_types, val_maneuver_quantities, model_type);

% Training data
maneuver_quantities = [0 0 4 4 0 0];

for model_i = 1:num_models_to_create
    % Train different models on different data
    [data, ~] = create_combined_iddata(metadata, maneuver_types, maneuver_quantities, model_type);
    
    model_name = "Model_" + model_i;
    disp(" ")
    disp("========================")
    disp("  Training " + model_name)
    % Create model with slightly randomized initial parameters
    model_params = fit_lon_model(data, num_states, num_outputs, num_inputs, model_type);
    
    % Store params
    [params_table] = add_params_to_table(model_name, model_params, params_table);
    disp("  Models:");
    disp(params_table);
    writetable(params_table, curr_run_path + "/params.dat", 'WriteRowNames', true);
    
    % Evaluate the fit of that model on the validation data
    fit_table.(model_name) = evaluate_performance(model_params, data_val, num_states, num_inputs, num_outputs, model_type)';
    disp("  Fits:");
    disp(fit_table)
    writetable(fit_table, curr_run_path + "/fits.dat", 'WriteRowNames', true);
end



%%
% Create plots of validation data
for model_i = 1:length(trained_params)
    params = trained_params{model_i};
    initial_states = create_initial_states_struct(data_val, num_states, num_outputs, model_type);
    nlgr_model = create_nlgr_object(num_states, num_outputs, num_inputs, params, initial_states, model_type);
    %print_parameters(nlgr_model.Parameters, "all");
    print_parameters(nlgr_model.Parameters, "free")
    

    models_path = "fitted_models/multiple_models/" + "model_" + model_i + "/";
    save_plot = true;
    show_plot = false;
    sim_responses(nlgr_model, data_val, data_full_state_val, models_path, save_plot, model_type, show_plot);
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
    nlgr_model, data, data_full_state, models_path, ...,
    save_plot, model_type, show_plot);
print_parameters(nlgr_model.Parameters, "free")

%% Save model
mkdir(models_path)
save(models_path + "model.mat", 'nlgr_model');
disp("Saved " + model_name);
%% Local functions

function [params] = load_params_from_table(params, params_table, model_i)
    [N_params, ~] = size(params_table);
    param_names = params_table.Properties.RowNames;
    for i = 1:N_params
        param_name = param_names{i};
        param_value = params_table{param_name,model_i};
        params = load_param_into_params(param_name, param_value, params);
    end
end

function [params] = load_param_into_params(param_name, param_value, params)
    for i = 1:length(params)
        if strcmp(params(i).Name, param_name)
            params(i).Value = param_value;
        end
    end
end

function [params_table] = create_param_table(params)
    temp = struct2cell(params);
    non_const_params_start = 22;
    Parameters = temp(1,non_const_params_start:end)';
    params_table = table('RowNames',Parameters);
end

function [params_table] = add_params_to_table(col_name, params, params_table)
    temp = struct2cell(params);
    non_const_params_start = 22;
    Values = cell2mat(temp(3,non_const_params_start:end)');
    
    params_table.(col_name) = Values;
end

function [new_params] = fit_lon_model(data, num_states, num_outputs, num_inputs, model_type)
    % Create params from initial values, but augment randomly
    params = create_collected_params([]);
    augment_percentage = 100;
    lon_params = 22:34;
    params = rand_augment_params(lon_params, params, augment_percentage);
    
    % Create model with params and initial states for the experiments
    initial_states = create_initial_states_struct(data, num_states, num_outputs, model_type);
    nlgr_model = create_nlgr_object(num_states, num_outputs, num_inputs, params, initial_states, model_type);
    
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
    nlgr_model = nlgreyest(data, nlgr_model, opt);
    new_params = nlgr_model.Parameters();
end