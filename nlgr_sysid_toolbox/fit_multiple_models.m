clc; clear all; close all;

% Model type
model_type = "lon";
num_states = 10; % [e0 e1 e2 e3 q u w delta_a delta_e delta_r]
num_outputs = 7; % [e0 e1 e2 e3 q u w]
num_inputs = 11; % [nt1 nt2 nt3 nt4 delta_a_sp delta_e_sp delta_r_sp np p r v]

% Create model path
models_path = "runs/" + string(today('datetime')) + "/" + model_type + "/";

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Select how to compose datasets
maneuver_types = [...
    "roll_211", "roll_211_no_throttle",...
    "pitch_211", "pitch_211_no_throttle",...
    "yaw_211", "yaw_211_no_throttle",...
    "freehand", "cruise",...
    ];

%%
% Find run number of the day
run_number = 1;
curr_run_path = models_path + "run_" + run_number;
while isfolder(curr_run_path)
   run_number = run_number + 1;
   curr_run_path = models_path + "run_" + run_number;
end
disp("Saving as run_" + run_number);
mkdir(curr_run_path)

% Run settings
num_models_to_create = 5;
num_maneuvers_in_each_set = 10;
num_training_rounds = 5;
augment_params_percentage = 30;

% Create initial params
initial_params = create_collected_params([]);

% Create tables for run data
first_param_to_save = 23;
last_param_to_save = 50;
params_table = create_param_table(initial_params, first_param_to_save, last_param_to_save);
std_table = create_param_table(initial_params, 23, 36);
fit_table = table('RowNames', {'e0','e1','e2','e4','q','u','w',});

% Trim values
aileron_trim = initial_params(17).Value;
elevator_trim = initial_params(18).Value;
rudder_trim = initial_params(19).Value;
input_trims = [aileron_trim elevator_trim rudder_trim];

% Validation data 
val_maneuver_quantities = [2 2 2 2 2 2 2 2];
val_maneuver_quantities_only_pitch = [0 0 5 5 0 0 0 0];
[data_val, data_full_state_val] = create_combined_iddata(metadata, maneuver_types, val_maneuver_quantities, model_type);
[data_val_only_pitch, data_full_state_val_only_pitch] = create_combined_iddata(metadata, maneuver_types, val_maneuver_quantities, model_type);

for model_i = 1:num_models_to_create
    model_name = "Model_" + model_i;
    disp(" ")
    disp("=======================")
    disp("Training " + model_name)
    
    % Create model with slightly randomized initial parameters
    [model_params] = create_augmented_params(augment_params_percentage);
    
    % Train model X times on new training data
    for i = 1:num_training_rounds
        disp(" - Training round: " + i)
        
        % Randomly pick what kind of maneuvers to use
        maneuver_quantities = rand_gen_maneuver_quantities(num_maneuvers_in_each_set);
        disp(" -- Training with: ");
        disp(maneuver_quantities);
        [data, data_full_state] = create_combined_iddata(metadata, maneuver_types, maneuver_quantities, model_type);
        
        % Create model with params and initial states for the experiments
        initial_states = create_initial_states_struct(data, input_trims, num_states, num_outputs, model_type);
        nlgr_model = create_nlgr_object(num_states, num_outputs, num_inputs, model_params, initial_states, model_type);

        % Unfix lon params
        nlgr_model = fix_parameters(1:51, nlgr_model, true);
        lon_params = 23:36;
        nlgr_model = fix_parameters(lon_params, nlgr_model, false);
        
        % Estimate params from data
        [model_params, pvec_std, noise_covar] = estimate_model(nlgr_model, data);
        disp(" -- Mean std of parameters: " + mean(pvec_std));
    end
    
    % Store params
    [params_table] = add_params_to_table(model_name, model_params, params_table, first_param_to_save, last_param_to_save);
    disp("  Models:");
    disp(params_table);
    writetable(params_table, curr_run_path + "/params.dat", 'WriteRowNames', true);
    
    % Save plots of validation data
    test_model_params(model_params, input_trims, data_val, data_full_state_val, num_states, num_outputs, num_inputs, model_type, curr_run_path + "/" + model_name + "/", true, false)
    
    % Evaluate the fit of that model on the validation data
    fit_table.(model_name) = evaluate_performance(model_params, input_trims, data_val_only_pitch, num_states, num_inputs, num_outputs, model_type)';
    disp("  Fits on new pitch maneuvers:");
    disp(fit_table)
    writetable(fit_table, curr_run_path + "/fits.dat", 'WriteRowNames', true);
    
    % Store standard deviations of model
    std_table.(model_name) = pvec_std;
    disp("  Standard deviations:");
    disp(std_table);
    writetable(std_table, curr_run_path + "/std.dat", 'WriteRowNames', true);
end








%%
test_model_params(model_params, input_trims, data_val, data_full_state_val, num_states, num_outputs, num_inputs, model_type, curr_run_path + "/" + model_name + "/", true, false)

%%
run_to_read = 1;
model_i = 2;

    
% Trim values
aileron_trim = model_params(17).Value;
elevator_trim = model_params(18).Value;
rudder_trim = model_params(19).Value;
input_trims = [aileron_trim elevator_trim rudder_trim];


path = models_path + "run_" + run_to_read + "/";
params_table = readtable(path + "params.dat", "ReadRowNames", true);
initial_params = create_collected_params([]);
model_params = load_params_from_table(initial_params, params_table, model_i);
%%

% Randomly pick what kind of maneuvers to use
maneuver_quantities = rand_gen_maneuver_quantities(num_maneuvers_in_each_set);
disp(" -- Training with: ");
disp(maneuver_quantities);

[data, data_full_state] = create_combined_iddata(metadata, maneuver_types, maneuver_quantities, model_type);

% Create model with params and initial states for the experiments
initial_states = create_initial_states_struct(data, input_trims, num_states, num_outputs, model_type);
nlgr_model = create_nlgr_object(num_states, num_outputs, num_inputs, model_params, initial_states, model_type);

% Unfix lon params
nlgr_model = fix_parameters(1:51, nlgr_model, true);
lon_params = 23:36;
nlgr_model = fix_parameters(lon_params, nlgr_model, false);

inertia_params = [8 13 14];
nlgr_model = fix_parameters(inertia_params, nlgr_model, false);

% Estimate params from data
[model_params, pvec_std, noise_covar] = estimate_model(nlgr_model, data);
disp(" -- Mean std of parameters: " + mean(pvec_std));

test_model_params(model_params, input_trims, data_val, data_full_state_val, num_states, num_outputs, num_inputs, model_type, "", false, true)


%% Load an old model
val_maneuver_quantities = [1 0 1 1 0 0 0 0];
[data_val, data_full_state_val] = create_combined_iddata(metadata, maneuver_types, val_maneuver_quantities, model_type);

%%
run_to_read = 10;
model_i = 1;

    
% Trim values
aileron_trim = model_params(17).Value;
elevator_trim = model_params(18).Value;
rudder_trim = model_params(19).Value;
input_trims = [aileron_trim elevator_trim rudder_trim];


path = models_path + "run_" + run_to_read + "/";
params_table = readtable(path + "params.dat", "ReadRowNames", true);
initial_params = create_collected_params([]);
model_params = load_params_from_table(initial_params, params_table, model_i);

%%
test_model_params(model_params, input_trims, data_val, data_full_state_val, num_states, num_outputs, num_inputs, model_type, "", false, true)


%% Functions
function [maneuver_quantities] = rand_gen_maneuver_quantities(num_maneuvers)
    % Only use pitch maneuvers to fit pitch coefficients
    prob = [1   2   3    4    5   6   7   8;
            0 0 0.5 0.5 0 0 0 0];
%     prob = [1   2   3    4    5   6   7   8;
%             0.4/6 0.4/6 0.3 0.3 0.4/6 0.4/6 0.4/6 0.4/6];
%     prob = [1   2   3    4    5   6   7   8;
%             1/8 1/8 1/8 1/8 1/8 1/8 1/8 1/8];
%     
    maneuver_quantities = zeros(1,8);
    for i = 1:num_maneuvers
        % Randomly load N maneuvers into dataset
        maneuver_type_to_use = randsrc(1,1,prob);
        maneuver_quantities(maneuver_type_to_use) = maneuver_quantities(maneuver_type_to_use) + 1;
    end
end

function [] = test_model_params(params, input_trims, data_val, data_full_state_val, num_states, num_outputs, num_inputs, model_type, path, save_plot, show_plot)
    % Create initial states and model with validation data
    initial_states = create_initial_states_struct(data_val, input_trims, num_states, num_outputs, model_type);
    nlgr_model = create_nlgr_object(num_states, num_outputs, num_inputs, params, initial_states, model_type);
    sim_responses(nlgr_model, data_val, data_full_state_val, path, save_plot, model_type, show_plot, input_trims);
end

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

function [params_table] = create_param_table(params, start_param, end_param)
    temp = struct2cell(params);
    Parameters = temp(1,start_param:end_param)';
    params_table = table('RowNames',Parameters);
end

function [params_table] = add_params_to_table(col_name, params, params_table, start_param, end_param)
    temp = struct2cell(params);
    Values = cell2mat(temp(3,start_param:end_param)');
    params_table.(col_name) = Values;
end

function [augmented_params] = create_augmented_params(augment_percentage)
    % Create clean params
    initial_params = create_collected_params([]);
    
    % Do not augment 24, 25, 28, 29, 30, 36 as we trust these more, and so these are confined to be
    % in a relatively small interval
    params_to_augment = [23 26 27 31:34 36];
    
    augmented_params = rand_augment_params(params_to_augment, initial_params, augment_percentage); 
end

function [new_params, pvec_std, noise_covar] = estimate_model(nlgr_model, data)
    % Optimization options
    %opt = nlgreyestOptions();
    opt = nlgreyestOptions('Display','On', 'EstimateCovariance',true);
    opt.SearchOptions.MaxIterations = 20;
    %opt.SearchOptions.FunctionTolerance = 1e-4; % Reduce by an order of 10
    %opt.SearchOptions.StepTolerance = 1e-5; % Reduce by an order of 10
    opt.OutputWeight = diag([1 1 1 1 1 1 1]);
    opt.Regularization.Lambda = 0.1; % Prevent values from becoming too large

    % Estimate model
    nlgr_model = nlgreyest(data, nlgr_model, opt);
    [~, pvec_std] = getpvec(nlgr_model, "Free");
    noise_covar = diag(nlgr_model.NoiseVariance);
    new_params = nlgr_model.Parameters();
end