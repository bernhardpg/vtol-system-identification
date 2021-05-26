clc; clear all; close all;

% Load airframe properties
aircraft_properties;

metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);
maneuver_type = "roll_211_no_throttle";

[full_state, full_input, t, maneuver_start_indices] = read_experiment_data(metadata, maneuver_type);

% Create ID data from each experiment
[data_full_state] = create_iddata(t, full_state, full_input, maneuver_start_indices, "full");
[data_lat] = create_iddata(t, full_state, full_input, maneuver_start_indices, "lat");

state_size = 8;
num_experiments = length(data_lat.Experiment);

%% Create new nlgr object
parameters = create_lon_parameter_struct();

% Create model path
model_number = 9;
model_path = "nlgr_models/" + "model_" + model_number + "/";

experiments_to_use = [1:10];
initial_states = create_initial_states_lon_struct(data_lon, state_size, experiments_to_use);

[nlgr_model] = create_nlgr_object(parameters, initial_states, "lon");