clear all; close all; clc;

% Import ss model from AVL
avl_state_space_model;

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lon.mat");

% Longitudinal System
% State = [u w q theta]
% Input = [delta_e delta_t]

maneuver_types = ["pitch_211"];

% Simulate maneuvers with different models
for maneuver_type = maneuver_types
    for maneuver_i = 1:length(fpr_data_lon.validation.(maneuver_type))
        maneuver = fpr_data_lon.validation.(maneuver_type)(maneuver_i);
        input_sequence = maneuver.get_lon_input_sequence();
        t_data_seq = maneuver.Time;
        y_0 = maneuver.get_lon_state_initial();
        tspan = [maneuver.Time(1) maneuver.Time(end)];
        lon_state_seq = maneuver.get_lon_state_sequence();
        
        % Simulate state space model
        delta_u = detrend(input_sequence(:,1)); % state space model assumes perturbation quantities
        [y_avl_ss_model, t_avl_ss_model] = lsim(lon_sys, delta_u, t_data_seq, y_0);
        
        % Collect all simulations
        y_all_models = {y_avl_ss_model};
        t_all_models = {t_avl_ss_model};
        
        % Compare with real flight data
        model_names = ["Real data" "State Space"];
        plot_styles = ["-" "-"];
        maneuver.show_plot_longitudinal_validation(t_all_models, y_all_models, model_names, plot_styles);    
    end
end