clear all; close all; clc;

% Import ss model from AVL
state_space_model;

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lat.mat");

% Lateral system
% State = [v p r phi]
% Input = [delta_a delta_r]

maneuver_types = [
    "roll_211",...
    "yaw_211",...
    ];

% Simulate maneuvers with different models
for maneuver_type = maneuver_types
    for maneuver_i = 1:length(fpr_data_lat.validation.(maneuver_type))
        maneuver = fpr_data_lat.validation.(maneuver_type)(maneuver_i);
        u = maneuver.get_lat_input_sequence();
        t_u = maneuver.Time;
        x_0 = maneuver.get_lat_state_initial();
        
        % Simulate state space model
        delta_u = detrend(u); % state space model assumes perturbation quantities
        [y, t] = lsim(lat_sys, delta_u, t_u, x_0);
        
        % Simulate nonlinear AVL model
        % Simulate equation-error model
        
        % Compare with real flight data
        model_names = ["Real data" "AVL SS"];
        plot_styles = ["-" "--"];
        maneuver.show_plot_lateral_validation(t, y, model_names, plot_styles);
        
    end
end