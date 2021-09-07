clear all; close all; clc;

% Import ss model from AVL
avl_state_space_model;

% Import coeffs from AVL
load("avl_model/avl_results/avl_coeffs_lat.mat");
avl_nonlin_lat_model = NonlinearModel(zeros(5,3), avl_coeffs_lat);

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lat.mat");

% Load equation_error parameters
load("model_identification/equation_error/results/equation_error_coeffs_lat.mat");
eq_error_lat_model = NonlinearModel(zeros(5,3), equation_error_coeffs_lat);

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
        input_sequence = maneuver.get_lat_input_sequence();
        t_data_seq = maneuver.Time;
        y_0 = maneuver.get_lat_state_initial();
        tspan = [maneuver.Time(1) maneuver.Time(end)];
        lon_state_seq = maneuver.get_lon_state_sequence();
        
        % Simulate state space model
        delta_u = detrend(input_sequence); % state space model assumes perturbation quantities
        [y_avl_ss_model, t_avl_ss_model] = lsim(lat_sys, delta_u, t_data_seq, y_0);
        
        % Simulate nonlinear AVL model
        [t_avl_nonlin_model, y_avl_nonlin_model] = ode45(@(t,y) avl_nonlin_lat_model.dynamics(t, y, t_data_seq, delta_u, lon_state_seq), tspan, y_0);
        
        % Simulate equation-error model
        [t_eq_error_model, y_eq_error_model] = ode45(@(t,y) eq_error_lat_model.dynamics(t, y, t_data_seq, input_sequence, lon_state_seq), tspan, y_0);
        
        % Collect all simulations
        y_all_models = {y_avl_ss_model, y_avl_nonlin_model, y_eq_error_model};
        t_all_models = {t_avl_ss_model, t_avl_nonlin_model, t_eq_error_model};
        
        % Compare with real flight data
        model_names = ["Real data" "AVL SS" "AVL Nonlinear Model" "Equation-Error"];
        plot_styles = ["-" "--" "--" "-"];
        maneuver.show_plot_lateral_validation(t_all_models, y_all_models, model_names, plot_styles);    
    end
end