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

% % Load output-error parameters
% load("model_identification/output_error/results/output_error_coeffs_lat.mat");
% output_error_lat_model = NonlinearModel(zeros(5,3), output_error_coeffs_lat);

% Lateral system
% State = [v p r phi]
% Input = [delta_a delta_r]

model_type = "lateral-directional";
maneuver_types = [
    "roll_211",...
    "yaw_211",...
    ];

test_avl_models = false;
test_nonlin_models = true;
show_maneuver_plots = true;
show_error_metric_plots = true;

for maneuver_type = maneuver_types
    num_maneuvers = length(fpr_data_lat.validation.(maneuver_type));
    error_metrics = {};
    
    for maneuver_i = 1:length(fpr_data_lat.validation.(maneuver_type))
        maneuver = fpr_data_lat.validation.(maneuver_type)(maneuver_i);
        t_sim = maneuver.Time;
        
        if test_avl_models
            % Simulate AVL state space model
            % First create perturbation state quantities, as this is what
            % the AVL model uses
            [y_avl_ss, error_calculations] = evaluate_ss_model(maneuver, lat_sys, model_type);
            error_metrics.avl_ss{maneuver_i} = error_calculations;

            % Simulate nonlinear AVL model
            [y_avl_nonlin, error_calculations] = evaluate_model(maneuver, avl_nonlin_lat_model, model_type);
            error_metrics.avl_nonlin{maneuver_i} = error_calculations;
            
            % Collect all simulations
            y_all_models = {y_avl_ss, y_avl_nonlin};

            % Compare with real flight data
            model_names = ["Real data" "State Space (AVL)" "Nonlinear (AVL)"];
            plot_styles = ["-" "-" "-"];
            if show_maneuver_plots
                maneuver.show_plot_lateral_validation(t_sim, y_all_models, model_names, plot_styles);
            end
        end
        if test_nonlin_models
            [y_eq_error, error_calculations] = evaluate_model(maneuver, eq_error_lat_model, model_type);
            error_metrics.eq_error{maneuver_i} = error_calculations;
            
%             [y_output_error, error_calculations] = evaluate_model(maneuver, output_error_lon_model);
%             error_metrics.output_error{maneuver_i} = error_calculations;
%             
%             [y_output_error_all_params, error_calculations] = evaluate_model(maneuver, output_error_lon_model_all_free);
%             error_metrics.y_output_error_all_params{maneuver_i} = error_calculations;
%             
%             [y_output_error_specific_params, error_calculations] = evaluate_model(maneuver, output_error_lon_model_final);
%             error_metrics.y_output_error_specific_params{maneuver_i} = error_calculations;
%             
            % Collect all simulations
            y_all_models = {y_eq_error};%; y_output_error y_output_error_all_params y_output_error_specific_params};

            % Compare with real flight data
            model_names = ["Real data" "Equation-Error"];% "Output-Error" "Output-Error (all params)" "Output-Error (some params)"];
            plot_styles = ["--" "-"]; %":" ":" "-"];
            if show_maneuver_plots
                maneuver.show_plot_lateral_validation(t_sim, y_all_models, model_names, plot_styles);
            end
        end
    end
end

%% OLD
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
        delta_u = detrend(input_sequence,0); % state space model assumes perturbation quantities
        [y_avl_ss_model, t_avl_ss_model] = lsim(lat_sys, delta_u, t_data_seq, y_0);
        
        % Simulate nonlinear AVL model
        [t_avl_nonlin_model, y_avl_nonlin_model] = ode45(@(t,y) avl_nonlin_lat_model.dynamics_lat_model_c(t, y, t_data_seq, delta_u, lon_state_seq), tspan, y_0);
        
        % Simulate equation-error model
        [t_eq_error_model, y_eq_error_model] = ode45(@(t,y) eq_error_lat_model.dynamics_lat_model_c(t, y, t_data_seq, delta_u, lon_state_seq), tspan, y_0);
        
        % Simulate output-error model
        [t_output_error_model, y_output_error_model] = ode45(@(t,y) output_error_lat_model.dynamics_lat_model_c(t, y, t_data_seq, delta_u, lon_state_seq), tspan, y_0);
        
        % Collect all simulations
        y_all_models = {y_avl_ss_model, y_avl_nonlin_model};
        t_all_models = {t_avl_ss_model, t_avl_nonlin_model};
        
        % Compare with real flight data
        model_names = ["Real data" "AVL SS" "AVL Nonlinear Model"];
        plot_styles = ["-" "--" "--"];

%         % Collect all simulations
%         y_all_models = {y_eq_error_model, y_output_error_model};
%         t_all_models = {t_eq_error_model, t_output_error_model};
%         
%         % Compare with real flight data
%         model_names = ["Real data" "Equation-Error" "Output-error"];
%         plot_styles = ["-" "-" "-"];
        maneuver.show_plot_lateral_validation(t_all_models, y_all_models, model_names, plot_styles);    
    end
end