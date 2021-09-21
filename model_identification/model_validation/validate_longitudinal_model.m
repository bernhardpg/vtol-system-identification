clear all; close all; clc;

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lon.mat");

% Import ss model from AVL
avl_state_space_model;

% Import coeffs from AVL
load("avl_model/avl_results/avl_coeffs_lon.mat");
avl_nonlin_lon_model = NonlinearModel(avl_coeffs_lon, zeros(6,3));

% Load equation_error coefficients
load("model_identification/equation_error/results/equation_error_coeffs_lon.mat");
eq_error_lon_model = NonlinearModel(equation_error_coeffs_lon, zeros(6,3));

% Load output-error coefficients
load("model_identification/output_error/results/output_error_coeffs_lon.mat");
output_error_lon_model = NonlinearModel(output_error_coeffs_lon, zeros(6,3));

load("model_identification/output_error/results/output_error_coeffs_lon_all_free.mat");
output_error_lon_model_all_params = NonlinearModel(output_error_coeffs_lon_all_free, zeros(6,3));

% Longitudinal System
% State = [u w q theta]
% Input = [delta_e delta_t]

maneuver_types = "pitch_211";
test_avl_models = false;
test_nonlin_models = true;

% Simulate maneuvers with different models

for maneuver_type = maneuver_types
    num_maneuvers = length(fpr_data_lon.validation.(maneuver_type));
    error_metrics = {};
    
    for maneuver_i = 1:length(fpr_data_lon.validation.(maneuver_type))
        maneuver = fpr_data_lon.validation.(maneuver_type)(maneuver_i);
        t_sim = maneuver.Time;
        
        if test_avl_models
            % Simulate AVL state space model
            % First create perturbation state quantities, as this is what
            % the AVL model uses
            [y_avl_ss, error_calculations] = evaluate_ss_model(maneuver, lon_sys);
            error_metrics.avl_ss{maneuver_i} = error_calculations;

            % Simulate nonlinear AVL model
            [y_avl_nonlin, error_calculations] = evaluate_model(maneuver, avl_nonlin_lon_model);
            error_metrics.avl_nonlin{maneuver_i} = error_calculations;
            
            % Collect all simulations
            y_all_models = {y_avl_ss, y_avl_nonlin};

            % Compare with real flight data
            model_names = ["Real data" "State Space (AVL)" "Nonlinear (AVL)"];
            plot_styles = ["-" "-" "-"];
            maneuver.show_plot_longitudinal_validation(t_sim, y_all_models, model_names, plot_styles);
        end
        if test_nonlin_models
            [y_eq_error, error_calculations] = evaluate_model(maneuver, eq_error_lon_model);
            error_metrics.eq_error{maneuver_i} = error_calculations;
            
            [y_output_error, error_calculations] = evaluate_model(maneuver, output_error_lon_model);
            error_metrics.output_error{maneuver_i} = error_calculations;
            
            [y_output_error_all_params, error_calculations] = evaluate_model(maneuver, output_error_lon_model_all_params);
            error_metrics.y_output_error_all_params{maneuver_i} = error_calculations;
            
            % Collect all simulations
            y_all_models = {y_eq_error y_output_error y_output_error_all_params};

            % Compare with real flight data
            model_names = ["Real data" "Equation-Error" "Output-Error" "Output-Error (all params)"];
            plot_styles = ["-" "-" "-" "-"];
            maneuver.show_plot_longitudinal_validation(t_sim, y_all_models, model_names, plot_styles);
        end
    end
end

if test_avl_models
    [gof_avl_ss, tic_avl_ss, an_avl_ss] = evaluate_error_metrics(error_metrics.avl_ss);
    [gof_avl_nonlin, tic_avl_nonlin, an_avl_nonlin] = evaluate_error_metrics(error_metrics.avl_nonlin);

    model_names = ["State-Space (AVL)", "Nonlinear (AVL)"];
    create_bar_plot([gof_avl_ss; gof_avl_nonlin], model_names, "Goodness-of-Fit (GOF)", {'u','w','q','\theta'});
    create_bar_plot([tic_avl_ss; tic_avl_nonlin], model_names, "Theils-Inequality-Coefficient (TIC)", {'u','w','q','\theta'});
    create_bar_plot([an_avl_ss; an_avl_nonlin], model_names, "Average-Normalized Error Measures", {'ANMAE','ANRMSE'});
end

if test_nonlin_models
    [gof_eq_error, tic_eq_error, an_eq_error] = evaluate_error_metrics(error_metrics.eq_error);
    [gof_output_error, tic_output_error, an_output_error] = evaluate_error_metrics(error_metrics.output_error);
    [gof_output_error_all_free, tic_output_error_all_free, an_output_error_all_free] = evaluate_error_metrics(error_metrics.y_output_error_all_params);

    model_names = ["Equation-Error", "Output-Error", "Output-Error (all free)"];
    create_bar_plot([gof_eq_error; gof_output_error; gof_output_error_all_free], model_names, "Goodness-of-Fit (GOF)", {'u','w','q','\theta'});
    create_bar_plot([tic_eq_error; tic_output_error; tic_output_error_all_free], model_names, "Theils-Inequality-Coefficient (TIC)", {'u','w','q','\theta'});
    create_bar_plot([an_eq_error; an_output_error; an_output_error_all_free], model_names, "Average-Normalized Error Measures", {'ANMAE','ANRMSE'});
end