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

% Load output-error parameters
load("model_identification/output_error/results/output_error_lat_coeffs.mat");
output_error_lat_model = NonlinearModel(zeros(5,3), output_error_lat_coeffs);

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
            
            [y_output_error, error_calculations] = evaluate_model(maneuver, output_error_lat_model, model_type);
            error_metrics.output_error{maneuver_i} = error_calculations;
%             
%             [y_output_error_all_params, error_calculations] = evaluate_model(maneuver, output_error_lon_model_all_free);
%             error_metrics.y_output_error_all_params{maneuver_i} = error_calculations;
%             
%             [y_output_error_specific_params, error_calculations] = evaluate_model(maneuver, output_error_lon_model_final);
%             error_metrics.y_output_error_specific_params{maneuver_i} = error_calculations;
%             
            % Collect all simulations
            y_all_models = {y_eq_error y_output_error}; %y_output_error_all_params y_output_error_specific_params};

            % Compare with real flight data
            model_names = ["Real data" "Equation-Error" "Output-Error"]; %"Output-Error (all params)" "Output-Error (some params)"];
            plot_styles = ["--" "-" "-"];% ":" "-"];
            if show_maneuver_plots
                maneuver.show_plot_lateral_validation(t_sim, y_all_models, model_names, plot_styles);
            end
        end
    end
end

if show_error_metric_plots
    if test_nonlin_models
        [gof_eq_error, tic_eq_error, an_eq_error] = evaluate_error_metrics(error_metrics.eq_error);
        [gof_output_error, tic_output_error, an_output_error] = evaluate_error_metrics(error_metrics.output_error);
        
        model_names = ["Equation-Error", "Output-Error"];
        create_bar_plot([gof_eq_error; gof_output_error], model_names, "Goodness-of-Fit (GOF)", ["v","p","r","\phi"], ["$v$","$p$","$r$","$\phi$"]);
        create_bar_plot([tic_eq_error; tic_output_error], model_names, "Theils-Inequality-Coefficient (TIC)", ["v","p","r","\phi"], ["$v$","$p$","$r$","$\phi$"]);
        create_bar_plot([an_eq_error; an_output_error], model_names, "Average-Normalized Error Measures", ["ANMAE","ANRMSE"], ["ANMAE","ANRMSE"]);
    end
end


load("model_identification/output_error/results/output_error_lat_cr_bounds.mat");
%load("model_identification/output_error/results/output_error_lon_all_free_cr_bounds.mat");
%load("model_identification/output_error/results/output_error_coeffs_lon_final_cr_bounds.mat");

% model_params = {output_error_coeffs_lon output_error_lon_all_free_coeffs output_error_coeffs_lon_final_coeffs};
% model_variances = {output_error_lon_cr_bounds output_error_lon_all_free_cr_bounds output_error_coeffs_lon_final_cr_bounds};

output_error_cr_bounds_percentage = calc_percentage_cr_bound(output_error_lat_coeffs, output_error_lat_cr_bounds);
%output_error_all_free_cr_bounds_percentage = calc_percentage_cr_bound(output_error_lon_all_free_coeffs, output_error_lon_all_free_cr_bounds);
%output_error_final_cr_bounds_percentage = calc_percentage_cr_bound(output_error_coeffs_lon_final_coeffs, output_error_coeffs_lon_final_cr_bounds);

param_names = ["cY0" "cYb" "cYp" "cYr" "cYda" "cYdr"...
    "cl0" "clb" "clp" "clr" "clda" "cldr"...
    "cn0" "cnb" "cnp" "cnr" "cnda" "cndr"];
param_names_latex = ["$c_{Y 0}$" "$c_{Y \beta}$" "$c_{Y p}$" "$c_{Y r}$" "$c_{Y {\delta_a}}$" "$c_{Y {\delta_r}}$"...
    "$c_{l 0}$" "$c_{l \beta}$" "$c_{l p}$" "$c_{l r}$" "$c_{l {\delta_a}}$" "$c_{l {\delta_r}}$"...
    "$c_{n 0}$" "$c_{n \beta}$" "$c_{n p}$" "$c_{n r}$" "$c_{n {\delta_a}}$" "$c_{n {\delta_r}}$"];

create_bar_plot([output_error_cr_bounds_percentage], ["Output-Error"], "2CR %", param_names, param_names_latex);

cr_bounds_means = [mean(fillmissing(output_error_cr_bounds_percentage,'constant',0))];
create_bar_plot(cr_bounds_means, ["Output-Error"], "2CR %", "Average CR Bound", param_names_latex);


%%
model_params = {avl_coeffs_lat equation_error_coeffs_lat output_error_lat_coeffs};
model_cr_bounds = {zeros(15,1) zeros(15,1) output_error_lat_cr_bounds};

num_models = numel(model_params);
figure
for param_i = 1:15
    for model_i = 1:num_models
        subplot(3,5,param_i);
        param = model_params{model_i}(param_i);
        covar = model_cr_bounds{model_i}(param_i);
        stddev = 2 * sqrt(covar); % Approximation taken from Tischler, see (29) in Dorobantu2013
        cr_percentage = abs(stddev / param) * 100;
        errorbar(model_i,param,stddev,'-s','MarkerSize',10); hold on
        xlim([0 num_models+1])
        set(gca,'xticklabel',{[]})
        title(param_names_latex(param_i))
    end
end
legend(["AVL" "Equation-Error" "Output-Error"],'Location','SouthEast')