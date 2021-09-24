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

% Load output-error parameters
load("model_identification/output_error/results/output_error_lat_coeffs_all_free.mat");
output_error_lat_model_all_free = NonlinearModel(zeros(5,3), output_error_lat_coeffs_all_free);

% Load output-error parameters
load("model_identification/output_error/results/output_error_lat_coeffs_final.mat");
output_error_lat_model_final = NonlinearModel(zeros(5,3), output_error_lat_coeffs_final);


coeffs = {equation_error_coeffs_lat, output_error_lat_coeffs, output_error_lat_coeffs_all_free, output_error_lat_coeffs_final};

% Lateral system
% State = [v p r phi]
% Input = [delta_a delta_r]

model_type = "lateral-directional";
maneuver_types = [
    "roll_211",...
    %"yaw_211",...
    ];

test_avl_models = false;
test_nonlin_models = true;
show_maneuver_plots = true;
show_error_metric_plots = true;
show_cr_bounds_plots = true;
show_param_map_plot = true;

for maneuver_type = maneuver_types
    num_maneuvers = length(fpr_data_lat.validation.(maneuver_type));
    error_metrics = {};
    
    for maneuver_i = 1:length(fpr_data_lat.validation.(maneuver_type))
        maneuver = fpr_data_lat.validation.(maneuver_type)(maneuver_i);
        t_sim = maneuver.Time;
        
        if test_avl_models
            model_names = ["NonlinearAvl"];
            models = {avl_nonlin_lat_model};
            [predicted_outputs, error_metrics_for_maneuver] = evaluate_models_on_maneuver(maneuver, models, model_names, model_type);

            % Add AVL state space model
            [predicted_outputs.StateSpaceAvl, error_metrics_for_maneuver.StateSpaceAvl] = evaluate_ss_model(maneuver, lat_sys, model_type);
            model_plot_styles = ["-" "-"];
            model_names = ["NonlinearAvl" "StateSpaceAvl"];
        end
        if test_nonlin_models
            models = {eq_error_lat_model, output_error_lat_model, output_error_lat_model_all_free, output_error_lat_model_final};
            model_names = ["EquationError" "OutputError" "OutputErrorAllFree" "OutputErrorFinal"];
            model_plot_styles = ["-" "-" "-" "-"];
            [predicted_outputs, error_metrics_for_maneuver] = evaluate_models_on_maneuver(maneuver, models, model_names, model_type);
        end
        if show_maneuver_plots
            maneuver.show_plot_lateral_validation(t_sim, predicted_outputs, model_names, model_plot_styles);
        end
        error_metrics{maneuver_i} = error_metrics_for_maneuver;
    end
end

if show_error_metric_plots
    if test_nonlin_models
        model_names = ["EquationError" "OutputError" "OutputErrorAllFree" "OutputErrorFinal"];
        signals_names = ["v","p","r","\phi"];
        signals_names_latex = ["$v$","$p$","$r$","$\phi$"];
        
        [gof_means, tic_means, an_means] = collect_mean_error_metrics_from_models(error_metrics, model_names);
        
        [gof_means] = collect_structs_into_array(gof_means, model_names);
        create_bar_plot(gof_means, model_names, "Goodness-of-Fit (GOF)", signals_names, signals_names_latex);
        
        [tic_means] = collect_structs_into_array(tic_means, model_names);
        create_bar_plot(tic_means, model_names, "Theils-Inequality-Coefficient (TIC)", signals_names, signals_names_latex);
        
        [an_means] = collect_structs_into_array(an_means, model_names);
        create_bar_plot(an_means, model_names, "Average-Normalized Error Measures", ["ANMAE","ANRMSE"], ["ANMAE","ANRMSE"]);
    end
end

if show_cr_bounds_plots
    load("model_identification/output_error/results/output_error_lat_cr_bounds.mat");
    load("model_identification/output_error/results/output_error_lat_all_free_cr_bounds.mat");
    load("model_identification/output_error/results/output_error_lat_final_cr_bounds.mat");
    cr_bounds = {zeros(size(output_error_lat_cr_bounds)) output_error_lat_cr_bounds output_error_lat_all_free_cr_bounds output_error_lat_final_cr_bounds};

    param_names = ["cY0" "cYb" "cYp" "cYr" "cYda" "cYdr"...
        "cl0" "clb" "clp" "clr" "clda" "cldr"...
        "cn0" "cnb" "cnp" "cnr" "cnda" "cndr"];
    param_names_latex = ["$c_{Y 0}$" "$c_{Y \beta}$" "$c_{Y p}$" "$c_{Y r}$" "$c_{Y {\delta_a}}$" "$c_{Y {\delta_r}}$"...
        "$c_{l 0}$" "$c_{l \beta}$" "$c_{l p}$" "$c_{l r}$" "$c_{l {\delta_a}}$" "$c_{l {\delta_r}}$"...
        "$c_{n 0}$" "$c_{n \beta}$" "$c_{n p}$" "$c_{n r}$" "$c_{n {\delta_a}}$" "$c_{n {\delta_r}}$"];

    cr_bounds_percentage = create_cr_bounds_percentages(model_names, cr_bounds, coeffs);
    
    [cr_bounds_percentage] = collect_structs_into_array(cr_bounds_percentage, model_names);
    create_bar_plot(cr_bounds_percentage, model_names, "2CR %", param_names, param_names_latex);

    cr_bound_means = mean(cr_bounds_percentage,2);
    create_bar_plot(cr_bound_means, model_names, "2CR %", "Average CR Bound", param_names_latex);
end

if show_param_map_plot
    coeffs_all = [avl_coeffs_lat coeffs];
    cr_bounds_all = [zeros(size(output_error_lat_cr_bounds)) cr_bounds];
    
    figure
    num_models = numel(coeffs_all);
    for param_i = 1:numel(coeffs_all{1})
        for model_i = 1:num_models
            [num_coeffs, num_regressors] = size(coeffs_all{model_i});
            subplot(num_coeffs,num_regressors,param_i);
            param = coeffs_all{model_i}(param_i);
            cr_bound = cr_bounds_all{model_i}(param_i);
            stddev = 2 * sqrt(cr_bound); % Approximation taken from Tischler, see (29) in Dorobantu2013
            errorbar(model_i,param,stddev,'-s','MarkerSize',10); hold on
            xlim([0 num_models+1])
            set(gca,'xticklabel',{[]})
            title(param_names_latex(param_i))
        end
    end
    legend(["AVL" model_names],'Location','SouthEast')
end
