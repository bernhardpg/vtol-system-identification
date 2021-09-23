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
load("model_identification/output_error/results/output_error_lon_coeffs.mat");
output_error_lon_model = NonlinearModel(output_error_lon_coeffs, zeros(6,3));

load("model_identification/output_error/results/output_error_lon_all_free_coeffs.mat");
output_error_lon_model_all_free = NonlinearModel(output_error_lon_all_free_coeffs, zeros(6,3));

load("model_identification/output_error/results/output_error_coeffs_lon_final_coeffs.mat");
output_error_lon_model_final = NonlinearModel(output_error_coeffs_lon_final_coeffs, zeros(6,3));

%%
% Longitudinal System
% State = [u w q theta]
% Input = [delta_e delta_t]

maneuver_types = ["pitch_211"];
test_avl_models = false;
test_nonlin_models = true;
show_maneuver_plots = false;
show_error_metric_plots = true;

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
            if show_maneuver_plots
                maneuver.show_plot_longitudinal_validation(t_sim, y_all_models, model_names, plot_styles);
            end
        end
        if test_nonlin_models
            [y_eq_error, error_calculations] = evaluate_model(maneuver, eq_error_lon_model);
            error_metrics.eq_error{maneuver_i} = error_calculations;
            
            [y_output_error, error_calculations] = evaluate_model(maneuver, output_error_lon_model);
            error_metrics.output_error{maneuver_i} = error_calculations;
            
            [y_output_error_all_params, error_calculations] = evaluate_model(maneuver, output_error_lon_model_all_free);
            error_metrics.y_output_error_all_params{maneuver_i} = error_calculations;
            
            [y_output_error_specific_params, error_calculations] = evaluate_model(maneuver, output_error_lon_model_final);
            error_metrics.y_output_error_specific_params{maneuver_i} = error_calculations;
            
            % Collect all simulations
            y_all_models = {y_eq_error y_output_error y_output_error_all_params y_output_error_specific_params};

            % Compare with real flight data
            model_names = ["Real data" "Equation-Error" "Output-Error" "Output-Error (all params)" "Output-Error (some params)"];
            plot_styles = ["--" "-" ":" ":" "-"];
            if show_maneuver_plots
                maneuver.show_plot_longitudinal_validation(t_sim, y_all_models, model_names, plot_styles);
            end
        end
    end
end

if show_error_metric_plots
    if test_avl_models
        [gof_avl_ss, tic_avl_ss, an_avl_ss] = evaluate_error_metrics(error_metrics.avl_ss);
        [gof_avl_nonlin, tic_avl_nonlin, an_avl_nonlin] = evaluate_error_metrics(error_metrics.avl_nonlin);

        model_names = ["State-Space (AVL)", "Nonlinear (AVL)"];
        create_bar_plot([gof_avl_ss; gof_avl_nonlin], model_names, "Goodness-of-Fit (GOF)", ["u","w","q","\theta"]);
        create_bar_plot([tic_avl_ss; tic_avl_nonlin], model_names, "Theils-Inequality-Coefficient (TIC)", ["u","w","q","\theta"]);
        create_bar_plot([an_avl_ss; an_avl_nonlin], model_names, "Average-Normalized Error Measures", ["ANMAE","ANRMSE"]);
    end

    if test_nonlin_models
        [gof_eq_error, tic_eq_error, an_eq_error] = evaluate_error_metrics(error_metrics.eq_error);
        [gof_output_error, tic_output_error, an_output_error] = evaluate_error_metrics(error_metrics.output_error);
        [gof_output_error_all_free, tic_output_error_all_free, an_output_error_all_free] = evaluate_error_metrics(error_metrics.y_output_error_all_params);
        [gof_output_error_specific, tic_output_error_specific, an_output_error_specific] = evaluate_error_metrics(error_metrics.y_output_error_specific_params);

        model_names = ["Equation-Error", "Output-Error", "Output-Error (all free)", "Output-Error (some free)"];
        create_bar_plot([gof_eq_error; gof_output_error; gof_output_error_all_free; gof_output_error_specific], model_names, "Goodness-of-Fit (GOF)", ["u","w","q","\theta"], ["$u$","$w$","$q$","$\theta$"]);
        create_bar_plot([tic_eq_error; tic_output_error; tic_output_error_all_free; tic_output_error_specific], model_names, "Theils-Inequality-Coefficient (TIC)", ["u","w","q","\theta"], ["$u$","$w$","$q$","$\theta$"]);
        create_bar_plot([an_eq_error; an_output_error; an_output_error_all_free; an_output_error_specific], model_names, "Average-Normalized Error Measures", ["ANMAE","ANRMSE"], ["ANMAE","ANRMSE"]);
    end
end

load("model_identification/output_error/results/output_error_lon_cr_bounds.mat");
load("model_identification/output_error/results/output_error_lon_all_free_cr_bounds.mat");
%load("model_identification/output_error/results/output_error_coeffs_lon_final_cr_bounds.mat");

% model_params = {output_error_coeffs_lon output_error_lon_all_free_coeffs output_error_coeffs_lon_final_coeffs};
% model_variances = {output_error_lon_cr_bounds output_error_lon_all_free_cr_bounds output_error_coeffs_lon_final_cr_bounds};

output_error_cr_bounds_percentage = calc_percentage_cr_bound(output_error_lon_coeffs, output_error_lon_cr_bounds);
output_error_all_free_cr_bounds_percentage = calc_percentage_cr_bound(output_error_lon_all_free_coeffs, output_error_lon_all_free_cr_bounds);
output_error_final_cr_bounds_percentage = calc_percentage_cr_bound(output_error_coeffs_lon_final_coeffs, output_error_coeffs_lon_final_cr_bounds);

param_names = ["cD0" "cDa" "cDa2" "cDq" "cDde" "cL0" "cLa" "cLa2" "cLq" "cLde" "cm0" "cma" "cma2" "cmq" "cmde"];
param_names_latex = ["$c_{D 0}$" "$c_{D \alpha}$" "$c_{D \alpha^2}$" "$c_{D q}$" "$c_{D {\delta_e}}$"...
    "$c_{L 0}$" "$c_{L \alpha}$" "$c_{L \alpha^2}$" "$c_{L q}$" "$c_{L {\delta_e}}$"...
    "$c_{m 0}$" "$c_{m \alpha}$" "$c_{m \alpha^2}$" "$c_{m q}$" "$c_{m {\delta_e}}$"];

create_bar_plot([output_error_cr_bounds_percentage output_error_all_free_cr_bounds_percentage output_error_final_cr_bounds_percentage], ["Output-Error" "Output-Error (all params)" "Output-Error (specific params)"], "2CR %", param_names, param_names_latex);

cr_bounds_means = [mean(fillmissing(output_error_cr_bounds_percentage,'constant',0)) mean(fillmissing(output_error_all_free_cr_bounds_percentage, 'constant',0)) mean(fillmissing(output_error_final_cr_bounds_percentage, 'constant',0))];
create_bar_plot(cr_bounds_means, ["Output-Error" "Output-Error (all params)" "Output-Error (specific params)"], "2CR %", "Average CR Bound", param_names_latex);

%%
model_params = {avl_coeffs_lon equation_error_coeffs_lon output_error_coeffs_lon output_error_coeffs_lon_all_params output_error_coeffs_lon_specific_params};
model_variances = {zeros(15,1) zeros(15,1) output_error_lon_variances output_error_lon_all_params_variances output_error_lon_specific_params_variances};

num_models = numel(model_params);
figure
for param_i = 1:15
    for model_i = 1:num_models
        subplot(3,5,param_i);
        param = model_params{model_i}(param_i);
        covar = model_variances{model_i}(param_i);
        stddev = 2 * sqrt(covar); % Approximation taken from Tischler, see (29) in Dorobantu2013
        cr_percentage = abs(stddev / param) * 100;
        errorbar(model_i,param,stddev,'-s','MarkerSize',10); hold on
        xlim([0 num_models+1])
        set(gca,'xticklabel',{[]})
        title(param_names_latex(param_i))
    end
end
legend(["AVL" "Equation-Error" "Output-Error" "Output-Error (all params)" "Output-Error (specific params)"],'Location','SouthEast')