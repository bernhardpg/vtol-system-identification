function [] = validate_models(...
    model_type, test_avl_models, test_nonlin_models, show_maneuver_plots,...
    state_names,state_names_latex, param_names, param_names_latex,...
    show_error_metric_plots, show_cr_bounds_plots, show_param_map_plot,...
    maneuver_types, models, model_coeffs, model_names, cr_bounds, fpr_data...
    )
    % Maneuver prediction
    for maneuver_type = maneuver_types
        num_maneuvers = length(fpr_data.validation.(maneuver_type));
        error_metrics = {};
        for maneuver_i = 1:num_maneuvers
            maneuver = fpr_data.validation.(maneuver_type)(maneuver_i);
            t_sim = maneuver.Time;

    %         if test_avl_models
    %             model_names = ["NonlinearAvl"];
    %             models = {avl_nonlin_lat_model};
    %             [predicted_outputs, error_metrics_for_maneuver] = evaluate_models_on_maneuver(maneuver, models, model_names, model_type);
    % 
    %             % Add AVL state space model
    %             [predicted_outputs.StateSpaceAvl, error_metrics_for_maneuver.StateSpaceAvl] = evaluate_ss_model(maneuver, lat_sys, model_type);
    %             model_plot_styles = ["-" "-"];
    %             model_names = ["NonlinearAvl" "StateSpaceAvl"];
    %         end

            if test_nonlin_models
                model_plot_styles = ["-" "-" "-" "-"];
                [predicted_outputs, error_metrics_for_maneuver] = evaluate_models_on_maneuver(maneuver, models, model_names, model_type);
            end
            if show_maneuver_plots
                if strcmp(model_type, "lateral-directional")
                    maneuver.show_plot_lateral_validation(t_sim, predicted_outputs, model_names, model_plot_styles);
                elseif strcmp(model_type, "longitudinal")
                    maneuver.show_plot_longitudinal_validation(t_sim, predicted_outputs, model_names, model_plot_styles);
                end
            end
            error_metrics{maneuver_i} = error_metrics_for_maneuver;
        end
    end

    if show_error_metric_plots
        if test_nonlin_models
            [gof_means, tic_means, an_means] = collect_mean_error_metrics_from_models(error_metrics, model_names);

            [gof_means] = collect_structs_into_array(gof_means, model_names);
            create_bar_plot(gof_means, model_names, "Goodness-of-Fit (GOF)", state_names, state_names_latex);

            [tic_means] = collect_structs_into_array(tic_means, model_names);
            create_bar_plot(tic_means, model_names, "Theils-Inequality-Coefficient (TIC)", state_names, state_names_latex);

            [an_means] = collect_structs_into_array(an_means, model_names);
            create_bar_plot(an_means, model_names, "Average-Normalized Error Measures", ["ANMAE","ANRMSE"], ["ANMAE","ANRMSE"]);
        end
    end

    if show_cr_bounds_plots
        cr_bounds_percentage = create_cr_bounds_percentages(model_names, cr_bounds, model_coeffs);

        [cr_bounds_percentage] = collect_structs_into_array(cr_bounds_percentage, model_names);
        create_bar_plot(cr_bounds_percentage, model_names, "2CR %", param_names, param_names_latex);

        cr_bound_means = mean(cr_bounds_percentage,2);
        create_bar_plot(cr_bound_means, model_names, "2CR %", "Average CR Bound", param_names_latex);
    end

    if show_param_map_plot
        figure
        num_models = numel(model_coeffs);
        for param_i = 1:numel(model_coeffs{1})
            for model_i = 1:num_models
                [num_coeffs, num_regressors] = size(model_coeffs{model_i});
                subplot(num_coeffs,num_regressors,param_i);
                param = model_coeffs{model_i}(param_i);
                cr_bound = cr_bounds{model_i}(param_i);
                stddev = 2 * sqrt(cr_bound); % Approximation taken from Tischler, see (29) in Dorobantu2013
                errorbar(model_i,param,stddev,'-s','MarkerSize',10); hold on
                xlim([0 num_models+1])
                set(gca,'xticklabel',{[]})
                title(param_names_latex(param_i))
            end
        end
        legend(model_names,'Location','SouthEast')
    end
end