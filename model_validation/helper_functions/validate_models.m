function [] = validate_models(...
    plot_title,...
    model_type, test_avl_models, test_nonlin_models, show_maneuver_plots,...
    state_names,state_names_latex, param_names, param_names_latex,...
    show_error_metric_plots, show_cr_bounds_plots, show_param_map_plot,...
    maneuver_types, models, models_avl, model_coeffs, model_names, model_names_avl, cr_bounds, fpr_data,...
    model_names_avl_to_display, model_names_to_display...
    )
    % Maneuver prediction
    
    num_maneuvers_to_plot = 3;
    
    simulation_data = {};
    for maneuver_type = maneuver_types
        num_maneuvers = length(fpr_data.validation.(maneuver_type));
        error_metrics = {};
        error_metrics = {};
        for maneuver_i = 1:num_maneuvers
            maneuver = fpr_data.validation.(maneuver_type)(maneuver_i);
            t_sim = maneuver.Time;
            
            simulation_data_curr_maneuver = {};
            simulation_data_curr_maneuver.Time = maneuver.Time;
            
            if test_avl_models
                plot_styles = ["-" "-"];
                [predicted_outputs, error_metrics_for_maneuver] = evaluate_models_on_maneuver(maneuver, models_avl, model_names_avl, model_type);
                model_names = model_names_avl;
                model_names_to_display = model_names_avl_to_display;
                
%                 % Add AVL state space model
%                 % Import ss model from AVL
%                 avl_state_space_model;
%                 [predicted_outputs_avl.StateSpaceAvl, error_metrics_for_maneuver.StateSpaceAvl] = evaluate_ss_model(maneuver, lon_sys, model_type);
%                 
%                 simulation_data_curr_maneuver.Time = maneuver.Time;
%                 simulation_data_curr_maneuver.Models = predicted_outputs_avl;
%                 
%                 model_names = [model_names_avl "StateSpaceAvl"];
%                 model_names_to_display = [model_names_avl_to_display "State-Space VLM"];
            end

            if test_nonlin_models
                plot_styles = ["-" "-" "-" "-"];
                [predicted_outputs, error_metrics_for_maneuver] = evaluate_models_on_maneuver(maneuver, models, model_names, model_type);
                error_metrics{maneuver_i} = error_metrics_for_maneuver;
            end

            % Store simulation data
            if strcmp(model_type, "lateral-directional")
                simulation_data_curr_maneuver.RecordedData = [maneuver.VelV maneuver.AngP maneuver.AngR maneuver.EulPhi];
                simulation_data_curr_maneuver.Input = [maneuver.DeltaA maneuver.DeltaR];
            elseif strcmp(model_type, "longitudinal")
                simulation_data_curr_maneuver.RecordedData = [maneuver.VelU maneuver.VelW maneuver.AngQ maneuver.EulTheta];
                simulation_data_curr_maneuver.Input = [maneuver.DeltaE maneuver.DeltaT];
            end
            simulation_data_curr_maneuver.Models = predicted_outputs;
            simulation_data{maneuver_i} = simulation_data_curr_maneuver;
            error_metrics{maneuver_i} = error_metrics_for_maneuver;
            
        end
    end
    
    if show_maneuver_plots
        plot_validation_maneuvers(plot_title, simulation_data, model_type, model_names, plot_styles, model_names_to_display, num_maneuvers_to_plot);
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