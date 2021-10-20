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
    error_metrics_for_each_maneuver = {}; % Not used in the final work, instead the error metrics for all collected maneuvers is used
    for maneuver_type = maneuver_types
        simulation_data.(maneuver_type) = {};
        error_metrics_for_each_maneuver.(maneuver_type) = {};
        num_maneuvers = length(fpr_data.validation.(maneuver_type));
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
            simulation_data.(maneuver_type){maneuver_i} = simulation_data_curr_maneuver;
            error_metrics_for_each_maneuver.(maneuver_type){maneuver_i} = error_metrics_for_maneuver;
            
        end
    end
    
    if show_maneuver_plots
        for maneuver_type = maneuver_types 
            plot_validation_maneuvers(plot_title, simulation_data.(maneuver_type), model_type, model_names, plot_styles, model_names_to_display, num_maneuvers_to_plot);
        end
    end

    if show_error_metric_plots
        if test_nonlin_models
            collected_simulated_outputs = collect_all_simulated_outputs(simulation_data, maneuver_types, model_names);
            collected_recorded_outputs = collect_all_recorded_outputs(simulation_data, maneuver_types);
            [collected_mae, collected_rmse, collected_nmae, collected_nrmse, collected_gof, collected_tic] ...
                = calculate_error_metrics_for_models(model_names, collected_simulated_outputs, collected_recorded_outputs);
            
            %[gof_means, tic_means, an_means, mae_mean, rmse_mean] = collect_mean_error_metrics_from_models(error_metrics_for_each_maneuver, model_names, maneuver_types);

            [collected_gof] = collect_structs_into_array(collected_gof, model_names);
            create_bar_plot(collected_gof, model_names_to_display, "Goodness-of-Fit (GOF)", state_names, state_names_latex);

            [collected_tic] = collect_structs_into_array(collected_tic, model_names);
            create_bar_plot(collected_tic, model_names_to_display, "Theils-Inequality-Coefficient (TIC)", state_names, state_names_latex);
            
            [collected_nmae] = collect_structs_into_array(collected_nmae, model_names);
            create_bar_plot(collected_nmae, model_names_to_display, "Normalized Mean-Absolute-Errors (NMAE)", state_names, state_names_latex);
            
            [collected_nrmse] = collect_structs_into_array(collected_nrmse, model_names);
            create_bar_plot(collected_nrmse, model_names_to_display, "Normalized Root-Mean-Squared-Errors (NRMSE)", state_names, state_names_latex);

            disp("MAE: ")
            print_metric_for_latex(collected_mae, model_type, model_names);

            disp("RMSE: ")
            print_metric_for_latex(collected_rmse, model_type, model_names);
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

function simulated_outputs = collect_all_simulated_outputs(simulation_data, maneuver_types, model_names)
    simulated_outputs = {};
    for model_name = model_names
        simulated_outputs.(model_name) = [];
        for maneuver_type = maneuver_types
            for maneuver_i = 1:numel(simulation_data.(maneuver_type))
                curr_simulation_data = simulation_data.(maneuver_type){maneuver_i};
                simulated_outputs.(model_name) = [simulated_outputs.(model_name);
                                                  curr_simulation_data.Models.(model_name)];
            end
        end
    end
end

function recorded_outputs = collect_all_recorded_outputs(simulation_data, maneuver_types)
    recorded_outputs = [];
    for maneuver_type = maneuver_types
        for maneuver_i = 1:numel(simulation_data.(maneuver_type))
            curr_simulation_data = simulation_data.(maneuver_type){maneuver_i};
            recorded_outputs = [recorded_outputs;
                                curr_simulation_data.RecordedData];
        end
    end
end

function [collected_mae, collected_rmse, collected_nmae, collected_nrmse, collected_gof, collected_tic] ...
    = calculate_error_metrics_for_models(model_names, collected_simulated_outputs, collected_recorded_outputs)
    collected_mae = {};
    collected_gof = {};
    collected_tic = {};
    collected_nrmse = {};
    collected_nmae = {};
    collected_rmse = {};

    z = collected_recorded_outputs;
    for model_name = model_names
        y = collected_simulated_outputs.(model_name);
        collected_mae.(model_name) = mean_absolute_error(y, z);
        collected_gof.(model_name) = goodness_of_fit(y, z);
        collected_tic.(model_name) = theils_ineq_coeff(y, z);
        collected_nrmse.(model_name) = norm_rmse(y, z);
        collected_nmae.(model_name) = norm_rmse(y, z);
        collected_rmse.(model_name) = root_mean_square_error(y, z);
    end
end

function print_metric_for_latex(metric, model_type, model_names)
    n_decimals = 3;
    for model_name = model_names
        model_metric = metric.(model_name);
        % Convert to degrees from radians
        if strcmp(model_type,"lateral-directional")
            model_metric = [model_metric(1) rad2deg(model_metric(2)) rad2deg(model_metric(3)) rad2deg(model_metric(4))];
        elseif strcmp(model_type,"longitudinal")
            model_metric = [model_metric(1) model_metric(2) rad2deg(model_metric(3)) rad2deg(model_metric(4))];
        end
        disp(model_name)
        disp(sprintf('%4.3f %4.3f %4.3f %4.3f',round(model_metric,n_decimals)));
    end
end