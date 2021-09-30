classdef OutputErrorProblem
    properties
        ModelType
        ModelSpecific
        DynamicsModel
        TrainingData
        ValidationData
        ManeuverTypes
        InitialParams
        OptData
        ManeuverData
        ManeuverSimulations
        ParametersToUpdate
        ParamPerturbation = 0.01
        LineSearchRes = 10
    end
    methods
        function obj = OutputErrorProblem(model_type, fpr_data, dynamics_model, maneuver_types, params_to_update, weight)
            obj.ModelType = model_type;
            obj.TrainingData = obj.collect_all_maneuvers_into_array(fpr_data.training);
            obj.ValidationData = obj.collect_all_maneuvers_into_array(fpr_data.validation);
            obj.DynamicsModel = dynamics_model;
            obj.ManeuverTypes = maneuver_types;
            obj.OptData.iteration_number = 0;
            obj.OptData.reached_convergence = false;
            obj.OptData.params.CoeffsLat = dynamics_model.CoeffsLat;
            obj.OptData.params.CoeffsLon = dynamics_model.CoeffsLon;
            obj.InitialParams.CoeffsLat = dynamics_model.CoeffsLat;
            obj.InitialParams.CoeffsLon = dynamics_model.CoeffsLon;
            obj.OptData.costs = [];
            obj.OptData.iteration_indices = 0;
            obj.OptData.UpdatedRIndices = 0;
            obj.OptData.parameters_converged = false;
            obj.OptData.error_covariance_converged = false;
            obj.ManeuverData = {};
            obj.ParametersToUpdate = params_to_update;
            
            % Collect simulation data for all maneuvers
            for maneuver_i = 1:length(obj.TrainingData)
                obj.ManeuverData{maneuver_i} = obj.create_maneuver_simulation_data(obj.TrainingData(maneuver_i));
            end
            
            obj.OptData.N_total = obj.sum_datapoints_for_all_maneuvers();
            
            % Create functions etc specific to current model type
            if obj.ModelType == "longitudinal"
                obj.ModelSpecific.curr_params = obj.OptData.params.CoeffsLon;
                obj.ModelSpecific.sim_model = @(lat_params, lon_params, maneuver_i) obj.sim_model_lon(lat_params, lon_params, maneuver_i);
                obj.ModelSpecific.calc_residuals = @(maneuver, y_pred) obj.calc_residuals(maneuver.lon_z, y_pred);
                obj.ModelSpecific.f_calc_y = @(params) obj.get_y_pred_for_maneuvers(obj.OptData.params.CoeffsLat, params, obj.TrainingData);
                obj.ModelSpecific.simulate_maneuvers = @(params, fpr_data) obj.simulate_maneuvers(obj.OptData.params.CoeffsLat, params, fpr_data);
            elseif obj.ModelType == "lateral-directional"
                obj.ModelSpecific.curr_params = obj.OptData.params.CoeffsLat;
                obj.ModelSpecific.sim_model = @(lat_params, lon_params, maneuver_i) obj.sim_model_lat(lat_params, lon_params, maneuver_i);
                obj.ModelSpecific.calc_residuals = @(maneuver, y_pred) obj.calc_residuals(maneuver.lat_z, y_pred);
                obj.ModelSpecific.f_calc_y = @(params) obj.get_y_pred_for_maneuvers(params, obj.OptData.params.CoeffsLon, obj.TrainingData);
                obj.ModelSpecific.simulate_maneuvers = @(params, fpr_data) obj.simulate_maneuvers(params, obj.OptData.params.CoeffsLon, fpr_data);
            end
        end
        
        function maneuver_array = collect_all_maneuvers_into_array(~, dataset)
            maneuver_array = [];
            maneuver_types = fieldnames(dataset);
            for i = 1:numel(maneuver_types)
                maneuver_type = maneuver_types{i};
                maneuvers = dataset.(maneuver_type);
                maneuver_array = [maneuver_array; maneuvers];
            end
        end
        
        function obj = solve(obj)
            % Evaluate initial parameters
            obj.ManeuverSimulations = obj.simulate_maneuvers(obj.InitialParams.CoeffsLat, obj.InitialParams.CoeffsLon, obj.TrainingData);
            [obj.OptData.cost, obj.OptData.residuals, obj.OptData.R_hat]...
                = obj.calc_cost_for_maneuvers(obj.ManeuverSimulations);
            obj.OptData.costs = obj.OptData.cost;

            % Plot cost history
            figure;
            fig_cost_axes = axes;
            obj.draw_cost_function_history(fig_cost_axes, obj.OptData.iteration_indices, obj.OptData.costs, obj.OptData.UpdatedRIndices);
            
            % Optimization routine
            iteration = 0;
            while ~obj.OptData.error_covariance_converged
                newton_raphson_step_i = 0;
                while ~obj.OptData.parameters_converged
                    % Optimization step
                    iteration = iteration + 1;
                    newton_raphson_step_i = newton_raphson_step_i + 1;
                    disp("Performing Newton-Raphson step: " + newton_raphson_step_i)
                    
                    disp("	Calculating gradients")
                    tic
                    [params_update, information_matrix, cost_gradient]...
                        = obj.calc_params_update(obj.ParametersToUpdate, obj.OptData.R_hat, obj.OptData.residuals, obj.ModelSpecific.curr_params, obj.ModelSpecific.f_calc_y);
                    toc
                    
                    obj.OptData.CramerRaoLowerBound = diag(inv(information_matrix));
                    
                    % Create params update matrix
                    params_update_matrix = zeros(size(obj.ModelSpecific.curr_params));
                    for i = 1:length(obj.ParametersToUpdate)
                        params_update_matrix(obj.ParametersToUpdate(i)) = params_update(i);
                    end

                    % Perform simple line search to find step size
                    % which minimizes cost along current direction
                    disp("	Performing line search")
                    tic
                    alpha = obj.do_line_search(obj.ModelSpecific.curr_params, params_update_matrix);
                    toc

                    % Update parameters with chosen alpha
                    params_new = obj.ModelSpecific.curr_params + alpha * params_update_matrix;

                    % Evaluate new model performance
                    maneuver_simulations = obj.ModelSpecific.simulate_maneuvers(params_new, obj.TrainingData);
                    [cost_new, residuals_new] = obj.calc_cost_for_maneuvers_with_const_noise_covar(maneuver_simulations, obj.OptData.R_hat);
               
                    % Check convergence
                    [obj.OptData.parameters_converged, obj.OptData.ConvergenceReason]...
                        = obj.check_for_param_convergence(...
                            obj.ModelSpecific.curr_params, params_new, ...
                            obj.OptData.cost, cost_new, cost_gradient...
                            );

                    % Save all data and move on to next iteration
                    obj.ModelSpecific.curr_params = params_new;
                    obj.OptData.residuals = residuals_new;
                    obj.OptData.cost = cost_new;
                    obj.OptData.costs = [obj.OptData.costs obj.OptData.cost];
                    obj.OptData.iteration_indices = [obj.OptData.iteration_indices iteration];

                    obj.draw_cost_function_history(fig_cost_axes, obj.OptData.iteration_indices, obj.OptData.costs, obj.OptData.UpdatedRIndices);
                end

                % Update noise covariance matrix
                disp("== Updating R_hat ==")
                obj.OptData.UpdatedRIndices = [obj.OptData.UpdatedRIndices iteration];
                iteration = iteration - 1;
                obj.draw_cost_function_history(fig_cost_axes, obj.OptData.iteration_indices, obj.OptData.costs, obj.OptData.UpdatedRIndices);
                R_hat_new = obj.est_noise_covariance(obj.OptData.residuals, obj.OptData.N_total);

                % Start optimization routine over again
                obj.OptData.parameters_converged = false;

                % Check for convergence
                [obj.OptData.error_covariance_converged, obj.OptData.ConvergenceReason] ...
                    = obj.check_for_noise_covar_convergence(obj.OptData.R_hat, R_hat_new);
                obj.OptData.R_hat = R_hat_new;
            end
        end
        
        function maneuver_simulations = simulate_maneuvers(obj, params_lat, params_lon, fpr_data)
            maneuver_simulations = {};
            for maneuver_i = 1:length(fpr_data)
                curr_simulation = {};
                curr_simulation.y_pred = obj.ModelSpecific.sim_model(params_lat, params_lon, maneuver_i);
                curr_simulation.residuals = obj.ModelSpecific.calc_residuals(obj.ManeuverData{maneuver_i}, curr_simulation.y_pred);
                maneuver_simulations{maneuver_i} = curr_simulation;
            end
        end
        
        function y_pred = get_y_pred_for_maneuvers(obj, params_lat, params_lon, fpr_data)
            maneuver_simulations = obj.simulate_maneuvers(params_lat, params_lon, fpr_data);
            y_pred = [];
            for maneuver_i = 1:numel(maneuver_simulations)
                y_pred = [y_pred;
                          maneuver_simulations{maneuver_i}.y_pred];
            end
        end

        function residuals = aggregate_residuals_from_all_maneuvers(~, maneuver_simulations)
            residuals = [];
            for maneuver_i = 1:numel(maneuver_simulations)
                residuals = [residuals; maneuver_simulations{maneuver_i}.residuals];
            end
        end
        
        function N_total = sum_datapoints_for_all_maneuvers(obj)
            N_total = 0;
            for maneuver_i = 1:length(obj.TrainingData)
                N_total = N_total + obj.ManeuverData{maneuver_i}.N;
            end
        end
        
        function [has_converged, convergence_reason] = check_for_noise_covar_convergence(~, R_hat_old, R_hat_new)
            has_converged = false;
            convergence_reason = "Not converged";
            
            abs_noise_covar_change = abs(diag(R_hat_old - R_hat_new) ./ (diag(R_hat_old)));
            if all((abs_noise_covar_change) < 0.01)
                has_converged = true;
                convergence_reason = "All covariances smaller than specified treshold";
                disp("Converged: " + convergence_reason);
            end 
        end
        
        function [has_converged, convergence_reason] = check_for_param_convergence(~, params_old, params_new, cost_old, cost_new, cost_gradient)
            has_converged = false;
            convergence_reason = "Not converged";
            
            eucl_params_change = norm(reshape((params_new - params_old),[numel(params_old) 1]),2);
            eucl_prev_params = norm(reshape((params_old),[numel(params_old) 1]),2);
            param_change_ratio = eucl_params_change / eucl_prev_params;
            if param_change_ratio < 0.001
               has_converged = true;
               convergence_reason = "Parameter change smaller than specified threshold";
               disp("Converged: " + convergence_reason);
            end

            abs_cost_change = abs((cost_new - cost_old) / cost_old);
            if abs_cost_change < 0.001
               has_converged = true;
               convergence_reason = "Cost change smaller than specified threshold";
               disp("Converged: " + convergence_reason);
            end

            if all(abs(cost_gradient) < 0.01)
               has_converged = true;
               convergence_reason = "All gradients smaller than specificed treshold";
               disp("Converged: " + convergence_reason);
            end
        end
        
        function alpha = do_line_search(obj, theta_0, delta_theta)
            min_cost = inf;
            for potential_alpha = linspace(0.05,1,obj.LineSearchRes)
                % Update parameters
                theta_new = theta_0 + potential_alpha * delta_theta;

                maneuver_simulations = obj.ModelSpecific.simulate_maneuvers(theta_new, obj.TrainingData);
                [cost_new, ~] = obj.calc_cost_for_maneuvers_with_const_noise_covar(maneuver_simulations, obj.OptData.R_hat);
                if cost_new < min_cost
                    min_cost = cost_new;
                    alpha = potential_alpha;
                end
            end
        end
        
        function [cost, residuals, R_hat] = calc_cost_for_maneuvers(obj, maneuver_simulations)            
            % Calculate all residuals
            residuals = obj.aggregate_residuals_from_all_maneuvers(maneuver_simulations);
            
            % Calculate noise covariance matrix from all maneuvers
            R_hat = obj.est_noise_covariance(residuals, obj.OptData.N_total);

            % Calculate cost from all maneuvers
            cost = obj.calc_cost(R_hat, residuals);
        end
        
        function [cost, residuals] = calc_cost_for_maneuvers_with_const_noise_covar(obj, maneuver_simulations, R_hat)            
            % Calculate all residuals
            residuals = obj.aggregate_residuals_from_all_maneuvers(maneuver_simulations);

            % Calculate cost from all maneuvers
            cost = obj.calc_cost(R_hat, residuals);
        end
        
        function draw_cost_function_history(obj, fig_axes, iteration_indices, costs, r_updated_indices)

            plot(fig_axes, iteration_indices, costs); hold on
            for i = r_updated_indices
                xline(i, ":", "Updated $\hat{R}$",'interpreter','latex');    
            end
            hold off;
            title("Cost function")
            xlabel("Iteration")
            drawnow;
        end
        
        function data = create_maneuver_simulation_data(~, maneuver)
            data = {};
            data.t_data_sequence = maneuver.Time;
            data.tspan = [maneuver.Time(1) maneuver.Time(end)];
            data.N = length(maneuver.Time);
            data.lat_z = maneuver.get_lat_state_sequence();
            data.lon_z = maneuver.get_lon_state_sequence();
            data.lat_y_0 = maneuver.get_lat_state_initial();
            data.lon_y_0 = maneuver.get_lon_state_initial();
            data.lon_state_sequence = maneuver.get_lon_state_sequence();
            data.lat_state_sequence = maneuver.get_lat_state_sequence();
            data.lon_input_sequence = maneuver.get_lon_input_sequence();
            data.lat_input_sequence = maneuver.get_lat_input_sequence();
        end
        
        function [params_update, information_matrix, cost_gradient] = calc_params_update(obj, params_to_update, R_hat, residuals, params, f_calc_y)            
            [N, n_outputs] = size(residuals);
            n_params = numel(params_to_update);
            
            sensitivity_matrices = obj.calc_sensitivity_matrices(N, n_outputs, n_params, params_to_update, params, f_calc_y);
            
            % Calculate information matrix
            information_matrix_terms = zeros(n_params, n_params, N);
            for timestep_i = 1:N
                information_matrix_terms(:,:,timestep_i) = sensitivity_matrices(:,:,timestep_i)' * (R_hat \ sensitivity_matrices(:,:,timestep_i));    
            end
            information_matrix = sum(information_matrix_terms,3);
            
            % Calculate cost_gradient
            cost_gradient_all_i = zeros(n_params, 1, N);
            for timestep_i = 1:N
                cost_gradient_all_i(:,:,timestep_i) = sensitivity_matrices(:,:,timestep_i)' * (R_hat \ residuals(timestep_i,:)');    
            end
            cost_gradient = -sum(cost_gradient_all_i,3);
            
            % Compute param_update
            params_update = - information_matrix \ cost_gradient;
        end
        
        function sensitivity_matrices = calc_sensitivity_matrices(obj, N, n_outputs, n_params, params_to_update, params, f_calc_y)
            % Compute a sensitivity matrix (dy_dtheta) for each timestep
            sensitivity_matrices = zeros(N, n_outputs, n_params);
            for j = 1:n_params                
                % Compute sensitivies by using first order central differences
                delta_theta_j = params(params_to_update(j)) * obj.ParamPerturbation;

                if delta_theta_j == 0 % Handle parameters set to 0
                   delta_theta_j = obj.ParamPerturbation;
                end
                
                param_perturbation_matrix = zeros(size(params));
                param_perturbation_matrix(params_to_update(j)) = delta_theta_j;
                    
                y_pos = f_calc_y(params + param_perturbation_matrix);
                y_neg = f_calc_y(params - param_perturbation_matrix);

                dy_dtheta_j = (y_pos - y_neg) ./ (2 * delta_theta_j);
                sensitivity_matrices(:,:,j) = dy_dtheta_j;
            end
            sensitivity_matrices = permute(sensitivity_matrices,[2 3 1]); % Get S in the correct dimensions: (n_outputs, n_params, n_timesteps)
        end
        
        function y_pred = sim_model_lat(obj, lat_params, lon_params, maneuver_i)
            obj.DynamicsModel = obj.DynamicsModel.set_params(lat_params, lon_params);
            [t_pred, y_pred] = ode45(@(t,y) ...
                obj.DynamicsModel.dynamics_lat_model_c(t, y, ...
                    obj.ManeuverData{maneuver_i}.t_data_sequence, ...
                    obj.ManeuverData{maneuver_i}.lat_input_sequence, ...
                    obj.ManeuverData{maneuver_i}.lon_state_sequence), ...
                obj.ManeuverData{maneuver_i}.tspan, ...
                obj.ManeuverData{maneuver_i}.lat_y_0...
                );
            y_pred = interp1(t_pred, y_pred, obj.ManeuverData{maneuver_i}.t_data_sequence); % change y_pred to correct time before returning
        end
        
        function y_pred = sim_model_lon(obj, lat_params, lon_params, maneuver_i)
            obj.DynamicsModel = obj.DynamicsModel.set_params(lat_params, lon_params);
            [t_pred, y_pred] = ode45(@(t,y) ...
                obj.DynamicsModel.dynamics_lon_model_c(t, y, ...
                    obj.ManeuverData{maneuver_i}.t_data_sequence, ...
                    obj.ManeuverData{maneuver_i}.lon_input_sequence, ...
                    obj.ManeuverData{maneuver_i}.lat_state_sequence), ...
                obj.ManeuverData{maneuver_i}.tspan, ...
                obj.ManeuverData{maneuver_i}.lon_y_0...
                );
            y_pred = interp1(t_pred, y_pred, obj.ManeuverData{maneuver_i}.t_data_sequence); % change y_pred to correct time before returning
        end
        
        function v = calc_residuals(~, z, y_pred)
            v = z - y_pred; % residuals
        end

        function cost = calc_cost(~, R_hat, residuals)
            cost = 0.5 * sum(diag(residuals / R_hat * residuals'));
        end        
        
        function R_hat = est_noise_covariance(~, residuals, N)
            R_hat = diag(diag(residuals' * residuals) / N); % Assumes cross-covariances to be zero ny neglecting nondiagonal terms
        end
   end
end