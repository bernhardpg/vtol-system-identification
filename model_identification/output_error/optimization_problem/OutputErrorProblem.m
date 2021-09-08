classdef OutputErrorProblem
    properties
        DynamicsModel
        TrainingData
        ValidationData
        ManeuverTypes
        InitialParams
        OptData
        ManeuverData
        ManeuverSimulations
        ParamPerturbation = 0.01
    end
    methods
        function obj = OutputErrorProblem(fpr_data, dynamics_model, maneuver_types)
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
            obj.OptData.parameters_converged = false;
            obj.OptData.error_covariance_converged = false;
            obj.ManeuverData = {};
            
            % Collect simulation data for all maneuvers
            for maneuver_i = 1:length(obj.TrainingData)
                obj.ManeuverData{maneuver_i} = obj.create_maneuver_simulation_data(obj.TrainingData(maneuver_i));
            end
            
            obj.OptData.N_total = obj.sum_datapoints_for_all_maneuvers();
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
            obj.draw_cost_function_history(fig_cost_axes, obj.OptData.costs);

            % Optimization routine
            while ~obj.OptData.error_covariance_converged
                step_i = 0;
                while ~obj.OptData.parameters_converged
                    % Optimization step
                    step_i = step_i + 1;
                    disp("Performing Newton-Raphson step: " + step_i)
                    
                    tic
                    disp("	Calculating gradients")
                    [params_update, information_matrix, cost_gradient]...
                        = obj.calc_params_update(obj.OptData.R_hat, obj.OptData.residuals, obj.OptData.params.CoeffsLat,...
                            @(params) obj.get_y_pred_for_maneuvers(params, obj.OptData.params.CoeffsLon, obj.TrainingData));
                    toc

                    params_update_matrix = reshape(params_update, size(obj.OptData.params.CoeffsLat));

                    % Perform simple line search to find step size
                    % which minimizes cost along current direction
                    tic
                    disp("	Performing line search")
                    alpha = obj.do_line_search(obj.OptData.params.CoeffsLat, params_update_matrix);
                    toc

                    % Update parameters with chosen alpha
                    params_lat_new = obj.OptData.params.CoeffsLat + alpha * params_update_matrix;

                    % Evaluate new model performance
                    maneuver_simulations = obj.simulate_maneuvers(params_lat_new, obj.InitialParams.CoeffsLon, obj.TrainingData);
                    [cost_new, residuals_new] = obj.calc_cost_for_maneuvers_with_const_noise_covar(maneuver_simulations, obj.OptData.R_hat);
               
                    % Check convergence
                    [obj.OptData.parameters_converged, obj.OptData.ConvergenceReason]...
                        = obj.check_for_param_convergence(...
                            obj.OptData.params.CoeffsLat, params_lat_new, ...
                            obj.OptData.cost, cost_new, cost_gradient...
                            );

                    % Save all data and move on to next iteration
                    obj.OptData.params.CoeffsLat = params_lat_new;
                    obj.OptData.residuals = residuals_new;
                    obj.OptData.cost = cost_new;
                    obj.OptData.costs = [obj.OptData.costs obj.OptData.cost];

                    obj.draw_cost_function_history(fig_cost_axes, obj.OptData.costs);
                end

                % Update noise covariance matrix
                disp("Updating R_hat")
                R_hat_new = obj.est_noise_covariance(obj.OptData.residuals, obj.OptData.N_total);

                % Start optimization routine over again
                obj.OptData.parameters_converged = false;

                % Check for convergence
                [obj.OptData.error_covariance_converged, obj.OptData.ConvergenceReason] ...
                    = obj.check_for_noise_covar_convergence(obj.OptData.R_hat, R_hat_new);
                obj.OptData.R_hat = R_hat_new;
            end

            % Plot new model vs. initial model
            y_initial = obj.sim_model(obj.InitialParams.CoeffsLat, obj.InitialParams.CoeffsLon);

            maneuver.plot_lateral_validation({maneuver.Time, maneuver.Time}, {y_initial, obj.CurrManeuverData.y_pred}, ...
               ["Real data", "Initial", "New"], ["-", "--", "-"], true, false, "", "");
        end
        
        function maneuver_simulations = simulate_maneuvers(obj, params_lat, params_lon, fpr_data)
            maneuver_simulations = {};
            for maneuver_i = 1:length(fpr_data)
                curr_simulation = {};
                curr_simulation.y_pred = obj.sim_model(params_lat, params_lon, maneuver_i);
                curr_simulation.residuals = obj.calc_residuals_lat(obj.ManeuverData{maneuver_i}.z, curr_simulation.y_pred);
                maneuver_simulations{maneuver_i} = curr_simulation;
            end
        end
        
        function y_pred = get_y_pred_for_maneuvers(obj, params_lat, params_lon, fpr_data)
            maneuver_simulations = simulate_maneuvers(obj, params_lat, params_lon, fpr_data);
            y_pred = [];
            for maneuver_i = 1:numel(maneuver_simulations)
                y_pred = [y_pred;
                          maneuver_simulations{maneuver_i}.y_pred];
            end
        end

        function residuals = aggregate_residuals_from_all_maneuvers(obj, maneuver_simulations)
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
            if all((abs_noise_covar_change) < 0.1)
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
            if param_change_ratio < 0.01
               has_converged = true;
               convergence_reason = "Parameter change smaller than specified threshold";
               disp("Converged: " + convergence_reason);
            end

            abs_cost_change = abs((cost_new - cost_old) / cost_old);
            if abs_cost_change < 0.01
               has_converged = true;
               convergence_reason = "Cost change smaller than specified threshold";
               disp("Converged: " + convergence_reason);
            end

            if all(abs(cost_gradient) < 0.05)
               has_converged = true;
               convergence_reason = "All gradients smaller than specificed treshold";
               disp("Converged: " + convergence_reason);
            end
        end
        
        function alpha = do_line_search(obj, theta_0, delta_theta)
            min_cost = inf;
            for potential_alpha = linspace(0.05,1,20)
                % Update parameters
                theta_new_lat = theta_0 + potential_alpha * delta_theta;

                maneuver_simulations = obj.simulate_maneuvers(theta_new_lat, obj.InitialParams.CoeffsLon, obj.TrainingData);
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
            cost = obj.calc_cost_lat(R_hat, residuals);
        end
        
        function [cost, residuals] = calc_cost_for_maneuvers_with_const_noise_covar(obj, maneuver_simulations, R_hat)            
            % Calculate all residuals
            residuals = obj.aggregate_residuals_from_all_maneuvers(maneuver_simulations);

            % Calculate cost from all maneuvers
            cost = obj.calc_cost_lat(R_hat, residuals);
        end
        
        function draw_cost_function_history(obj, fig_axes, costs)
            plot(fig_axes, costs);
            title("Cost function")
            xlabel("Iteration")
            drawnow;
        end
        
        function data = create_maneuver_simulation_data(~, maneuver)
            data = {};
            data.t_data_sequence = maneuver.Time;
            data.tspan = [maneuver.Time(1) maneuver.Time(end)];
            data.N = length(maneuver.Time);
            data.z = maneuver.get_lat_state_sequence();
            data.y_0 = maneuver.get_lat_state_initial();
            data.lon_state_sequence = maneuver.get_lon_state_sequence();
            data.input_sequence = maneuver.get_lat_input_sequence();
        end
        
        function [params_update, information_matrix, cost_gradient] = calc_params_update(obj, R_hat, residuals, params, f_calc_y)            
            [N, n_outputs] = size(residuals);
            n_params = numel(params);

            % Compute a sensitivity matrix (dy_dtheta) for each timestep
            sensitivity_matrices = zeros(N, n_outputs, n_params);
            for j = 1:n_params
                % Compute sensitivies by using first order central differences
                delta_theta_j = params(j) * obj.ParamPerturbation;

                if delta_theta_j == 0 % Handle parameters set to 0
                   delta_theta_j = obj.ParamPerturbation;
                end
                
                param_perturbation_matrix = zeros(size(params));
                param_perturbation_matrix(j) = delta_theta_j;
                    
                y_pos = f_calc_y(params + param_perturbation_matrix);
                y_neg = f_calc_y(params - param_perturbation_matrix);

                dy_dtheta_j = (y_pos - y_neg) ./ (2 * delta_theta_j);
                sensitivity_matrices(:,:,j) = dy_dtheta_j;
            end
            sensitivity_matrices = permute(sensitivity_matrices,[2 3 1]); % Get S in the correct dimensions: (n_outputs, n_params, n_timesteps)
            
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
        
        function y_pred = sim_model(obj, lat_params, lon_params, maneuver_i)
            obj.DynamicsModel = obj.DynamicsModel.set_params(lat_params, lon_params);
            [t_pred, y_pred] = ode45(@(t,y) ...
                obj.DynamicsModel.dynamics_lat_model_c(t, y, ...
                    obj.ManeuverData{maneuver_i}.t_data_sequence, ...
                    obj.ManeuverData{maneuver_i}.input_sequence, ...
                    obj.ManeuverData{maneuver_i}.lon_state_sequence), ...
                obj.ManeuverData{maneuver_i}.tspan, ...
                obj.ManeuverData{maneuver_i}.y_0...
                );
            y_pred = interp1(t_pred, y_pred, obj.ManeuverData{maneuver_i}.t_data_sequence); % change y_pred to correct time before returning
        end
        
        function v = calc_residuals_lat(~, z, y_pred)
            v = z - y_pred; % residuals
        end

        function cost = calc_cost_lat(~, R_hat, residuals)
            cost = 0.5 * sum(diag(residuals / R_hat * residuals'));
        end        
        
        function R_hat = est_noise_covariance(~, residuals, N)
            R_hat = diag(diag(residuals' * residuals) / N); % Assumes cross-covariances to be zero ny neglecting nondiagonal terms
        end
   end
end