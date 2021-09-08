classdef OutputErrorProblem
    properties
        DynamicsModel
        FprData
        ManeuverTypes
        OptData = {}
        ParamPerturbation = 0.01
        NParamsLat = 18
        NOutputsLat = 4
    end
    methods
        function obj = OutputErrorProblem(fpr_data, dynamics_model, maneuver_types)
            obj.FprData = fpr_data;
            obj.DynamicsModel = dynamics_model;
            obj.ManeuverTypes = maneuver_types;
            obj.OptData.iteration_number = 0;
            obj.OptData.reached_convergence = false;
            obj.OptData.params.CoeffsLat = dynamics_model.CoeffsLat;
            obj.OptData.params.CoeffsLon = dynamics_model.CoeffsLon;
        end
        
        function obj = solve(obj)
            for maneuver_type = obj.ManeuverTypes(1)
                for maneuver_i = 1
                    maneuver = obj.FprData.training.(maneuver_type)(maneuver_i);
                                            
                    % Initial evaluation
                    [obj.OptData.R_hat, obj.OptData.cost, obj.OptData.params.CoeffsLat] = evaluate_iteration_for_maneuver(obj, maneuver);
                    
                    % Run until convergence
                    while ~obj.OptData.reached_convergence
                        obj.OptData.iteration_number = obj.OptData.iteration_number + 1;
                        disp("Running iteration: " + obj.OptData.iteration_number);
                        
                        [R_hat_new, cost_new, params_new] = evaluate_iteration_for_maneuver(obj, maneuver);
                        obj.OptData.reached_convergence = check_for_convergence(obj, params_new, cost_new, R_hat_new);
                    end
                    disp("Reached convergence after " + obj.OptData.iteration_number + " iterations");
                end
            end
        end
        
        function [R_hat_new, cost_new, params_new] = evaluate_iteration_for_maneuver(obj, maneuver)
            % Get maneuver data
            t_data_sequence = maneuver.Time;
            tspan = [maneuver.Time(1) maneuver.Time(end)];
            N = length(t_data_sequence);
            z = maneuver.get_lat_state_sequence();
            y_0 = maneuver.get_lat_state_initial();
            lon_state_sequence = maneuver.get_lon_state_sequence();
            input_sequence = maneuver.get_lat_input_sequence();
            data_sequence = {t_data_sequence input_sequence lon_state_sequence};
            
            % Estimate noise covariance matrix (R_hat) for Newton-Raphson
            % step
            y_pred = obj.sim_model(data_sequence, y_0, tspan, obj.OptData.params.CoeffsLat, obj.OptData.params.CoeffsLon);
            residuals = obj.calc_residuals_lat(z, y_pred);
            R_hat = obj.est_noise_covariance(residuals, N);
            
            % Minimize the cost by updating the parameters while keeping
            % R_hat constant
            
            % Compute S (sensitivity matrices) for lateral parameters
            disp("Computing sensitivies")
            tic
            S_all_i = zeros(N, obj.NOutputsLat, obj.NParamsLat);
            for j = 1:obj.NParamsLat
                % Compute sensitivies by using first order central differences
                delta_theta_j = obj.OptData.params.CoeffsLat(j) * obj.ParamPerturbation;
                % Handle parameters set to 0
                if delta_theta_j == 0
                   delta_theta_j = obj.ParamPerturbation;
                end
                perturbed_params_lat_pos = obj.create_perturbed_params_matrix(delta_theta_j, obj.OptData.params.CoeffsLat, j, 1);
                perturbed_params_lat_neg = obj.create_perturbed_params_matrix(delta_theta_j, obj.OptData.params.CoeffsLat, j, -1);
                    
                %tic
                y_pos = obj.sim_model(data_sequence, y_0, tspan, perturbed_params_lat_pos, obj.OptData.params.CoeffsLon);
                y_neg = obj.sim_model(data_sequence, y_0, tspan, perturbed_params_lat_neg, obj.OptData.params.CoeffsLon);
                %toc
                
                dy_dtheta_j = (y_pos - y_neg) ./ (2 * delta_theta_j);
                S_all_i(:,:,j) = dy_dtheta_j;
            end
            S_all_i = permute(S_all_i,[2 3 1]); % Get S in the correct dimensions: (n_outputs, n_params, n_timesteps)
            toc
            
            % Calculate M (information matrix)
            disp("Computing information matrix")
            tic
            M_all_i = zeros(obj.NParamsLat, obj.NParamsLat, N);
            for timestep_i = 1:N
                M_all_i(:,:,timestep_i) = S_all_i(:,:,timestep_i)' * (R_hat \ S_all_i(:,:,timestep_i));    
            end
            M = sum(M_all_i,3);
            toc
            
            % Calculate g
            disp("Computing g")
            tic
            g_all_i = zeros(obj.NParamsLat, 1, N);
            for timestep_i = 1:N
                g_all_i(:,:,timestep_i) = S_all_i(:,:,timestep_i)' * (R_hat \ residuals(timestep_i,:)');    
            end
            g = sum(g_all_i,3);
            toc
            
            % Compute param_update
            delta_theta = - M \ g;
            
            % Update parameters
            params_new = obj.OptData.params.CoeffsLat + reshape(delta_theta, size(obj.OptData.params.CoeffsLat));
            
            % Estimate noise covariance matrix
            y_pred = obj.sim_model(data_sequence, y_0, tspan, obj.OptData.params.CoeffsLat, obj.OptData.params.CoeffsLon);
            residuals_new = obj.calc_residuals_lat(z, y_pred);
            R_hat_new = obj.est_noise_covariance(residuals_new, N);
            cost_new = obj.calc_cost_lat(R_hat_new, residuals_new);
        end
        
        function perturbed_params_matrix = create_perturbed_params_matrix(~, delta_theta_j, params, j, sign)
            perturbed_params_matrix = params;
            perturbed_params_matrix(j) = perturbed_params_matrix(j) + sign * delta_theta_j;
        end
        
        function y_pred = sim_model(obj, data_sequence, y_0, tspan, lat_params, lon_params)
            obj.DynamicsModel = obj.DynamicsModel.set_params(lat_params, lon_params);
            [t_data_seq, input_sequence, lon_state_seq] = data_sequence{:};
            [t_pred, y_pred] = ode45(@(t,y) obj.DynamicsModel.dynamics(t, y, t_data_seq, input_sequence, lon_state_seq), tspan, y_0);
            y_pred = interp1(t_pred, y_pred, t_data_seq); % change y_pred to correct time before returning
        end
        
        function v = calc_residuals_lat(~, z, y_pred)
            v = z - y_pred; % residuals
        end

        function cost = calc_cost_lat(~, R_hat, residuals)
            cost = 0.5 * sum(diag(residuals / R_hat * residuals'));
        end        
                
        function reached_convergence = check_for_convergence(obj, params_new, cost_new, R_hat_new)
            % TODO: Not finished. See Klein p 197 for details
            abs_param_change = abs(obj.OptData.params - params_new);
            abs_fval_change = abs((cost_new - obj.OptData.cost) / obj.OptData.cost);
            abs_covar_change = abs(diag(obj.OptData.R_hat - R_hat_new)./(diag(obj.OptData.R_hat)));

            if all(abs_param_change < 1e-5)
                reached_convergence = true;
            end
            if abs_fval_change < 0.001
                reached_convergence = true;
            end
            if all(abs_covar_change < 0.05)
                reached_convergence = true;
            end 
        end
        
        function R_hat = est_noise_covariance(~, residuals, N)
            R_hat = diag(diag(residuals' * residuals) / N); % Assumes cross-covariances to be zero ny neglecting nondiagonal terms
        end
   end
end