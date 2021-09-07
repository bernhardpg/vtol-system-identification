classdef OutputErrorProblem
    properties
        DynamicsModel
        FprData
        ManeuverTypes
        OptData = {}
    end
    methods
        function obj = OutputErrorProblem(fpr_data, dynamics_model, maneuver_types)
            obj.FprData = fpr_data;
            obj.DynamicsModel = dynamics_model;
            obj.ManeuverTypes = maneuver_types;
            obj.OptData.iteration_number = 0;
            obj.OptData.reached_convergence = false;
        end
        
        function obj = solve(obj)
            for maneuver_type = obj.ManeuverTypes(1)
                for maneuver_i = 1
                    maneuver = obj.FprData.training.(maneuver_type)(maneuver_i);
                                            
                    % Initial evaluation
                    [obj.OptData.R_hat, obj.OptData.cost, obj.OptData.theta] = evaluate_iteration_for_maneuver(obj, maneuver);
                    
                    % Run until convergence
                    while ~obj.OptData.reached_convergence
                        obj.OptData.iteration_number = obj.OptData.iteration_number + 1;
                        disp("Running iteration: " + obj.OptData.iteration_number);
                        
                        [R_hat_new, cost_new, theta_new] = evaluate_iteration_for_maneuver(obj, maneuver);
                        obj.OptData.reached_convergence = check_for_convergence(obj, theta_new, cost_new, R_hat_new);
                    end
                    disp("Reached convergence after " + obj.OptData.iteration_number + " iterations");
                end
            end
        end
        
        function [R_hat_new, cost_new, theta_new] = evaluate_iteration_for_maneuver(obj, maneuver)
            t_data_seq = maneuver.Time;
            tspan = [maneuver.Time(1) maneuver.Time(end)];
            N = length(t_data_seq);
            z = maneuver.get_lat_state_sequence();
            y_0 = maneuver.get_lat_state_initial();
            lon_state_seq = maneuver.get_lon_state_sequence();
            input_sequence = maneuver.get_lat_input_sequence();
            
            % Calculate 
            residuals = obj.calc_residuals_lat(t_data_seq, input_sequence, lon_state_seq, z, y_0, tspan);
            R_hat_new = obj.est_noise_covariance(residuals, N);
            cost_new = obj.calc_cost_lat(R_hat_new, residuals);
            theta_new = 0;
        end
        
        function v = calc_residuals_lat(obj, t_data_seq, input_sequence, lon_state_seq, z, y_0, tspan)
            [t_pred, y_pred] = ode45(@(t,y) obj.DynamicsModel.dynamics(t, y, t_data_seq, input_sequence, lon_state_seq), tspan, y_0);
            y_pred = interp1(t_pred, y_pred, t_data_seq); % change y_pred to correct time
            v = z - y_pred; % residuals
        end

        function cost = calc_cost_lat(~, R_hat, residuals)
            cost = 0.5 * sum(diag(residuals / R_hat * residuals'));
        end        
                
        function reached_convergence = check_for_convergence(obj, theta_new, cost_new, R_hat_new)
            % TODO: Not finished
            abs_param_change = abs(obj.OptData.theta - theta_new);
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