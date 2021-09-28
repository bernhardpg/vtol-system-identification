classdef NonlinearModel
    properties
        StaticParams
        Params = {}
        CoeffsLat;
        CoeffsLon;
    end
    methods
        function obj = NonlinearModel(coeffs_lon, coeffs_lat)
            airframe_static_properties;
            obj.Params.rho = rho;
            obj.Params.mass_kg = mass_kg;
            obj.Params.g = g;
            obj.Params.wingspan_m = wingspan_m;
            obj.Params.mean_aerodynamic_chord_m = mean_aerodynamic_chord_m;
            obj.Params.planform_sqm = planform_sqm;
            obj.Params.V_nom = V_nom;
            obj.Params.alpha_nom = alpha_nom;
            obj.Params.delta_e_nom = delta_e_nom;
            obj.Params.delta_a_nom = delta_a_nom;
            obj.Params.delta_r_nom = delta_r_nom;
            obj.Params.gam_1 = gam(1);
            obj.Params.gam_2 = gam(2);
            obj.Params.gam_3 = gam(3);
            obj.Params.gam_4 = gam(4);
            obj.Params.gam_5 = gam(5);
            obj.Params.gam_6 = gam(6);
            obj.Params.gam_7 = gam(7);
            obj.Params.gam_8 = gam(8);
            obj.Params.J_yy = J_yy;
            obj.Params.prop_diam_pusher = prop_diam_pusher;
            obj.Params.c_T_pusher = c_T_pusher;
            
            obj.Params.u_nom = V_nom / (sqrt(1 + tan(alpha_nom)^2));
            obj.Params.w_nom = sqrt(V_nom^2 - obj.Params.u_nom^2);
            
            obj.StaticParams = [rho mass_kg g wingspan_m ...
                mean_aerodynamic_chord_m planform_sqm ...
                V_nom alpha_nom delta_e_nom ...
                gam J_yy ...
                prop_diam_pusher c_T_pusher]';
            
            obj.CoeffsLat = coeffs_lat;
            obj.CoeffsLon = coeffs_lon;
        end
             
        function dy_dt = dynamics_lat_model_c(obj, t, y, t_data_seq, input_seq, lon_state_seq)
            % y = [v p r phi]
            % u = [delta_a delta_r]
            % lon_states = [u w q theta]
            %   lon states are taken as measured, and not simulated
            input_seq_for_c_file = [t_data_seq input_seq lon_state_seq];
            params = [obj.StaticParams;
                      reshape(obj.CoeffsLat, [numel(obj.CoeffsLat),1])];
            dy_dt = dynamics_lat_c(t, y, input_seq_for_c_file, params);
        end
        
        function dy_dt = dynamics_lon_model_c(obj, t, y, t_data_seq, lon_input_seq, lat_state_seq)
            lon_input_seq(:,1) = detrend(lon_input_seq(:,1),0);
            input_seq_for_c_file = [t_data_seq lon_input_seq lat_state_seq];
            params_vector = [obj.StaticParams;
                      reshape(obj.CoeffsLon, [numel(obj.CoeffsLon),1])];
            dy_dt = dynamics_lon_c(t, y, input_seq_for_c_file, params_vector);
        end
        
        function dy_dt = dynamics_lon_model(obj, t, y, t_data_seq, input_seq, lat_state_seq)
            % Extract states
            y = num2cell(y);
            [u, w, q, theta] = y{:};
           
            % Roll index forward until we get to approx where we should get
            % inputs from. This basically implements zeroth-order hold for
            % the input
            curr_index_data_seq = 1;
            while t_data_seq(curr_index_data_seq) < t
               curr_index_data_seq = curr_index_data_seq + 1;
            end
            
            % Get input at t
            input_at_t = input_seq(curr_index_data_seq,:);
            input_at_t = num2cell(input_at_t);
            [delta_e, delta_t] = input_at_t{:};
            
            % Assume lat inputs to be 0. These are not actually used
            % anywhere
            delta_a = 0;
            delta_r = 0;
            
            % Get lon states at t
            lat_state_at_t = lat_state_seq(curr_index_data_seq,:);
            lat_state_at_t = num2cell(lat_state_at_t);
            [v, p, r, phi] = lat_state_at_t{:};
            
            % Calc aerodynamic angles and airspeed
            [alpha, beta, V] = obj.calc_aerodynamic_quantities(u, v, w);
            
            % Calculate forces and moments
            [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = obj.nondimensionalize_states(p, q, r, u, v, w);
            
            explanatory_vars_lat = [1 beta p_hat r_hat delta_a delta_r];
            explanatory_vars_lon = [1 alpha alpha.^2 q_hat delta_e alpha*delta_e];
            
            [c_D, c_Y, c_L, c_l, c_m, c_n] = obj.calc_coeffs(explanatory_vars_lat, explanatory_vars_lon);
            dyn_pressure = obj.calc_dyn_pressure(u, v, w);
            [X, Y, Z] = calc_forces(obj, c_D, c_Y, c_L, dyn_pressure, alpha);
            T = calc_thrust_pusher(obj, delta_t);
            [l, m, n] = calc_moments(obj, c_l, c_m, c_n, dyn_pressure);
            
            % Dynamics
            [~, theta_dot, ~] = obj.eul_dynamics(phi, theta, p, q, r);
            [~, q_dot, ~] = obj.ang_vel_dynamics(p, q, r, l, m, n);
            [u_dot, ~, w_dot] = obj.vel_body_dynamics(phi, theta, p, q, r, u, v, w, X, Y, Z, T);
   
            dy_dt = [u_dot w_dot q_dot theta_dot]';
        end
        
        function T = calc_thrust_pusher(obj, delta_t)
            T = obj.Params.rho * obj.Params.prop_diam_pusher^4 * obj.Params.c_T_pusher * delta_t.^2;
        end
        
        function dy_dt = dynamics_lat_model(obj, t, y, t_data_seq, input_seq, lon_state_seq)
            % Extract states
            y = num2cell(y);
            [v, p, r, phi] = y{:};
           
            % Roll index forward until we get to approx where we should get
            % inputs from. This basically implements zeroth-order hold for
            % the input
            curr_index_data_seq = 1;
            while t_data_seq(curr_index_data_seq) < t
               curr_index_data_seq = curr_index_data_seq + 1;
            end
            
            % Get input at t
            input_at_t = input_seq(curr_index_data_seq,:);
            input_at_t = num2cell(input_at_t);
            [delta_a, delta_r] = input_at_t{:};
            
            % Do not use longitudinal inputs for lateral model
            delta_e = 0;
            T = 0;
            
            % Get lon states at t
            lon_state_at_t = lon_state_seq(curr_index_data_seq,:);
            lon_state_at_t = num2cell(lon_state_at_t);
            [u, w, q, theta] = lon_state_at_t{:};
            
            % Calc aerodynamic angles and airspeed
            [alpha, beta, V] = obj.calc_aerodynamic_quantities(u, v, w);
            
            % Calculate forces and moments
            [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = obj.nondimensionalize_states(p, q, r, u, v, w);
            
            explanatory_vars_lat = [1 beta p_hat r_hat delta_a delta_r];
            explanatory_vars_lon = [1 alpha alpha.^2 q_hat delta_e];
            
            [c_X, c_Y, c_Z, c_l, c_m, c_n] = obj.calc_coeffs(explanatory_vars_lat, explanatory_vars_lon);
            dyn_pressure = obj.calc_dyn_pressure(u, v, w);
            [X, Y, Z] = calc_forces(obj, c_X, c_Y, c_Z, dyn_pressure, alpha);
            [l, m, n] = calc_moments(obj, c_l, c_m, c_n, dyn_pressure);
            
            % Dynamics
            [phi_dot, ~, psi_dot] = obj.eul_dynamics(phi, theta, p, q, r);
            [p_dot, ~, r_dot] = obj.ang_vel_dynamics(p, q, r, l, m, n);
            [~, v_dot, ~] = obj.vel_body_dynamics(phi, theta, p, q, r, u, v, w, X, Y, Z, T);
   
            dy_dt = [v_dot p_dot r_dot phi_dot]';
        end
        
        function [alpha, beta, airspeed] = calc_aerodynamic_quantities(obj, u, v, w)
            airspeed = sqrt(u^2 + v^2 + w^2);
            alpha = atan2(w,u) - obj.Params.alpha_nom;
            beta = asin(v/airspeed); 
        end
        
        function obj = set_params(obj, lat_params, lon_params)
           obj.CoeffsLat = lat_params;
           obj.CoeffsLon = lon_params;
        end

        function [phi_dot, theta_dot, psi_dot] = eul_dynamics(~, phi, theta, p, q, r)
            phi_dot = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
            theta_dot = q * cos(phi) - r * sin(phi);
            psi_dot = (q * sin(phi) + r * cos(phi)) * sec(theta);
        end
        
        function [p_dot, q_dot, r_dot] = ang_vel_dynamics(obj, p, q, r, l, m, n)
            p_dot = obj.Params.gam_1 * p * q - obj.Params.gam_2 * q * r + obj.Params.gam_3 * l + obj.Params.gam_4 * n;
            q_dot = obj.Params.gam_5 * p * r - obj.Params.gam_6 * (p^2 - r^2) + (1/obj.Params.J_yy) * m;
            r_dot = obj.Params.gam_7 * p * q - obj.Params.gam_1 * q * r + obj.Params.gam_4 * l + obj.Params.gam_8 * n; 
        end
        
        function [u_dot, v_dot, w_dot] = vel_body_dynamics(obj, phi, theta, p, q, r, u, v, w, X, Y, Z, T)
            u_dot = r * v - q * w + (1/obj.Params.mass_kg) * (X + T - obj.Params.mass_kg * obj.Params.g * sin(theta));
            v_dot = p * w - r * u + (1/obj.Params.mass_kg) * (Y + obj.Params.mass_kg * obj.Params.g * cos(theta) * sin(phi));
            w_dot = q * u - p * v + (1/obj.Params.mass_kg) * (Z + obj.Params.mass_kg * obj.Params.g * cos(theta) * cos(phi)); 
        end
        
        function [c_D, c_Y, c_L, c_l, c_m, c_n] = calc_coeffs(obj, explanatory_vars_lat, explanatory_vars_lon)            
            % Lat coeffs
            temp = explanatory_vars_lat * obj.CoeffsLat;
            c_Y = temp(1);
            c_l = temp(2);
            c_n = temp(3);
            
            temp = explanatory_vars_lon * obj.CoeffsLon;
            c_D = temp(1);
            c_L = temp(2);
            c_m = temp(3);
        end
        
        function [X, Y, Z] = calc_forces(obj, c_D, c_Y, c_L, dyn_pressure, alpha)
            D = c_D * dyn_pressure * obj.Params.planform_sqm;
            Y = c_Y * dyn_pressure * obj.Params.planform_sqm;
            L = c_L * dyn_pressure * obj.Params.planform_sqm;
            
            X = -cos(alpha) * D + sin(alpha) * L;
            Z = -sin(alpha) * D - cos(alpha) * L;
        end
        
        function [l, m, n] = calc_moments(obj, c_l, c_m, c_n, dyn_pressure)
            l = c_l * dyn_pressure * obj.Params.planform_sqm * obj.Params.wingspan_m;
            m = c_m * dyn_pressure * obj.Params.planform_sqm * obj.Params.mean_aerodynamic_chord_m;
            n = c_n * dyn_pressure * obj.Params.planform_sqm * obj.Params.wingspan_m;
        end
        
        function [dyn_pressure] = calc_dyn_pressure(obj, u, v, w)
            V = sqrt(u.^ 2 + v .^ 2 + w .^ 2);
            dyn_pressure = 0.5 * obj.Params.rho * V .^ 2;
        end

        function [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = nondimensionalize_states(obj, p, q, r, u, v, w)
            u_hat = u / obj.Params.V_nom;
            v_hat = v / obj.Params.V_nom;
            w_hat = w / obj.Params.V_nom;
            p_hat = p * (obj.Params.wingspan_m / (2 * obj.Params.V_nom));
            q_hat = q * (obj.Params.mean_aerodynamic_chord_m / (2 * obj.Params.V_nom));
            r_hat = r * (obj.Params.wingspan_m / (2 * obj.Params.V_nom));
        end

    end
end