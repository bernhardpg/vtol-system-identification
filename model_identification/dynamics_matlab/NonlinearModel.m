classdef NonlinearModel
    properties
        Params = {}
        CoeffsLat = {}
        CoeffsLon = {}
    end
    methods
        function obj = NonlinearModel(coeffs_lon, coeffs_lat)
            airframe_static_properties;
            obj.Params.rho = rho;
            obj.Params.g = g;
            obj.Params.mass_kg = mass_kg;
            obj.Params.wingspan_m = wingspan_m;
            obj.Params.mean_aerodynamic_chord_m = mean_aerodynamic_chord_m;
            obj.Params.planform_sqm = planform_sqm;
            obj.Params.V_nom = V_nom;
            obj.Params.gam_1 = gam(1);
            obj.Params.gam_2 = gam(2);
            obj.Params.gam_3 = gam(3);
            obj.Params.gam_4 = gam(4);
            obj.Params.gam_5 = gam(5);
            obj.Params.gam_6 = gam(6);
            obj.Params.gam_7 = gam(7);
            obj.Params.gam_8 = gam(8);
            obj.Params.J_yy = J_yy;
            
            obj.CoeffsLat = coeffs_lat;
            obj.CoeffsLon = coeffs_lon;
        end
            
        % y = [v p r phi]
        % u = [delta_a delta_r]
        % lon_states = [u w q theta]
        %   lon states are taken as measured, and not simulated
        function dy_dt = dynamics(obj, t, y, t_data_seq, input_seq, lon_state_seq)
            % Extract states
            y = num2cell(y);
            [v, p, r, phi] = y{:};
           
            % Get input at t
            input_at_t = interp1(t_data_seq, input_seq, t);
            input_at_t = num2cell(input_at_t);
            [delta_a, delta_r] = input_at_t{:};
            
            % TODO: temp for lat model
            delta_e = 0;
            
            % Get lon states at t
            lon_state_at_t = interp1(t_data_seq, lon_state_seq, t);
            lon_state_at_t = num2cell(lon_state_at_t);
            [u, w, q, theta] = lon_state_at_t{:};
            
            % Calculate forces and moments
            [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = obj.nondimensionalize_states(p, q, r, u, v, w);
            
            exp_vars = {};
            exp_vars.p_hat = p_hat;
            exp_vars.q_hat = q_hat;
            exp_vars.r_hat = r_hat;
            exp_vars.u_hat = u_hat;
            exp_vars.v_hat = v_hat;
            exp_vars.w_hat = w_hat;
            exp_vars.delta_a = delta_a;
            exp_vars.delta_e = delta_e;
            exp_vars.delta_r = delta_r;
            
            [c_X, c_Y, c_Z, c_l, c_m, c_n] = obj.calc_coeffs(exp_vars);
            dyn_pressure = obj.calc_dyn_pressure(u, v, w);
            [X, Y, Z] = calc_forces(obj, c_X, c_Y, c_Z, dyn_pressure);
            [l, m, n] = calc_moments(obj, c_l, c_m, c_n, dyn_pressure);
            
            % Dynamics
            [phi_dot, theta_dot, psi_dot] = obj.eul_dynamics(phi, theta, p, q, r);
            [p_dot, q_dot, r_dot] = obj.ang_vel_dynamics(p, q, r, l, m, n);
            T = 0; % TODO: for now
            [u_dot, v_dot, w_dot] = obj.vel_body_dynamics(phi, theta, p, q, r, u, v, w, X, Y, Z, T);
   
            dy_dt = [v_dot p_dot r_dot phi_dot]';
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
            u_dot = r * v - q * w + (1/obj.Params.mass_kg) * (X - T - obj.Params.mass_kg * obj.Params.g * sin(theta));
            v_dot = p * w - r * u + (1/obj.Params.mass_kg) * (Y + obj.Params.mass_kg * obj.Params.g * cos(theta) * sin(phi));
            w_dot = q * u - p * v + (1/obj.Params.mass_kg) * (Z + obj.Params.mass_kg * obj.Params.g * cos(theta) * cos(phi)); 
        end
        
        function [c_X, c_Y, c_Z, c_l, c_m, c_n] = calc_coeffs(obj, exp_vars)
            % For now, set all lon coeffs to zero
            c_X = 0;
            c_Z = 0;
            c_m = 0;
            
            % Lat coeffs
            c_Y = 0;
            coeff_names = string(fieldnames(obj.CoeffsLat.c_Y));
            for i = 1:numel(coeff_names)
                coeff_name = coeff_names(i);
                if strcmp(coeff_name, "bias")
                    c_Y = c_Y + obj.CoeffsLat.c_Y.bias * 1; 
                else
                    c_Y = c_Y + obj.CoeffsLat.c_Y.(coeff_name) * exp_vars.(coeff_name);
                end
            end
            
            c_l = 0;
            coeff_names = string(fieldnames(obj.CoeffsLat.c_l));
            for i = 1:numel(coeff_names)
                coeff_name = coeff_names(i);
                if strcmp(coeff_name, "bias")
                    c_l = c_l + obj.CoeffsLat.c_l.bias * 1; 
                else
                    c_l = c_l + obj.CoeffsLat.c_l.(coeff_name) * exp_vars.(coeff_name);
                end
            end
            
            c_n = 0;
            coeff_names = string(fieldnames(obj.CoeffsLat.c_n));
            for i = 1:numel(coeff_names)
                coeff_name = coeff_names(i);
                if strcmp(coeff_name, "bias")
                    c_n = c_n + obj.CoeffsLat.c_n.bias * 1; 
                else
                    c_n = c_n + obj.CoeffsLat.c_n.(coeff_name) * exp_vars.(coeff_name);
                end
            end
            
        end
        
        function [X, Y, Z] = calc_forces(obj, c_X, c_Y, c_Z, dyn_pressure)
            X = c_X * dyn_pressure * obj.Params.planform_sqm;
            Y = c_Y * dyn_pressure * obj.Params.planform_sqm;
            Z = c_Z * dyn_pressure * obj.Params.planform_sqm;
        end
        
        function [l, m, n] = calc_moments(obj, c_l, c_m, c_n, dyn_pressure)
            l = c_l * dyn_pressure * obj.Params.planform_sqm * obj.Params.wingspan_m;
            m = c_m * dyn_pressure * obj.Params.planform_sqm * obj.Params.mean_aerodynamic_chord_m;
            n = c_n * dyn_pressure * obj.Params.planform_sqm * obj.Params.wingspan_m;
        end
        
        function [dyn_pressure] = calc_dyn_pressure(obj, u, v, w)
            V = sqrt(u .^ 2 + v .^ 2 + w .^ 2);
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