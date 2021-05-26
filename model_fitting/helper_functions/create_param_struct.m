function [parameters] = create_param_struct(type)
    aircraft_properties;
    initial_guess_lon;
    approx_zero = eps;
    
    if type == "lon"
        ParName = {
            'g',                ...
            'half_rho_planform', ...
            'mass',					...
            'mean_chord_length',              ...
            'wingspan',					...
            'nondim_constant_lon', ...
            'lam',				...
            'J_yy' ,            ...
            'servo_time_constant',...
            'servo_rate_lim_rad_s',...
            'elevator_trim_rad',...
            'c_L_0',				...
            'c_L_alpha',      	...
            'c_L_q',          	...
            'c_L_delta_e',    	...
            'c_D_p',				...
            'c_D_alpha',				...
            'c_D_alpha_sq',				...
            'c_D_q',          	...
            'c_D_delta_e',    	...
            'c_m_0',				...
            'c_m_alpha',          ...
            'c_m_q',				...
            'c_m_delta_e',		...
        };

        ParFixed = {
            true,... % g,                  ...
            true,... % half_rho_planform, ...
            true,... % mass_kg,					...
            true,... % mean_chord_length,              ...
            true,... % wingspan,					...
            true,... %nondim_constant_lon
            true,... % lam,				...
            true,... % Jyy, ...
            true,... % servo_time_const,...
            true,... % servo_rate_lim_rad_s,...
            true,... % elevator_trim_rad
            false,... % c_L_0,				...
            false,... % c_L_alpha,      	...
            false,... % c_L_q,          	...
            false,... % c_L_delta_e,    	...
            false,... % c_D_p,				...
            false,... % c_D_alpha,          ...
            false,... % c_D_alpha_sq,          ...
            false,... % c_D_q,          	...
            false,... % c_D_delta_e,    	...
            false,... % c_m_0,				...
            false,... % c_m_alpha,          ...
            false,... % c_m_q,				...
            false,... % c_m_delta_e,		...
        };

        ParMin = {
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            approx_zero,... % servo_time_const
            approx_zero,... % servo_time_const
            -Inf,... % elevator_trim_rad
            approx_zero,... % c_L_0,				...
            approx_zero,... % c_L_alpha,      	...
            -Inf,... % c_L_q,          	...
            approx_zero,... % c_L_delta_e,    	...
            approx_zero, ... % c_D_p,				...
            0, ...% c_D_alpha,          ...
            approx_zero, ...% c_D_alpha_sq,          ...
            approx_zero,... % c_D_q,          	...
            approx_zero,... % c_D_delta_e,    	...
            approx_zero,... % c_m_0,				...
            -Inf,... % c_m_alpha,          ...
            -Inf,... % c_m_q,				...
            -Inf,... % c_m_delta_e,		...
        };

        ParMax = {
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,... % servo_time_const
            Inf,... % servo_rate_lim_rad_s
            Inf,... % elevator_trim_rad
            Inf,... % c_L_0,				...
            Inf,... % c_L_alpha,      	...
            Inf,... % c_L_q,          	...
            Inf,... % c_L_delta_e,    	...
            Inf, ... % c_D_p,				...
            Inf, ...% c_D_alpha,          ...
            Inf, ...% c_D_alpha_sq,          ...
            Inf,... % c_D_q,          	...
            Inf,... % c_D_delta_e,    	...
            Inf,... % c_m_0,				...
            -approx_zero,... % c_m_alpha,          ...
            -approx_zero,... % c_m_q,				...
            -approx_zero,... % c_m_delta_e,		...
        };

        ParValue = {
            g,                  ...
            half_rho_planform, ...
            mass_kg,					...
            mean_aerodynamic_chord_m,              ...
            wingspan_m,					...
            nondim_constant_lon, ...
            lam,				...
            Jyy, ...
            servo_time_const, ...
            servo_rate_lim_rad_s,...
            elevator_trim_rad,...
            c_L_0,				...
            c_L_alpha,      	...
            c_L_q,          	...
            c_L_delta_e,    	...
            c_D_p,				...
            c_D_alpha,          ...
            c_D_alpha_sq,          ...
            c_D_q,          	...
            c_D_delta_e,    	...
            c_m_0,				...
            c_m_alpha,          ...
            c_m_q,				...
            c_m_delta_e,		...
        };
    elseif type == "lat"
        ParName = {
            'g',                ...
            'half_rho_planform', ...
            'mass',					...
            'mean_chord_length',              ...
            'wingspan',					...
            'nondim_constant_lon', ...
            'lam',				...
            'J_yy' ,            ...
            'servo_time_constant',...
            'servo_rate_lim_rad_s',...
            'elevator_trim_rad',...
            'c_L_0',				...
            'c_L_alpha',      	...
            'c_L_q',          	...
            'c_L_delta_e',    	...
            'c_D_p',				...
            'c_D_alpha',				...
            'c_D_alpha_sq',				...
            'c_D_q',          	...
            'c_D_delta_e',    	...
            'c_m_0',				...
            'c_m_alpha',          ...
            'c_m_q',				...
            'c_m_delta_e',		...
        };

        ParFixed = {
            true,... % g,                  ...
            true,... % half_rho_planform, ...
            true,... % mass_kg,					...
            true,... % mean_chord_length,              ...
            true,... % wingspan,					...
            true,... %nondim_constant_lon
            true,... % lam,				...
            true,... % Jyy, ...
            true,... % servo_time_const,...
            true,... % servo_rate_lim_rad_s,...
            true,... % elevator_trim_rad
            false,... % c_L_0,				...
            false,... % c_L_alpha,      	...
            false,... % c_L_q,          	...
            false,... % c_L_delta_e,    	...
            false,... % c_D_p,				...
            false,... % c_D_alpha,          ...
            false,... % c_D_alpha_sq,          ...
            false,... % c_D_q,          	...
            false,... % c_D_delta_e,    	...
            false,... % c_m_0,				...
            false,... % c_m_alpha,          ...
            false,... % c_m_q,				...
            false,... % c_m_delta_e,		...
        };

        ParMin = {
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            -Inf,...
            approx_zero,... % servo_time_const
            approx_zero,... % servo_time_const
            -Inf,... % elevator_trim_rad
            approx_zero,... % c_L_0,				...
            approx_zero,... % c_L_alpha,      	...
            -Inf,... % c_L_q,          	...
            approx_zero,... % c_L_delta_e,    	...
            approx_zero, ... % c_D_p,				...
            0, ...% c_D_alpha,          ...
            approx_zero, ...% c_D_alpha_sq,          ...
            approx_zero,... % c_D_q,          	...
            approx_zero,... % c_D_delta_e,    	...
            approx_zero,... % c_m_0,				...
            -Inf,... % c_m_alpha,          ...
            -Inf,... % c_m_q,				...
            -Inf,... % c_m_delta_e,		...
        };

        ParMax = {
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,...
            Inf,... % servo_time_const
            Inf,... % servo_rate_lim_rad_s
            Inf,... % elevator_trim_rad
            Inf,... % c_L_0,				...
            Inf,... % c_L_alpha,      	...
            Inf,... % c_L_q,          	...
            Inf,... % c_L_delta_e,    	...
            Inf, ... % c_D_p,				...
            Inf, ...% c_D_alpha,          ...
            Inf, ...% c_D_alpha_sq,          ...
            Inf,... % c_D_q,          	...
            Inf,... % c_D_delta_e,    	...
            Inf,... % c_m_0,				...
            -approx_zero,... % c_m_alpha,          ...
            -approx_zero,... % c_m_q,				...
            -approx_zero,... % c_m_delta_e,		...
        };

        ParValue = {
            g,                  ...
            half_rho_planform, ...
            mass_kg,					...
            mean_aerodynamic_chord_m,              ...
            wingspan_m,					...
            nondim_constant_lon, ...
            lam,				...
            Jyy, ...
            servo_time_const, ...
            servo_rate_lim_rad_s,...
            elevator_trim_rad,...
            c_L_0,				...
            c_L_alpha,      	...
            c_L_q,          	...
            c_L_delta_e,    	...
            c_D_p,				...
            c_D_alpha,          ...
            c_D_alpha_sq,          ...
            c_D_q,          	...
            c_D_delta_e,    	...
            c_m_0,				...
            c_m_alpha,          ...
            c_m_q,				...
            c_m_delta_e,		...
        };
    end

    parameters = struct('Name', ParName, ...
        'Unit', '',...
        'Value', ParValue, ...
        'Minimum', ParMin, ...
        'Maximum', ParMax, ...
        'Fixed', ParFixed);
end