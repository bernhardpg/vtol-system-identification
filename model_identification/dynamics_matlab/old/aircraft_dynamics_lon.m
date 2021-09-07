function [dy_dt] = aircraft_dynamics_lon(t, y, t_seq, input_seq, lat_state_seq, params)
    % Get parameters
     params = num2cell(params);
    [rho, mass_kg, g, wingspan_m, mean_aerodynamic_chord_m, planform_sqm, V_nom,...
     gam_1, gam_2, gam_3, gam_4, gam_5, gam_6, gam_7, gam_8, J_yy,...
     c_X_0, c_X_u, c_X_w, c_X_w_sq, c_X_q, c_X_n_p,...
     c_Z_0, c_Z_w, c_Z_w_sq, c_Z_delta_e,...
     c_m_0, c_m_w, c_m_q, c_m_delta_e,...
     ] = params{:};

    % Get state variables
    y = num2cell(y);
    [theta, q, u, w] = y{:};
    
    % Get inputs
    input = interp1(t_seq, input_seq, t); % Get value of input now
    input = num2cell(input);
    [delta_a, delta_e, delta_r, n_p] = input{:};
    
    % Get lat states
    lat_state = interp1(t_seq, lat_state_seq, t);
    lat_state = num2cell(lat_state);
    [phi, psi, p, r, v] = lat_state{:};
    
    % Calculate non-dimensionalized variables
    [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w, V_nom, wingspan_m, mean_aerodynamic_chord_m);
    
    % Dynamics
    theta_dot = q * cos(phi) - r * sin(phi);

    c_X = c_X_0 + c_X_u * u_hat + c_X_w * w_hat + c_X_w_sq * w_hat^2 + c_X_q * q_hat + c_X_n_p * n_p;
    c_Z = c_Z_0 + c_Z_w * w_hat + c_Z_w_sq * q_hat + c_Z_delta_e * delta_e;
    c_m = c_m_0 + c_m_w * w_hat + c_m_q * q_hat + c_m_delta_e * delta_e;
    
    dyn_pressure = calc_dyn_pressure(u, v, w, rho);
    
    X = c_X * dyn_pressure * planform_sqm;
    Z = c_Z * dyn_pressure * planform_sqm;
    M = c_m * dyn_pressure * planform_sqm * mean_aerodynamic_chord_m;
    
    T = 0;
   
    q_dot = gam_5 * p * r - gam_6 * (p^2 - r^2) + (1/J_yy) * M;
    
    u_dot = r * v - q * w + (1/mass_kg) * (X - T - mass_kg * g * sin(theta));
    w_dot = q * u - p * v + (1/mass_kg) * (Z + mass_kg * g * cos(theta) * cos(phi));
    
    dy_dt = [theta_dot q_dot u_dot w_dot]';
end

function [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w, V_nom, wingspan_m, mean_aerodynamic_chord_m)
    u_hat = u / V_nom;
    v_hat = v / V_nom;
    w_hat = w / V_nom;
    p_hat = p * (wingspan_m / (2 * V_nom));
    q_hat = q * (mean_aerodynamic_chord_m / (2 * V_nom));
    r_hat = r * (wingspan_m / (2 * V_nom));
end

function [dyn_pressure] = calc_dyn_pressure(u, v, w, rho)
    V = sqrt(u .^ 2 + v .^ 2 + w .^ 2);
    dyn_pressure = 0.5 * rho * V .^ 2;
end