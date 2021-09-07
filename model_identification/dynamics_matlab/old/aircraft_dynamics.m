function [dy_dt] = aircraft_dynamics(t, y, t_input, input_sequence, params)
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
    [phi, theta, psi, p, q, r, u, v, w] = y{:};
    
    % Get inputs
    input = interp1(t_input, input_sequence, t); % Get value of input now
    input = num2cell(input);
    [delta_a, delta_e, delta_r, n_p] = input{:};
    
    % Calculate non-dimensionalized variables
    [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w, V_nom, wingspan_m, mean_aerodynamic_chord_m);
    
    % Dynamics
    phi_dot = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot = (q * sin(phi) + r * cos(phi)) * sec(theta);

    c_X = c_X_0 + c_X_u * u_hat + c_X_w * w_hat + c_X_w_sq * w_hat^2 + c_X_q * q_hat + c_X_n_p * n_p;
    c_Z = c_Z_0 + c_Z_w * w_hat + c_Z_w_sq * q_hat + c_Z_delta_e * delta_e;
    c_m = c_m_0 + c_m_w * w_hat + c_m_q * q_hat + c_m_delta_e * delta_e;
    
    dyn_pressure = calc_dyn_pressure(u, v, w, rho);
    
    X = c_X * dyn_pressure * planform_sqm;
    Z = c_Z * dyn_pressure * planform_sqm;
    M = c_m * dyn_pressure * planform_sqm * mean_aerodynamic_chord_m;
   
    Y = 0;
    L = 0;
    N = 0;
    
    T = 0;
    
    p_dot = gam_1 * p * q - gam_2 * q * r + gam_3 * L + gam_4 * N;
    q_dot = gam_5 * p * r - gam_6 * (p^2 - r^2) + (1/J_yy) * M;
    r_dot = gam_7 * p * q - gam_1 * q * r + gam_4 * L + gam_8 * N;
    
    u_dot = r * v - q * w + (1/mass_kg) * (X - T - mass_kg * g * sin(theta));
    v_dot = p * w - r * u + (1/mass_kg) * (Y + mass_kg * g * cos(theta) * sin(phi));
    w_dot = q * u - p * v + (1/mass_kg) * (Z + mass_kg * g * cos(theta) * cos(phi));
    
    dy_dt = [phi_dot theta_dot psi_dot p_dot q_dot r_dot u_dot v_dot w_dot]';
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