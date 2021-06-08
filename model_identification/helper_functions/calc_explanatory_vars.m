function [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w)
    aircraft_properties; % Get V_nom, wingspan and MAC
    
    u_hat = u / V_nom;
    v_hat = v / V_nom;
    w_hat = w / V_nom;
    p_hat = p * (wingspan_m / (2 * V_nom));
    q_hat = q * (mean_aerodynamic_chord_m / (2 * V_nom));
    r_hat = r * (wingspan_m / (2 * V_nom));
end
