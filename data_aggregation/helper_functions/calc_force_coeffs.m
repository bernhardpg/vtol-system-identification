function [c_X, c_Y, c_Z] = calc_force_coeffs(u, v, w, a_x, a_y, a_z, n_p)
    dyn_pressure = calc_dyn_pressure(u, v, w);
    aircraft_properties; % get mass and planform
    
    %T = 0;
    T = calc_thrust_pusher(n_p);
    c_X = (mass_kg * a_x - T) ./ (dyn_pressure * planform_sqm);
    c_Y = (mass_kg * a_y) ./ (dyn_pressure * planform_sqm);
    c_Z = (mass_kg * a_z) ./ (dyn_pressure * planform_sqm);
end

function T = calc_thrust_pusher(n_p)
    aircraft_properties; % Load rho, diam and thrust coeff
    T = rho * prop_diam_pusher^4 * c_T_pusher * n_p.^2;
end