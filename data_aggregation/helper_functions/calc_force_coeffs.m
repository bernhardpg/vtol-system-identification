function [c_X, c_Y, c_Z] = calc_force_coeffs(u, v, w, a_x, a_y, a_z, T)
    dyn_pressure = calc_dyn_pressure(u, v, w);
    aircraft_properties; % get mass and planform
    % TODO: Add thrust here
    T = 0;
    c_X = (mass_kg * a_x - T) ./ (dyn_pressure * planform_sqm);
    c_Y = (mass_kg * a_y) ./ (dyn_pressure * planform_sqm);
    c_Z = (mass_kg * a_z) ./ (dyn_pressure * planform_sqm);
end