function [c_X, c_Y, c_Z, c_L, c_D] = calc_force_coeffs(u, v, w, a_x, a_y, a_z, n_p)
    dyn_pressure = calc_dyn_pressure(u, v, w);
    aircraft_properties; % get mass and planform
    
    % Calculate body forces
    T = calc_thrust_pusher(n_p);
    X = mass_kg * a_x - T;
    Y = (mass_kg * a_y);
    Z = (mass_kg * a_z);
    
    c_X = X ./ (dyn_pressure * planform_sqm);
    c_Y = Y ./ (dyn_pressure * planform_sqm);
    c_Z = Z ./ (dyn_pressure * planform_sqm);
    
    [c_L, c_D] = calc_lift_drag_coeff(u, w, c_X, c_Z);
end

function T = calc_thrust_pusher(n_p)
    aircraft_properties; % Load rho, diam and thrust coeff
    T = rho * prop_diam_pusher^4 * c_T_pusher * n_p.^2;
end

function [c_L, c_D] = calc_lift_drag_coeff(u, w, c_X, c_Z)
    % Calculate lift and drag forces
    aoa = atan2(w, u);
    c = zeros(2, length(c_X));
    for i = 1:length(aoa)
        R = [cos(-aoa(i)) -sin(-aoa(i));
             sin(-aoa(i)) cos(-aoa(i))];
        c(:,i) = R * [c_X(i); c_Z(i)];
    end
    c_D = -c(1,:)';
    c_L = -c(2,:)';
end