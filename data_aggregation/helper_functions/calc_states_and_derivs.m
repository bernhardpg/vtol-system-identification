function [phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot] = ...
    calc_states_and_derivs(dt_desired, t_recorded, phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded)
    
    t_0 = t_recorded(1);
    t_end = t_recorded(end);
    
    % Calc u, v and w from kinematic relationships
    [u_recorded, v_recorded, w_recorded] = calc_uvw(phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded);
    
    % Smooth signals
    temp = smooth_signal([phi_recorded theta_recorded psi_recorded]);
    phi = temp(:,1);
    theta = temp(:,2);
    psi = temp(:,3);
    
    temp = smooth_signal([u_recorded v_recorded w_recorded]);
    u = temp(:,1);
    v = temp(:,2);
    w = temp(:,3);
    
    % Calculate piecewise spline approximations of signals
    phi_spline = slmengine(t_recorded, phi,'knots',t_0:.1:t_end + 0.1); % add 'plot', 'on' to see fit
    theta_spline = slmengine(t_recorded, theta,'knots',t_0:.1:t_end + 0.1);
    psi_spline = slmengine(t_recorded, psi,'knots',t_0:.1:t_end + 0.1);
        
    u_spline = slmengine(t_recorded, u,'knots',t_0:.1:t_end + 0.1, 'plot', 'off');
    v_spline = slmengine(t_recorded, v,'knots',t_0:.1:t_end + 0.1, 'plot', 'off');
    w_spline = slmengine(t_recorded, w,'knots',t_0:.1:t_end + 0.1, 'plot', 'off');
    
    % Set desired time vector
    t = (0:dt_desired:t_end)';
    
    % Calculate derivatives and attitude angles at desired times
    phi = slmeval(t, phi_spline);
    phi_dot = slmeval(t, phi_spline, 1);
    theta = slmeval(t, theta_spline);
    theta_dot = slmeval(t, theta_spline, 1);
    psi = slmeval(t, psi_spline);
    psi_dot = slmeval(t, psi_spline, 1);
    
    u = slmeval(t, u_spline);
    u_dot = slmeval(t, u_spline, 1);
    v = slmeval(t, v_spline);
    v_dot = slmeval(t, v_spline, 1);
    w = slmeval(t, w_spline);
    w_dot = slmeval(t, w_spline, 1);
    
    % Calc p, q and r from kinematic relationships
    [p, q, r] = calc_pqr(phi, theta, psi, phi_dot, theta_dot, psi_dot);
    
    % Calc translational accelerations
    aircraft_properties; % to get g
    [a_x, a_y, a_z] = calc_trans_acc(g, phi, theta, psi, p, q, r, u, v, w, u_dot, v_dot, w_dot);
    
    % Calc angular accelerations
    p_spline = slmengine(t, p,'knots',t_0:.1:t_end + 0.1, 'plot', 'off');
    q_spline = slmengine(t, q,'knots',t_0:.1:t_end + 0.1, 'plot', 'off');
    r_spline = slmengine(t, r,'knots',t_0:.1:t_end + 0.1, 'plot', 'off');
    
    p_dot = slmeval(t, p_spline, 1);
    q_dot = slmeval(t, q_spline, 1);
    r_dot = slmeval(t, r_spline, 1);
end

function [p, q, r] = calc_pqr(phi, theta, psi, phi_dot, theta_dot, psi_dot)
    p = phi_dot - psi_dot .* sin(theta);
    q = theta_dot .* cos(phi) + psi_dot .* sin(phi) .* cos(theta);
    r = psi_dot .* cos(phi) .* cos(theta) - theta_dot .* sin(phi);
end

function [u, v, w] = calc_uvw(phi, theta, psi, v_N, v_E, v_D)
    u = cos(theta) .* cos(psi) .* v_N + cos(theta) .* sin(psi) .* v_E - sin(theta) .* v_D;
    v = (cos(psi) .* sin(theta) .* sin(phi) - cos(phi) .* sin(psi)) .* v_N ...
        + (cos(phi) .* cos(psi) + sin(theta) .* sin(phi) .* sin(psi)) .* v_E ...
        + cos(theta) .* sin(phi) .* v_D;
    w = (cos(psi) .* sin(theta) .* cos(phi) + sin(phi) .* sin(psi)) .* v_N ...
        + (sin(theta) .* cos(phi) .* sin(psi) - sin(phi) .* cos(psi)) .* v_E ...
        + cos(theta) .* cos(phi) .* v_D;
end

function [a_x, a_y, a_z] = calc_trans_acc(g, phi, theta, psi, p, q, r, u, v, w, u_dot, v_dot, w_dot)
    a_x = u_dot + q .* w - r .* v + g .* sin(theta);
    a_y = v_dot + r .* u - p .* w - g .* cos(theta) .* sin(phi);
    a_z = w_dot + p .* v - q .* u - g .* cos(theta) .* cos(phi);
end