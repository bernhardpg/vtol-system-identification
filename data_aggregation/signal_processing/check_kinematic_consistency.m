function [] = check_kinematic_consistency(t, phi, theta, psi, p, q, r, t_recorded, phi_recorded, theta_recorded, psi_recorded)
    y0 = [phi(1) theta(1) psi(1)];
    tspan = [t(1) t(end)];
    [t_sim, y] = ode23s(@(t_sim,y) rot_kinematics(t_sim, y, t, [p q r]), tspan, y0);
    
    phi_pred = y(:,1);
    theta_pred = y(:,2);
    psi_pred = y(:,3);
    
    plot_attitude(t, phi, theta, psi, t_sim, phi_pred, theta_pred, psi_pred, t_recorded, phi_recorded, theta_recorded, psi_recorded)
end


function [dy_dt] = rot_kinematics(t, y, t_u, u)
    phi = y(1);
    theta = y(2);
    psi = y(2);
    
    p = interp1(t_u, u(:,1), t);
    q = interp1(t_u, u(:,2), t);
    r = interp1(t_u, u(:,3), t);

    phi_dot = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot = (q * sin(phi) + r * cos(phi)) * sec(theta);
    
    dy_dt = [phi_dot theta_dot psi_dot]';
end


function [] = plot_attitude(t, phi, theta, psi, t_pred, phi_pred, theta_pred, psi_pred, t_recorded, phi_recorded, theta_recorded, psi_recorded)
    figure
    subplot(3,1,1)
    plot(t, rad2deg(phi), t_pred, rad2deg(phi_pred), t_recorded, rad2deg(phi_recorded), '--r')
    legend("\phi", "\phi (predicted)", "\phi (recorded)")
    
    subplot(3,1,2)
    plot(t, rad2deg(theta), t_pred, rad2deg(theta_pred), t_recorded, rad2deg(theta_recorded), '--r')
    legend("\theta", "\theta (predicted)", "\theta (recorded)")
    
    subplot(3,1,3)
    plot(t, rad2deg(psi), t_pred, rad2deg(psi_pred), t_recorded, rad2deg(psi_recorded), '--r')
    legend("\psi", "\psi (predicted)", "\psi (recorded)")
end
