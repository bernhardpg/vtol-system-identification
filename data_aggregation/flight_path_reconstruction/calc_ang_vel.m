function [p, q, r] = calc_ang_vel(phi, theta, phi_dot, theta_dot, psi_dot)
    p = phi_dot - psi_dot .* sin(theta);
    q = theta_dot .* cos(phi) + psi_dot .* sin(phi) .* cos(theta);
    r = psi_dot .* cos(phi) .* cos(theta) - theta_dot .* sin(phi);
end