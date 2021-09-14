function [a_x, a_y, a_z] = calc_trans_acc(g, phi, theta, p, q, r, u, v, w, u_dot, v_dot, w_dot)
    a_x = u_dot + q .* w - r .* v + g .* sin(theta);
    a_y = v_dot + r .* u - p .* w - g .* cos(theta) .* sin(phi);
    a_z = w_dot + p .* v - q .* u - g .* cos(theta) .* cos(phi);
end