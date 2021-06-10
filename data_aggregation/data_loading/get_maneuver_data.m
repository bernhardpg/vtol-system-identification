function [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_sp_m, delta_e_sp_m, delta_r_sp_m, delta_a_m, delta_e_m, delta_r_m, n_p_m]...
    = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_e_sp, delta_r_sp, delta_a, delta_e, delta_r, n_p)
   
    num_maneuvers = length(maneuver_start_indices);
    maneuver_start = maneuver_start_indices(maneuver_i); 
    if maneuver_i == num_maneuvers
        maneuver_end = length(t);
    else
        maneuver_end = maneuver_start_indices(maneuver_i + 1) - 1;
    end
   
   t_m = t(maneuver_start:maneuver_end);
   phi_m = phi(maneuver_start:maneuver_end);
   theta_m = theta(maneuver_start:maneuver_end);
   psi_m = psi(maneuver_start:maneuver_end);
   p_m = p(maneuver_start:maneuver_end);
   q_m = q(maneuver_start:maneuver_end);
   r_m = r(maneuver_start:maneuver_end);
   u_m = u(maneuver_start:maneuver_end);
   v_m = v(maneuver_start:maneuver_end);
   w_m = w(maneuver_start:maneuver_end);
   a_x_m = a_x(maneuver_start:maneuver_end);
   a_y_m = a_y(maneuver_start:maneuver_end);
   a_z_m = a_z(maneuver_start:maneuver_end);
   delta_a_sp_m = delta_a_sp(maneuver_start:maneuver_end);
   delta_e_sp_m = delta_e_sp(maneuver_start:maneuver_end);
   delta_r_sp_m = delta_r_sp(maneuver_start:maneuver_end);
   delta_a_m = delta_a(maneuver_start:maneuver_end);
   delta_e_m = delta_e(maneuver_start:maneuver_end);
   delta_r_m = delta_r(maneuver_start:maneuver_end);
   n_p_m = n_p(maneuver_start:maneuver_end);
end