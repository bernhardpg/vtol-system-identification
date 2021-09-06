function [delta] = simulate_control_surface_dynamics(t_seq, delta_sp)
   tspan = [t_seq(1) t_seq(end)];
   airframe_static_properties;
   [t_pred, delta] = ode45(@(t,y) servo_dynamics(t, y, t_seq, delta_sp, servo_time_const_s, servo_rate_lim_rad_s), tspan, delta_sp(1));
   delta = interp1(t_pred, delta, t_seq);
end

function dy_dt = servo_dynamics(t, y, t_seq, u_seq, Tc, rate_lim)
   u = interp1(t_seq, u_seq, t);
   dy_dt = bound(-y / Tc + u / Tc, -rate_lim, rate_lim);
end

function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y = min(max(x,bl),bu);
end