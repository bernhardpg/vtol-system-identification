% Calculates n-th order derivative using splines
function x_der = calc_spline_derivative(t_data, x, t_target, dt_spline_knots, n)
    t_0 = t_target(1);
    t_end = t_target(end);
    x_spline = slmengine(t_data, x,'knots',t_0:dt_spline_knots:t_end + dt_spline_knots, 'plot', 'off'); % add 'plot', 'on' to see fit
    x_der = slmeval(t_target, x_spline, n);
end