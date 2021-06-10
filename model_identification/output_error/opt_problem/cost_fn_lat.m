function cost = cost_fn_lat(x, w, residual_weight, R_hat, dt, seq_data, y0, tspan, z, const_params, lon_params)
    v = calc_residuals_lat(z, seq_data, const_params, lon_params, x, dt, tspan, y0);
    cost = 0.5 * sum(diag(residual_weight * v / R_hat * w * v'));
end