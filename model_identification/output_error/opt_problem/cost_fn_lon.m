function cost = cost_fn_lon(x, w, residual_weight, R_hat, seq_data, y0, tspan, z, const_params)
    v = calc_residuals_lon(z, seq_data, const_params, x, tspan, y0);
    cost = 0.5 * sum(diag(residual_weight * v / R_hat * w * v'));
end