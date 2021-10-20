function error_metric = calc_error_metrics(y,z,maneuver_id)
    error_metric = {};
    error_metric.ManeuverId = maneuver_id;
    error_metric.mae = mean_absolute_error(y, z);
    error_metric.rmse = root_mean_square_error(y, z);
    error_metric.anrmse = mean(norm_rmse(y, z));
    error_metric.anmae = mean(norm_mae(y, z));
    error_metric.mae = norm_mae(y, z);
    error_metric.rmse = norm_rmse(y, z);
    error_metric.gof = goodness_of_fit(y, z);
    error_metric.tic = theils_ineq_coeff(y, z);
end