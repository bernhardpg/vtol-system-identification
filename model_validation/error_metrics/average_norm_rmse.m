function anrmse = average_norm_rmse(y, z)
    rmse = root_mean_square_error(y, z);
    anrmse = rmse ./ range(z);
end