function nrmse = norm_rmse(y, z)
    rmse = root_mean_square_error(y, z);
    nrmse = rmse ./ range(z);
end