function rmse = root_mean_square_error(y, z)
    N = length(z);
    rmse = sqrt(diag((1/N) * (z - y)'*(z - y)))';
end