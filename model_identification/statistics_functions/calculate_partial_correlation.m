function [r] = calculate_partial_correlation(X, z)
    X_bar = mean(X);
    N = length(z);
    z_bar = mean(z);
    
    cov_Xz = (X - X_bar)' * (z - z_bar) / (N - 1);
    var_X = diag((X - X_bar)' * (X - X_bar)) / (N - 1);
    var_z = diag((z - z_bar)' * (z - z_bar)) / (N - 1);
    r = cov_Xz ./ sqrt(var_X * var_z);
end