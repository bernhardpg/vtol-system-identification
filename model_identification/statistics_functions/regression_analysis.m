function [y_hat, F0, R_sq, cov_th, th_hat] = regression_analysis(X, z)
    [N, n_p] = size(X);
    D = (X' * X)^(-1);
    d = diag(D);
    th_hat = D * X' * z;
    
    y_hat = X * th_hat; % Estimated output
    
    v = z - y_hat; % Residuals
    sig_sq_hat = v' * v / (N - n_p); % Estimated noise variance
    cov_th = sig_sq_hat * d; % Estimates parameter variance
    
    % Calculate partial F metrix F0
    F0 = th_hat .^ 2 ./ cov_th;
    
    % Calculate R^2
    z_bar = mean(z);
    R_sq = (y_hat' * z - N * z_bar^2) / (z' * z - N * z_bar^2);
end