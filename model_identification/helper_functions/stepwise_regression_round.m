function [y_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_variables_str, R_sq_prev)
    [y_hat, F0, R_sq, cov_th, th_hat] = regression_analysis(X, z);

    disp("Independent variables: [" + indep_variables_str + "]")
    fprintf("F0: ")
    fprintf([repmat('%4.2f ',1,length(F0)) '\n'], F0);
    disp("R_sq: " + R_sq);
    if abs(R_sq_prev) > 1e-3
        R_sq_change = (R_sq - R_sq_prev) / R_sq_prev * 100;
        if R_sq_change < 0.5
            disp("WARNING: Too low change");
        end
        fprintf('Change in R_sq: %2.4f %%\n', R_sq_change);
    end
end