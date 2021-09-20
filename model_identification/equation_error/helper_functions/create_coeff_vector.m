function coeff_vector = create_coeff_vector(std_regr_order, th_hat, th_names)
    n_total_regr = length(std_regr_order);
    coeff_vector = zeros(n_total_regr,1);
    % Iterate through found regressors
    for regr_i = 1:length(th_hat)
        curr_regr_name = th_names(regr_i);
        % Find the right place in the coeff vector
        for coeff_i = 1:n_total_regr
            if strcmp(curr_regr_name, std_regr_order(coeff_i)) % found right place in coeff matrix
                coeff_vector(coeff_i) = th_hat(regr_i);
                break
            end
        end
    end
end