function [r] = calculate_partial_correlation(X, z)
    [~, num_vars] = size(X);
    r = zeros(1, num_vars);
    for i = 1:num_vars
       temp = corrcoef(z, X(:,i)); 
       r(i) = temp(2,1); % cross correlation
    end
end