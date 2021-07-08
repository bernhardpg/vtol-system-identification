function [RSS] = calc_RSS(y_hat, z)
    % Calculate Regression Sum of Squares
    RSS = (y_hat - z)' * (y_hat - z);
end