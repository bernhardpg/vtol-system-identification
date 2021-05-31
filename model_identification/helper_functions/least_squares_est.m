function [theta] = least_squares_est(Phi, Y)
    P = inv(Phi * Phi');
    B = Phi * Y;

    theta = P * B;
end