function [th_hat] = LSE(X, z)
    D = (X' * X)^(-1);
    th_hat = D * X' * z;
end
