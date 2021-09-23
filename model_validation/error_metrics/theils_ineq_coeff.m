function tic = theils_ineq_coeff(y, z)
    N = length(y);
    num = sqrt((1/N) * diag((z - y)' * (z - y))');
    den = sqrt((1/N) * diag(z'*z)') + sqrt((1/N) * diag(y'*y)');
    tic = num ./ den;
end