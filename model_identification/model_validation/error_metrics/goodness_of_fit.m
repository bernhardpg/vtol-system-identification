function gof = goodness_of_fit(y, z)
    [~, n_y] = size(y);
    num = diag(((z - y)' * (z - y)))';
    den = diag((z - z(1,:))' * (z - z(1,:)))';
    gof = ones(1,n_y) - num ./ den;
end