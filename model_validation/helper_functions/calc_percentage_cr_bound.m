function [cr_bound_percentage] = calc_percentage_cr_bound(params, cr_bounds)
    cr_bound_percentage = abs(sqrt(cr_bounds) * 2 ./ reshape(params,size(cr_bounds))) * 100;
    cr_bound_percentage = fillmissing(cr_bound_percentage,'constant',0); % remove NaNs
end