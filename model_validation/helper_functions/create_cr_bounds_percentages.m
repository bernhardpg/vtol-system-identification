function cr_bounds_percentage = create_cr_bounds_percentages(model_names, cr_bounds, coeffs)
    cr_bounds_percentage = {};
    for model_i = 1:numel(model_names)
        cr_bounds_percentage.(model_names(model_i)) = calc_percentage_cr_bound(coeffs{model_i}, cr_bounds{model_i});
    end
end