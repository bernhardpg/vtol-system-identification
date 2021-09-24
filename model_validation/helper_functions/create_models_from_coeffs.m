function models = create_models_from_coeffs(model_coeffs, model_type)
    models = {};
    for model_i = 1:numel(model_coeffs)
        if strcmp(model_type, "lateral-directional")
            models{model_i} = NonlinearModel(zeros(5,3), model_coeffs{model_i});
        elseif strcmp(model_type, "longitudinal")
            models{model_i} = NonlinearModel(model_coeffs{model_i}, zeros(6,3));
        end
    end
end