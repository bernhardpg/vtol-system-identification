function [nlgr_model] = reset_parameters(nlgr_model, param_indices, params)
    for i = param_indices
        nlgr_model.Parameters(i).Value = params(i).Value;
    end
end