function [nlgr_model] = fix_parameters(parameters_to_fix, nlgr_model, fix)
    for i = parameters_to_fix
        nlgr_model.Parameters(i).Fixed = fix;
    end
end