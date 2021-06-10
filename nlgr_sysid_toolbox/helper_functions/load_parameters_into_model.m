function [nlgr_model] = load_parameters_into_model(nlgr_model, params)
    for i = 1:length(nlgr_model.Parameters)
        for j = 1:length(params)
            if strcmp(nlgr_model.Parameters(i).Name, params(j).Name)
                nlgr_model.Parameters(i).Value = params(j).Value;
            end
        end
    end
end