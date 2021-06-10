function [collected_params] = create_collected_params(model_paths_to_load)
    collected_params = create_param_struct("full");
    num_params = length(collected_params);

    for model_i = 1:size(model_paths_to_load)
        model_path = model_paths_to_load(model_i);

        load(model_path + "model.mat");
        model_params = nlgr_model.Parameters;

        for param_i = 1:num_params
            for model_param_i = 1:size(model_params)
                if strcmp(collected_params(param_i).Name, model_params(model_param_i).Name)
                   collected_params(param_i).Value = model_params(model_param_i).Value;
                end
            end
        end
    end
end