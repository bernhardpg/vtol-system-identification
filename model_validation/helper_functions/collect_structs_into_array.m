function [array] = collect_structs_into_array(struct, model_names)
    num_models = length(model_names);
    num_cols = length(struct.(model_names(1)));
    
    array = zeros(num_models, num_cols);
    for model_i = 1:num_models
        array(model_i,:) = struct.(model_names(model_i));
    end
end