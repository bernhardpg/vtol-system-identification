function [] = print_param_struct(param_table)
    % param matrix: N_params x N_models
    num_models = length(param_table);
    [num_params, num_models] = size(param_matrix);
    
    disp("=== Free (non-fixed) parameter values ===")
    fprintf(['Parameter | ' repmat('Model_%1.0f | ',1,num_models) '\n'], 1:num_models);
    for i = 1:num_params
       params = param_matrix{i};
       param_name = parameters(i).Name;
       param_value = parameters(i).Value;
       disp(i + ": " + param_name + " = " + param_value);
       fprintf([param_name + ' | ' repmat('%4.2f | ', 1, num_models) '\n'], param_value);
    end 
end
