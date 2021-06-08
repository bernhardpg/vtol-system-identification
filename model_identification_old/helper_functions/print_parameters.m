function [] = print_parameters(parameters, type)
    num_parameters = length(parameters);
    disp(" ")
    if type == "all"
        disp("=== All parameter values ===")
        for i = 1:num_parameters
           param_name = parameters(i).Name;
           param_value = parameters(i).Value;
           disp(i + ": " + param_name + " = " + param_value);
        end
    elseif type == "free"
        disp("=== Free (non-fixed) parameter values ===")
        for i = 1:num_parameters
           if ~parameters(i).Fixed
               param_name = parameters(i).Name;
               param_value = parameters(i).Value;
               disp(i + ": " + param_name + " = " + param_value);
           end
        end 
    end
end
