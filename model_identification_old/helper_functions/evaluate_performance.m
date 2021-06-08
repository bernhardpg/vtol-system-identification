function [fit] = evaluate_performance(params, input_trims, val_data, num_states, num_inputs, num_outputs, model_type)
    initial_states = create_initial_states_struct(val_data, input_trims, num_states, num_outputs, model_type);
    nlgr_model = create_nlgr_object(num_states, num_outputs, num_inputs, params, initial_states, model_type);

    num_experiments = length(val_data.ExperimentName);
    fits = zeros(num_experiments, num_outputs);
    for i = 1:num_experiments
        exp_name = string(val_data.ExperimentName);
        y = sim(nlgr_model, val_data);

        % Handle datatypes being different for single and multiple experiments
        if num_experiments == 1
            output_pred = y.y;
            output = val_data.y;
        else
            output_pred = cell2mat(y.y(i));
            output = cell2mat(val_data.y(i));
        end
        
        fits(i,:) = calc_fit(output, output_pred);
    end
    fit = mean(fits);
end

function [fit] = calc_fit(y, y_pred)
    [~, Ny] = size(y);
    fit = 100 * (ones(1,Ny) - vecnorm(y - y_pred) ./ vecnorm(y - mean(y)));
end