function [predicted_outputs, error_metrics_for_maneuver] = evaluate_models_on_maneuver(maneuver, models, model_names, model_type)
    predicted_outputs = {};
    error_metrics_for_maneuver = {};
    num_models = numel(models);
    
    for model_i = 1:num_models
        error_metrics_for_maneuver.(model_names(model_i)) = {};
        [curr_predicted_output, curr_error_calculations] = evaluate_model(maneuver, models{model_i}, model_type);
        error_metrics_for_maneuver.(model_names(model_i)) = curr_error_calculations;
        predicted_outputs.(model_names(model_i)) = curr_predicted_output;
    end
end