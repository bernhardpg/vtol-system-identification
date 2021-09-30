function [mean_gof, mean_tic, mean_an, mean_mae, mean_rmse] = collect_mean_error_metrics_from_models(error_metrics, model_names, maneuver_types)
    num_models = length(model_names);
    
    mean_gof = {};
    mean_tic = {};
    mean_an = {};
    mean_mae = {};
    mean_rmse = {};
    for model_i = 1:num_models
        model_name = model_names(model_i);
        mean_gof.(model_name) = [];
        mean_tic.(model_name) = [];
        mean_an.(model_name) = [];
        mean_mae.(model_name) = [];
        mean_rmse.(model_name) = [];
        for maneuver_type = maneuver_types
            curr_error_metric = error_metrics.(maneuver_type);

            [curr_mean_gof, curr_mean_tic, curr_mean_an, curr_mean_mae, curr_mean_rmse] = get_mean_error_metrics(curr_error_metric, model_name);
            mean_gof.(model_name) = mean([mean_gof.(model_name); curr_mean_gof],1);
            mean_tic.(model_name) = mean([mean_tic.(model_name); curr_mean_tic],1);
            mean_an.(model_name) = mean([mean_an.(model_name); curr_mean_an],1);
            mean_mae.(model_name) = mean([mean_mae.(model_name); curr_mean_mae],1);
            mean_rmse.(model_name) = mean([mean_rmse.(model_name); curr_mean_rmse],1);
        end
    end
end