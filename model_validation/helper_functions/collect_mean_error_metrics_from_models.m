function [mean_gof, mean_tic, mean_an, mean_mae, mean_rmse] = collect_mean_error_metrics_from_models(error_metrics, model_names, maneuver_types)
    mean_gof = {};
    mean_tic = {};
    mean_an = {};
    mean_mae = {};
    mean_rmse = {};
    num_models = length(model_names);
    for maneuver_type = maneuver_types
        curr_error_metric = error_metrics.(maneuver_type);
        for model_i = 1:num_models
            model_name = model_names(model_i);
            [curr_mean_gof, curr_mean_tic, curr_mean_an, curr_mean_mae, curr_mean_rmse] = get_mean_error_metrics(curr_error_metric, model_name);
            mean_gof.(model_name) = curr_mean_gof;
            mean_tic.(model_name) = curr_mean_tic;
            mean_an.(model_name) = curr_mean_an;
            mean_mae.(model_name) = curr_mean_mae;
            mean_rmse.(model_name) = curr_mean_rmse;
        end
    end
end