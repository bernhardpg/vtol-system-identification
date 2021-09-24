function [mean_gof, mean_tic, mean_an] = collect_mean_error_metrics_from_models(error_metrics, model_names)
    mean_gof = {};
    mean_tic = {};
    mean_an = {};
    num_models = length(model_names);
    for model_i = 1:num_models
        model_name = model_names(model_i);
        [curr_mean_gof, curr_mean_tic, curr_mean_an] = get_mean_error_metrics(error_metrics, model_name);
        mean_gof.(model_name) = curr_mean_gof;
        mean_tic.(model_name) = curr_mean_tic;
        mean_an.(model_name) = curr_mean_an;
    end
end