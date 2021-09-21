function [gof_mean, tic_mean, an_mean] = evaluate_error_metrics(error_metrics)
    gof_sum = 0;
    tic_sum = 0;
    anmae_sum = 0;
    anrmse_sum = 0;
    N_maneuvers = length(error_metrics);
    for i = 1:N_maneuvers
       gof_sum = gof_sum + error_metrics{i}.gof;
       tic_sum = tic_sum + error_metrics{i}.tic;
       anmae_sum = anmae_sum + error_metrics{i}.anmae;
       anrmse_sum = anrmse_sum + error_metrics{i}.anrmse;
    end

    gof_mean = gof_sum ./ N_maneuvers;
    tic_mean = tic_sum ./ N_maneuvers;
    anmae_mean = anmae_sum / N_maneuvers;
    anrmse_mean = anrmse_sum / N_maneuvers;
    an_mean = [anmae_mean anrmse_mean];
end