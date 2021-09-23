function anmae = average_norm_mae(y, z)
    mae = mean_absolute_error(y, z);
    anmae = mean(mae ./ range(z));
end