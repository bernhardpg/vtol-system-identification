function nmae = norm_mae(y, z)
    mae = mean_absolute_error(y, z);
    nmae = mae ./ range(z);
end