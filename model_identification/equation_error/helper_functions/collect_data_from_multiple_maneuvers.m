function collected_data = collect_data_from_multiple_maneuvers(fpr_data, maneuver_types, data_type)
    collected_data = [];
    for maneuver_type = maneuver_types
        for maneuver_i = 1:length(fpr_data.(maneuver_type))
            maneuver = fpr_data.(maneuver_type)(maneuver_i);
            collected_data = [collected_data
                              maneuver.(data_type)];
        end
    end
end