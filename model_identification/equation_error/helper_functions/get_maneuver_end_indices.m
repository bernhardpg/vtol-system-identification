function [maneuver_end_indices] = get_maneuver_end_indices(fpr_data, maneuver_types)
    maneuver_lengths = [];
    for maneuver_type = maneuver_types
        for maneuver_i = 1:length(fpr_data.(maneuver_type))
            maneuver = fpr_data.(maneuver_type)(maneuver_i);
            maneuver_lengths = [maneuver_lengths length(maneuver.Time)];
        end
    end
    maneuver_end_indices = cumsum(maneuver_lengths);
    maneuver_end_indices = maneuver_end_indices(1:end-1);
end