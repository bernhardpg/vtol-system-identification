function [data, data_full_state] = create_combined_iddata(metadata, maneuver_types, maneuver_quantities, model_type)
    for maneuver_type_i = 1:length(maneuver_types)
        maneuver_type = maneuver_types(maneuver_type_i);
        [full_state, full_input, t, maneuver_start_indices] = read_experiment_data(metadata, maneuver_type);

        curr_data_full_state = create_iddata(t, full_state, full_input, maneuver_start_indices, "full");
        curr_data = create_iddata(t, full_state, full_input, maneuver_start_indices, model_type);

        if maneuver_type_i == 1
            data_full_state = curr_data_full_state(:,:,:,1:maneuver_quantities(maneuver_type_i));
            data = curr_data(:,:,:,1:maneuver_quantities(maneuver_type_i));
        else
            data_full_state = merge(data_full_state, curr_data_full_state(:,:,:,1:maneuver_quantities(maneuver_type_i)));
            data = merge(data, curr_data(:,:,:,1:maneuver_quantities(maneuver_type_i)));
        end
    end
end