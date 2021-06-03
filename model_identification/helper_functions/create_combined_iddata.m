function [data, data_full_state] = create_combined_iddata(metadata, maneuver_types, num_maneuver_types, model_type)
    first_data_loaded = false;
    dt = metadata.dt;
    
    for maneuver_type_i = 1:length(maneuver_types)
        maneuver_quantity = num_maneuver_types(maneuver_type_i);
        if maneuver_quantity == 0
            continue
        end
        maneuver_type = maneuver_types(maneuver_type_i);
        choose_num_maneuvers = num_maneuver_types(maneuver_type_i);
        [full_state, full_input, t, maneuver_start_indices] = read_experiment_data(metadata, maneuver_type);
        total_num_maneuvers = length(maneuver_start_indices);

        curr_data_full_state = create_iddata(dt, t, full_state, full_input, maneuver_start_indices, "full");
        curr_data = create_iddata(dt, t, full_state, full_input, maneuver_start_indices, model_type);

        % Randomly select num_maneuver maneuvers
        maneuvers_to_use = rand_pick_n_indices(choose_num_maneuvers, total_num_maneuvers);
        selected_data_full_state = curr_data_full_state(:,:,:,maneuvers_to_use);
        selected_data = curr_data(:,:,:,maneuvers_to_use);
            
        if ~first_data_loaded
            data_full_state = selected_data_full_state;
            data = selected_data;
            first_data_loaded = true;
        else
            data_full_state = merge(data_full_state, selected_data_full_state);
            data = merge(data, selected_data);
        end
    end
end

function [randomly_picked_indices] = rand_pick_n_indices(choose_num_maneuvers, total_num_maneuvers)
    randomly_picked_indices = [];
    while length(randomly_picked_indices) < choose_num_maneuvers
        new_index = randi(total_num_maneuvers);
        new_index_already_used = sum(ismember(randomly_picked_indices, new_index)) == 1;
        if ~new_index_already_used
            randomly_picked_indices = [randomly_picked_indices new_index];
        end
    end
end