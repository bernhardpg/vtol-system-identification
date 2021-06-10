function [data, data_full_state] = create_datasets(metadata, maneuver_types, model_type, num_datasets)

    for dataset_i = 1:num_datasets
        
        
        [data, data_full_state] = create_combined_iddata(metadata, maneuver_types, model_type, maneuver_indices);
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

 function v = shuffle(v)
     v=v(randperm(length(v)));
 end