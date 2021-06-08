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