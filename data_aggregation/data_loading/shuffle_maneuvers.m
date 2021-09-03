function [t_recorded, eul_recorded, phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded, t_u_fw_recorded, u_fw_recorded]...
    = shuffle_maneuvers(maneuvers_to_skip, t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw)

    % Iterate through all maneuvers and do flight path reconstruction
    num_maneuvers = length(maneuver_start_indices_state);
    num_discarded_maneuvers = 0;
    
    % Randomly shuffle maneuver order
    disp("Shuffling order of maneuvers before aggregating...")
    maneuvers_to_parse = shuffle(1:num_maneuvers);
    
    for maneuver_i = maneuvers_to_parse
        % Skip maneuvers
        if any(maneuvers_to_skip(:) == maneuver_i)
            num_discarded_maneuvers = num_discarded_maneuvers + 1;
            continue
        end
        
        % Get correct maneuver start and end index
        maneuver_start_index_state = maneuver_start_indices_state(maneuver_i);
        if maneuver_i == num_maneuvers
            maneuver_end_index_state = length(t_state_all_maneuvers);
        else
            maneuver_end_index_state = maneuver_start_indices_state(maneuver_i + 1) - 1;
        end            

        % Extract recorded data during maneuver
        t_recorded = t_state_all_maneuvers(maneuver_start_index_state:maneuver_end_index_state,:);
        q_NB = q_NB_all_maneuvers(maneuver_start_index_state:maneuver_end_index_state,:);
        v_NED = v_NED_all_maneuvers(maneuver_start_index_state:maneuver_end_index_state,:);

        eul_recorded = quat2eul(q_NB);
        phi_recorded = eul_recorded(:,3);
        theta_recorded = eul_recorded(:,2);
        psi_recorded = eul_recorded(:,1);
        v_N_recorded = v_NED(:,1);
        v_E_recorded = v_NED(:,2);
        v_D_recorded = v_NED(:,3);
        
        % Handle inputs
        maneuver_start_index_u_fw = maneuver_start_indices_u_fw(maneuver_i);
        if maneuver_i == num_maneuvers
            maneuver_end_index_u_fw = length(t_u_fw_all_maneuvers);
        else
            maneuver_end_index_u_fw = maneuver_start_indices_u_fw(maneuver_i + 1) - 1;
        end
        
        t_u_fw_recorded = t_u_fw_all_maneuvers(maneuver_start_index_u_fw:maneuver_end_index_u_fw,:);
        u_fw_recorded = u_fw_all_maneuvers(maneuver_start_index_u_fw:maneuver_end_index_u_fw,:);
    end

    disp("Loaded " + (num_maneuvers - num_discarded_maneuvers) + " maneuvers." + " Discarded " + num_discarded_maneuvers + " maneuvers due to dropout.");
end

 function v = shuffle(v)
     v = v(randperm(length(v)));
 end
 
 