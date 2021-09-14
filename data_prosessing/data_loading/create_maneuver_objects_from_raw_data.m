function maneuvers = create_maneuver_objects_from_raw_data(maneuver_type, t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw)

    num_maneuvers = length(maneuver_start_indices_state);
    maneuvers = [];
    
    for maneuver_i = 1:num_maneuvers
        curr_maneuver = FlightPathData(maneuver_i, maneuver_type);

        % COLLECT STATE DATA
        % Get correct maneuver start and end index
        maneuver_start_index_state = maneuver_start_indices_state(maneuver_i);
        if maneuver_i == num_maneuvers
            maneuver_end_index_state = length(t_state_all_maneuvers);
        else
            maneuver_end_index_state = maneuver_start_indices_state(maneuver_i + 1) - 1;
        end         
        
        % Extract recorded data during maneuver
        curr_maneuver.RawData.Time = t_state_all_maneuvers(maneuver_start_index_state:maneuver_end_index_state,:);
        %t_recorded = t_recorded - t_recorded(1); % Make t go from 0 to end
        curr_maneuver.RawData.QuatNedToBody = q_NB_all_maneuvers(maneuver_start_index_state:maneuver_end_index_state,:);
        eul_recorded = quat2eul(curr_maneuver.RawData.QuatNedToBody);
        curr_maneuver.RawData.EulPhi = eul_recorded(:,3);
        curr_maneuver.RawData.EulTheta = eul_recorded(:,2);
        curr_maneuver.RawData.EulPsi = eul_recorded(:,1);
        
        v_NED = v_NED_all_maneuvers(maneuver_start_index_state:maneuver_end_index_state,:);
        curr_maneuver.RawData.VelN = v_NED(:,1);
        curr_maneuver.RawData.VelE = v_NED(:,2);
        curr_maneuver.RawData.VelD = v_NED(:,3);
        
        [curr_maneuver.RawData.VelBodyU, curr_maneuver.RawData.VelBodyV, curr_maneuver.RawData.VelBodyW] = calc_body_vel(curr_maneuver.RawData.EulPhi, curr_maneuver.RawData.EulTheta, curr_maneuver.RawData.EulPsi, curr_maneuver.RawData.VelN, curr_maneuver.RawData.VelE, curr_maneuver.RawData.VelD);
        
        % HANDLE INPUTS
        % Handle inputs which are on their own timestamps and start indices
        maneuver_start_index_u_fw = maneuver_start_indices_u_fw(maneuver_i);
        if maneuver_i == num_maneuvers
            maneuver_end_index_u_fw = length(t_u_fw_all_maneuvers);
        else
            maneuver_end_index_u_fw = maneuver_start_indices_u_fw(maneuver_i + 1) - 1;
        end
        
        curr_maneuver.RawData.TimeInput = t_u_fw_all_maneuvers(maneuver_start_index_u_fw:maneuver_end_index_u_fw,:);
        u_fw_maneuver = u_fw_all_maneuvers(maneuver_start_index_u_fw:maneuver_end_index_u_fw,:);
        curr_maneuver.RawData.DeltaASp = u_fw_maneuver(:,1);
        curr_maneuver.RawData.DeltaESp = u_fw_maneuver(:,2);
        curr_maneuver.RawData.DeltaRSp = u_fw_maneuver(:,3);
        curr_maneuver.RawData.DeltaT = u_fw_maneuver(:,4);
        
        maneuvers = [maneuvers;
                     curr_maneuver];
    end  
end