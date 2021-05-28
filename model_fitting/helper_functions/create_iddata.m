function [data] = create_iddata(t, full_state, full_input, maneuver_start_indices, data_type)
    num_maneuvers = length(maneuver_start_indices);
    dt = 0.01; % TODO: Consider not hardcoding this!

    if data_type == "full"
        data = iddata('Name', 'Full state model');

        % Describe input
        InputName = {'nt1', 'nt2', 'nt3', 'nt4', 'delta_a_sp', 'delta_e_sp', 'delta_r_sp', 'n_p'};
        InputUnit =  {'rpm', 'rpm', 'rpm', 'rpm', 'rad', 'rad', 'rad', 'rpm'};

        % Describe state (which is equal to output)
        OutputName = {'q0', 'q1', 'q2', 'q3', 'p', 'q', 'r', 'u', 'v', 'w'};
        OutputUnit = {'', '', '', '', 'rad/s', 'rad/s', 'rad/s', 'm/s', 'm/s', 'm/s'};

        for maneuver_i = 1:num_maneuvers
            if maneuver_i == 1
                maneuver_start_index = 1;
            else
                maneuver_start_index = maneuver_start_indices(maneuver_i - 1);
            end

            maneuver_end_index = maneuver_start_indices(maneuver_i) - 1;

            % Extract only relevant maneuver data
            t_maneuver = t(maneuver_start_index:maneuver_end_index,:);
            full_state_maneuver = full_state(maneuver_start_index:maneuver_end_index,:);
            full_input_maneuver = full_input(maneuver_start_index:maneuver_end_index,:);

            quat = full_state_maneuver(:,1:4);
            e0 = quat(:,1);
            e1 = quat(:,2);
            e2 = quat(:,3);
            e3 = quat(:,4);

            p = full_state_maneuver(:,5);
            q = full_state_maneuver(:,6);
            r = full_state_maneuver(:,7);
            u = full_state_maneuver(:,8);
            v = full_state_maneuver(:,9);
            w = full_state_maneuver(:,10);
            
            n_t1 = full_input_maneuver(:,1);
            n_t2 = full_input_maneuver(:,2);
            n_t3 = full_input_maneuver(:,3);
            n_t4 = full_input_maneuver(:,4);
            delta_a = full_input_maneuver(:,5);
            delta_e = full_input_maneuver(:,6);
            delta_r = full_input_maneuver(:,7);
            n_p = full_input_maneuver(:,8);

            output = [e0 e1 e2 e3 p q r u v w];
            input = [n_t1 n_t2 n_t4 n_t4 ...
                     delta_a delta_e delta_r n_p];

            % Create sysid data object
            z = iddata(output, input, dt, 'Name', 'Full state data');
            z.TimeUnit = 's';
            z.Tstart = 0;
            z.InputName = InputName;
            z.InputUnit = InputUnit;
            z.OutputName = OutputName;
            z.OutputUnit = OutputUnit;

            if maneuver_i == 1
                data = z;
            else
                data = merge(data,z);
            end
        end
        
    elseif data_type == "lon"
        data = iddata('Name', 'Longitudinal data');

        % Describe input
        InputName = {'delta_e_sp','n_p'};
        InputUnit =  {'rad', 'rpm'};

        % Describe state (which is equal to output)
        OutputName = {'q0', 'q2', 'q', 'u', 'w'};
        OutputUnit = {'', '', 'rad/s', 'm/s', 'm/s'};

        for maneuver_i = 1:num_maneuvers
            if maneuver_i == 1
                maneuver_start_index = 1;
            else
                maneuver_start_index = maneuver_start_indices(maneuver_i - 1);
            end

            maneuver_end_index = maneuver_start_indices(maneuver_i) - 1;

            % Extract only relevant maneuver data
            t_maneuver = t(maneuver_start_index:maneuver_end_index,:);
            full_state_maneuver = full_state(maneuver_start_index:maneuver_end_index,:);
            full_input_maneuver = full_input(maneuver_start_index:maneuver_end_index,:);

            quat = full_state_maneuver(:,1:4);
            quat_only_pitch = create_quat_w_only_pitch_movement(quat);
            e0 = quat_only_pitch(:,1);
            e2 = quat_only_pitch(:,3);

            q = full_state_maneuver(:,6);
            u = full_state_maneuver(:,8);
            w = full_state_maneuver(:,10);
            delta_e = full_input_maneuver(:,6);
            n_p = full_input_maneuver(:,8);

            output = [e0 e2 q u w];
            input = [delta_e n_p];

            % Create sysid data object
            z = iddata(output, input, dt, 'Name', 'Pitch 211 maneuvers');
            z.TimeUnit = 's';
            z.Tstart = 0;
            z.InputName = InputName;
            z.InputUnit = InputUnit;
            z.OutputName = OutputName;
            z.OutputUnit = OutputUnit;

            if maneuver_i == 1
                data = z;
            else
                data = merge(data,z);
            end
        end
        
    elseif data_type == "lat"
        data = iddata('Name', 'Lateral data');

        % Describe input
        InputName = {'delta_a_sp', 'delta_r_sp', 'u', 'w'};
        InputUnit =  {'rad', 'rad', 'm/s', 'm/s'};

        % Describe state (which is equal to output)
        OutputName = {'q0', 'q1', 'q2', 'q3', 'p', 'r', 'v'};
        OutputUnit = {'', '', '', '', 'rad/s', 'rad/s', 'm/s'};

        for maneuver_i = 1:num_maneuvers
            if maneuver_i == 1
                maneuver_start_index = 1;
            else
                maneuver_start_index = maneuver_start_indices(maneuver_i - 1);
            end

            maneuver_end_index = maneuver_start_indices(maneuver_i) - 1;

            % Extract only relevant maneuver data
            t_maneuver = t(maneuver_start_index:maneuver_end_index,:);
            full_state_maneuver = full_state(maneuver_start_index:maneuver_end_index,:);
            full_input_maneuver = full_input(maneuver_start_index:maneuver_end_index,:);

            quat = full_state_maneuver(:,1:4);
            p = full_state_maneuver(:,5);
            r = full_state_maneuver(:,7);
            u = full_state_maneuver(:,8);
            v = full_state_maneuver(:,9);
            w = full_state_maneuver(:,10);
            delta_a = full_input_maneuver(:,5);
            delta_r = full_input_maneuver(:,7);

            output = [quat p r v];
            input = [delta_a delta_r u w];

            % Create sysid data object
            z = iddata(output, input, dt, 'Name', 'Lateral 211 maneuvers');
            z.TimeUnit = 's';
            z.Tstart = 0;
            z.InputName = InputName;
            z.InputUnit = InputUnit;
            z.OutputName = OutputName;
            z.OutputUnit = OutputUnit;

            if maneuver_i == 1
                data = z;
            else
                data = merge(data,z);
            end
        end
    end
end

function [quat_only_pitch] = create_quat_w_only_pitch_movement(quat)
    eul = quat2eul(quat);
    roll = eul(:,3);
    pitch = eul(:,2);
    yaw = eul(:,1);

    eul_only_pitch = [zeros(size(pitch)) pitch zeros(size(pitch))];
    quat_only_pitch = eul2quat(eul_only_pitch);
end