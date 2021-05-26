function [initial_states] = create_initial_states_struct(data, state_size, experiments_to_use, type)
    if type == "lon"
        initial_states_values = {};
        elevator_index = state_size; % elevator is last state
        if length(experiments_to_use) == 1
           for i = 1:state_size - 1
               initial_states_values(i) = {data(1,i,:,experiments_to_use).y};
           end
           initial_states_values(8) = {data(1,:,1,experiments_to_use).u};
        else
            for i = 1:state_size - 1
               initial_states_values(i) = {cell2mat(data(1,i,:,experiments_to_use).y)'};
            end
            % Load initial conditions for elevator
            initial_states_values(state_size) = {cell2mat(data(1,:,1,experiments_to_use).u)'};
        end

        initial_states = struct(...
            'Name', {'q0', 'q2', 'q','u', 'w','delta_e'},...
            'Unit', {'', '', 'rad/s', 'm/s', 'm/s','rad'}, ...
            'Value', initial_states_values, ...
            'Minimum', -Inf, 'Maximum', Inf, ...
            'Fixed', true);
    elseif type == "lat"
        disp("todo")
    end
end