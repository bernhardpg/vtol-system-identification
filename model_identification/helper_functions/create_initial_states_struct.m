function [initial_states] = create_initial_states_struct(data, num_states, num_outputs, experiments_to_use, type)
    initial_states_values = {}; % At each index, this will contain an array of initial state values for each experiment

    if length(experiments_to_use) == 1
        % Load initial conditions for states available as outputs
        for state_index = 1:num_outputs
           initial_states_values(state_index) = {data(1,state_index,:,experiments_to_use).y};
        end
    else
        % Load initial conditions for states available as outputs
        for state_index = 1:num_outputs
           initial_states_values(state_index) = {cell2mat(data(1,state_index,:,experiments_to_use).y)'};
        end
    end
    
    if type == "lon"
        state_names = {'q0', 'q2', 'q', 'u', 'w', 'delta_e'};
        state_units = {'', '', 'rad/s', 'm/s', 'm/s', 'rad'};
        
        if length(experiments_to_use) == 1
            % Load initial conditions for elevator state which
            % are actually inputs
            elevator_index = 1;
            initial_states_values(num_outputs + elevator_index) = {data(1,:,elevator_index,experiments_to_use).u};
        else
            % Load initial conditions for elevator state which
            % are actually inputs
            elevator_index = 1;
            initial_states_values(num_outputs + elevator_index) = {cell2mat(data(1,:,elevator_index,experiments_to_use).u)'};
        end

    elseif type == "lat"
        state_names = {'q0', 'q1', 'q2', 'q3', 'p','r', 'v', 'delta_a', 'delta_r'};
        state_units = {'', '', '', '', 'rad/s', 'rad/s', 'm/s', 'rad', 'rad'};

        if length(experiments_to_use) == 1
            % Load initial conditions for aileron and rudder states which
            % are actually inputs
            aileron_index = 1;
            rudder_index = 2;
            
            initial_states_values(num_outputs + aileron_index) = {data(1,:,aileron_index,experiments_to_use).u};
            initial_states_values(num_outputs + rudder_index) = {data(1,:,rudder_index,experiments_to_use).u};
        else
            % Load initial conditions for aileron and rudder states which
            % are actually inputs
            aileron_index = 1;
            rudder_index = 2;
            
            initial_states_values(num_outputs + aileron_index) = {cell2mat(data(1,:,aileron_index,experiments_to_use).u)'};
            initial_states_values(num_outputs + rudder_index) = {cell2mat(data(1,:,rudder_index,experiments_to_use).u)'};
        end
    end
    
    initial_states = struct(...
    'Name', state_names,...
    'Unit', state_units , ...
    'Value', initial_states_values, ...
    'Minimum', -Inf, 'Maximum', Inf, ...
    'Fixed', true);
end