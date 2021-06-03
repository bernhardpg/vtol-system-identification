function [initial_states] = create_initial_states_struct(data, num_states, num_outputs, type)
    initial_states_values = {}; % At each index, this will contain an array of initial state values for each experiment
    num_experiments = length(data.ExperimentName);
    
    if num_experiments == 1
        % Load initial conditions for states available as outputs
        for state_index = 1:num_outputs
           initial_states_values(state_index) = {data(1,state_index,:,:).y};
        end
    else
        % Load initial conditions for states available as outputs
        for state_index = 1:num_outputs
           initial_states_values(state_index) = {cell2mat(data(1,state_index,:,:).y)'};
        end
    end
    
    if type == "lon"
        state_names = {'q0', 'q2', 'q', 'u', 'w', 'delta_e'};
        state_units = {'', '', 'rad/s', 'm/s', 'm/s', 'rad'};
        
        if num_experiments == 1
            % Load initial conditions for elevator state which
            % are actually inputs
            elevator_input_index = 1;
            initial_states_values(num_outputs + elevator_input_index) = {data(1,:,elevator_input_index,:).u};
        else
            % Load initial conditions for elevator state which
            % are actually inputs
            elevator_input_index = 1;
            initial_states_values(num_outputs + elevator_input_index) = {cell2mat(data(1,:,elevator_input_index,:).u)'};
        end

    elseif type == "lat"
        state_names = {'q0', 'q1', 'q2', 'q3', 'p','r', 'v', 'delta_a', 'delta_r'};
        state_units = {'', '', '', '', 'rad/s', 'rad/s', 'm/s', 'rad', 'rad'};

        if num_experiments == 1
            % Load initial conditions for aileron and rudder states which
            % are actually inputs
            aileron_input_index = 1;
            rudder_input_index = 2;
            
            initial_states_values(num_outputs + aileron_input_index) = {data(1,:,aileron_input_index,:).u};
            initial_states_values(num_outputs + rudder_input_index) = {data(1,:,rudder_input_index,:).u};
        else
            % Load initial conditions for aileron and rudder states which
            % are actually inputs
            aileron_input_index = 1;
            rudder_input_index = 2;
            
            initial_states_values(num_outputs + aileron_input_index) = {cell2mat(data(1,:,aileron_input_index,:).u)'};
            initial_states_values(num_outputs + rudder_input_index) = {cell2mat(data(1,:,rudder_input_index,:).u)'};
        end
        
    elseif type == "full_lat_fixed"
        state_names = {'q0', 'q1', 'q2', 'q3', 'q', 'u', 'w', 'delta_a', 'delta_e', 'delta_r'};
        state_units = {'', '', '', '', 'rad/s', 'm/s', 'm/s', 'rad', 'rad', 'rad'};
        
        if num_experiments == 1
            % Load initial conditions for aileron, elevator and rudder states which
            % are actually inputs
            aileron_input_index = 1;
            elevator_input_index = 2;
            rudder_input_index = 3;
            num_inputs_before_control_surfaces = 4;
            
            initial_states_values(num_outputs + aileron_input_index) = {data(1,:,num_inputs_before_control_surfaces + aileron_input_index,:).u};
            initial_states_values(num_outputs + elevator_input_index) = {data(1,:,num_inputs_before_control_surfaces + elevator_input_index,:).u};
            initial_states_values(num_outputs + rudder_input_index) = {data(1,:,num_inputs_before_control_surfaces + rudder_input_index,:).u};
        else
            % Load initial conditions for aileron and rudder states which
            % are actually inputs
            aileron_input_index = 1;
            elevator_input_index = 2;
            rudder_input_index = 3;
            num_inputs_before_control_surfaces = 4;
            
            initial_states_values(num_outputs + aileron_input_index) = {cell2mat(data(1,:,num_inputs_before_control_surfaces + aileron_input_index,:).u)'};
            initial_states_values(num_outputs + elevator_input_index) = {cell2mat(data(1,:,num_inputs_before_control_surfaces + elevator_input_index,:).u)'};
            initial_states_values(num_outputs + rudder_input_index) = {cell2mat(data(1,:,num_inputs_before_control_surfaces + rudder_input_index,:).u)'};
        end
        
    elseif type == "full"
        % Describe state (which is equal to output)
        state_names = {'q0', 'q1', 'q2', 'q3', 'p', 'q', 'r', 'u', 'v', 'w', 'delta_a', 'delta_e', 'delta_r'};
        state_units = {'', '', '', '', 'rad/s', 'rad/s', 'rad/s', 'm/s', 'm/s', 'm/s', 'rad', 'rad', 'rad'};

        if num_experiments == 1
            % Load initial conditions for aileron, elevator and rudder states which
            % are actually inputs
            aileron_input_index = 1;
            elevator_input_index = 2;
            rudder_input_index = 3;
            num_inputs_before_control_surfaces = 4;
            
            initial_states_values(num_outputs + aileron_input_index) = {data(1,:,num_inputs_before_control_surfaces + aileron_input_index,:).u};
            initial_states_values(num_outputs + elevator_input_index) = {data(1,:,num_inputs_before_control_surfaces + elevator_input_index,:).u};
            initial_states_values(num_outputs + rudder_input_index) = {data(1,:,num_inputs_before_control_surfaces + rudder_input_index,:).u};
        else
            % Load initial conditions for aileron and rudder states which
            % are actually inputs
            aileron_input_index = 1;
            elevator_input_index = 2;
            rudder_input_index = 3;
            num_inputs_before_control_surfaces = 4;
            
            initial_states_values(num_outputs + aileron_input_index) = {cell2mat(data(1,:,num_inputs_before_control_surfaces + aileron_input_index,:).u)'};
            initial_states_values(num_outputs + elevator_input_index) = {cell2mat(data(1,:,num_inputs_before_control_surfaces + elevator_input_index,:).u)'};
            initial_states_values(num_outputs + rudder_input_index) = {cell2mat(data(1,:,num_inputs_before_control_surfaces + rudder_input_index,:).u)'};
        end
        
    end
    
    initial_states = struct(...
    'Name', state_names,...
    'Unit', state_units , ...
    'Value', initial_states_values, ...
    'Minimum', -Inf, 'Maximum', Inf, ...
    'Fixed', true);
end