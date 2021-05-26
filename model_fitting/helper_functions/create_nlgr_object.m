function [nlgr] = create_nlgr_object(parameters, initial_states, type)
    if type == "full"
        disp("not yet implemented")
    elseif type == "lon"
        % Create model
        FileName = 'longitudinal_model_c';
        Nx = 6; % number of states
        Ny = 5; % number of outputs
        Nu = 2; % number of inputs
        Order = [Ny Nu Nx];

        % Describe input
        InputName = {'delta_e_sp','n_p'};
        InputUnit =  {'rad', 'rpm'};

        % Describe state (which is equal to output)
        OutputName = {'q0', 'q2', 'q', 'u', 'w'};
        OutputUnit = {'', '','rad/s', 'm/s', 'm/s'};

        % Construct nlgr object
        Ts = 0; % Continuous system
        nlgr = idnlgrey(FileName, Order, parameters, initial_states, Ts, ...
            'Name', 'Longitudinal Model', ...
            'InputName', InputName, 'InputUnit', InputUnit, ...
            'OutputName', OutputName, 'OutputUnit', OutputUnit, ...
            'TimeUnit', 's');
    elseif type == "lat"
        disp("todo")
    end
end