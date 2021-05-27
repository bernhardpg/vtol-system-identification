function [nlgr] = create_nlgr_object(Nx, Ny, Nu, parameters, initial_states, type)
    Order = [Ny Nu Nx];

    if type == "full"
        disp("not yet implemented")
        
        
    elseif type == "lon"
        FileName = 'longitudinal_model_c';

        InputName = {'delta_e_sp', 'n_p'};
        InputUnit =  {'rad', 'rpm'};

        OutputName = {'q0', 'q2', 'q', 'u', 'w'};
        OutputUnit = {'', '','rad/s', 'm/s', 'm/s'};

    elseif type == "lat"
        FileName = 'lateral_model_c';
        
        InputName = {'delta_a_sp', 'delta_r_sp', 'u', 'w'};
        InputUnit =  {'rad', 'rad', 'm/s', 'm/s'};

        OutputName = {'q0', 'q1', 'q2', 'q3', 'p', 'r', 'v'};
        OutputUnit = {'', '', '', '', 'rad/s', 'rad/s', 'm/s'};
        
    end
    
    Ts = 0; % Continuous system
    nlgr = idnlgrey(FileName, Order, parameters, initial_states, Ts, ...
        'Name', 'Longitudinal Model', ...
        'InputName', InputName, 'InputUnit', InputUnit, ...
        'OutputName', OutputName, 'OutputUnit', OutputUnit, ...
        'TimeUnit', 's');
end