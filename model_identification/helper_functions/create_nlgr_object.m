function [nlgr] = create_nlgr_object(Nx, Ny, Nu, parameters, initial_states, type)
    Order = [Ny Nu Nx];

    if type == "full"
        FileName = 'full_state_model_c';
        
        InputName = {'nt1', 'nt2', 'nt3', 'nt4', 'delta_a_sp', 'delta_e_sp', 'delta_r_sp', 'n_p'};
        InputUnit =  {'rpm', 'rpm', 'rpm', 'rpm', 'rad', 'rad', 'rad', 'rpm'};

        OutputName = {'q0', 'q1', 'q2', 'q3', 'p', 'q', 'r', 'u', 'v', 'w'};
        OutputUnit = {'', '', '', '', 'rad/s', 'rad/s', 'rad/s', 'm/s', 'm/s', 'm/s'};
        
        
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