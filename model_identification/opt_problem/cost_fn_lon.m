function cost = cost_fn_lon(x, dt, t_seq, y_lon_seq, y_lat_seq, input_seq, const_params, maneuver_indices)
    % x = opt_vars
    % Takes y_lat as pure input.
    % Only integrates y_lon and penalizes this
    
    % Add decision variables to params before integrating
    all_params = [const_params;
                  x'];
    
    costs = zeros(length(maneuver_indices),1);
    
    % Integrate all maneuvers
    for i = 1:length(maneuver_indices) - 1
        [t_seq_m, y_lon_seq_m, y_lat_seq_m, input_seq_m] = extract_man_data_lon(i, maneuver_indices, t_seq, y_lon_seq, y_lat_seq, input_seq);
       
        % Integrate dynamics
        tspan = [t_seq_m(1) t_seq_m(end)];
        y0 = y_lon_seq_m(1,:);
        
        [t_pred, y_pred] = ode45(@(t,y) lon_dynamics(t, y, input_seq_m, y_lat_seq_m, all_params), tspan, y0);
        
        y_pred = interp1(t_pred, y_pred, tspan(1):dt:tspan(2));
        
        % Squared cost
        costs(i) = sum(diag((y_lon_seq_m - y_pred)' * (y_lon_seq_m - y_pred)));
    end
    cost = sum(costs);
end

function [t_seq_m, y_lon_seq_m, y_lat_seq_m, input_seq_m] = extract_man_data_lon(i, maneuver_indices, t_seq, y_lon_seq, y_lat_seq, input_seq)
    m_start = maneuver_indices(i);
    m_end = maneuver_indices(i + 1) - 1;

    % Get data sequence for this maneuver
    t_seq_m = t_seq(m_start:m_end);
    y_lon_seq_m = y_lon_seq(m_start:m_end,:);
    y_lat_seq_m = y_lat_seq(m_start:m_end,:);
    input_seq_m = input_seq(m_start:m_end,:);
end