function cost = cost_fn(x, t_seq, y, input_seq, lat_state_seq, params)
    % x = opt_vars

    % Integration interval
    dt = t_seq(2) - t_seq(1);
    tspan = t_seq(1):dt:t_seq(end);
    % Initial values
    y0 = y(1,:);
    
    % Add decision variables to params before integrating
    all_params = [params;
                  x'];
              
    % Integrate dynamics
    % Use fixed step solver to reduce computational time
    y_pred = ode5(@(t,y) aircraft_dynamics_lon(t, y, t_seq, input_seq, lat_state_seq, all_params), tspan, y0);
    
    % Squared cost
    cost = sum(diag((y - y_pred)' * (y - y_pred)));
end