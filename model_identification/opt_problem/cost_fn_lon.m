function cost = cost_fn_lon(x, dt, seq_data, y0, tspan, y_lon_seq_m, const_params)
    % Add decision variables to params before integrating
    all_params = [const_params;
                  x'];
    
    % Integrate dynamics
    
    [t_pred, y_pred] = ode45(@(t,y) lon_dynamics_c(t, y, seq_data, all_params), tspan, y0);
    y_pred = interp1(t_pred, y_pred, tspan(1):dt:tspan(2));

    % Squared cost
    cost = sum(diag((y_lon_seq_m - y_pred)' * (y_lon_seq_m - y_pred)));
end
