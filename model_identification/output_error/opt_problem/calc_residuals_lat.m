function [v] = calc_residuals_lat(z, seq_data, const_params, lon_params, x, dt, tspan, y0)
    all_params = [const_params;
                  lon_params';
                  x'];
              
    % Integrate dynamics
    [t_pred, y_pred] = ode45(@(t,y) full_dynamics_c(t, y, seq_data, all_params), tspan, y0);
    y_pred = interp1(t_pred, y_pred, tspan(1):dt:tspan(2)); % change y_pred to correct time

    % Squared cost
    v = z - y_pred; % residuals
end