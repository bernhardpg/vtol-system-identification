function [v] = calc_residuals_lon(z, seq_data, const_params, x, tspan, y0)
    all_params = [const_params;
                  x'];
              
    t_seq = seq_data(:,1);
    % Integrate dynamics
    [t_pred, y_pred] = ode45(@(t,y) lon_dynamics_liftdrag_c(t, y, seq_data, all_params), tspan, y0);
    y_pred = interp1(t_pred, y_pred, t_seq); % change y_pred to correct time
  
    acc = calc_acc_lon(y_pred, seq_data, all_params);
    v = z - [y_pred acc]; % residuals
end
