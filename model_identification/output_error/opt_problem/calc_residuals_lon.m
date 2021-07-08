function [v] = calc_residuals_lon(z, seq_data, const_params, x, tspan, y0)
    all_params = [const_params;
                  x'];
              
    t_seq = seq_data(:,1);
    % Integrate dynamics
    [t_pred, y_pred] = ode45(@(t,y) lon_dynamics_liftdrag_c(t, y, seq_data, all_params), tspan, y0);
    y_pred = interp1(t_pred, y_pred, t_seq); % change y_pred to correct time
  
    % Calculate accelerations from kinematic relationship
    N = length(y_pred);
    acc = zeros(N,2);
    q_dot = zeros(N,1);
    for i = 1:N
        y_pred_dot = lon_dynamics_liftdrag_c(t_seq(i), y_pred(i,:), seq_data, all_params);
        q_dot(i) = y_pred_dot(2);
        u_dot = y_pred_dot(3);
        w_dot = y_pred_dot(4);
        theta = y_pred(i,1);
        q = y_pred(i,2);
        u = y_pred(i,3);
        w = y_pred(i,4);
        phi = seq_data(i,6);
        %psi = seq_data(i,7);
        p = seq_data(i,8);
        r = seq_data(i,9);
        v = seq_data(i,10);
        
        acc(i,:) = calc_acc(u_dot, w_dot, p, q, r, u, v, w, theta, phi);
    end
    
    v = z - [y_pred acc q_dot]; % residuals
end

function [acc] = calc_acc(u_dot, w_dot, p, q, r, u, v, w, theta, phi)
    a_x = u_dot + q * w - r * v + 9.81 * sin(theta);
    a_z = w_dot + p * v - q * u - 9.81 * cos(theta) * cos (phi);
    acc = [a_x a_z];
end