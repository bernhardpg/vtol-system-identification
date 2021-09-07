% Calculate accelerations from kinematic relationship
function [acc] = calc_acc_lon(y_pred, seq_data, all_params)
    t_seq = seq_data(:,1);
    N = length(y_pred);
    q_dot = zeros(N,1);
    y_pred_dot = zeros(N,4);
    for i = 1:N
        y_pred_dot(i,:) = lon_dynamics_liftdrag_c(t_seq(i), y_pred(i,:), seq_data, all_params);
    end
    
    %theta_dot = y_pred_dot(:,1);
    q_dot = y_pred_dot(:,2);
    u_dot = y_pred_dot(:,3);
    w_dot = y_pred_dot(:,4);
    theta = y_pred(:,1);
    q = y_pred(:,2);
    u = y_pred(:,3);
    w = y_pred(:,4);
    phi = seq_data(:,6);
    %psi = seq_data(:,7);
    p = seq_data(:,8);
    r = seq_data(:,9);
    v = seq_data(:,10);

    a_x = u_dot + q .* w - r .* v + 9.81 .* sin(theta);
    a_z = w_dot + p .* v - q .* u - 9.81 .* cos(theta) .* cos (phi);
    
    acc = [a_x a_z q_dot];
end