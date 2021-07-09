% Calculate accelerations from kinematic relationship
function [acc] = calc_acc_lat(y_pred, seq_data, all_params)
    t_seq = seq_data(:,1);
    N = length(y_pred);
    p_dot = zeros(N,1);
    r_dot = zeros(N,1);
    y_pred_dot = zeros(N,5);
    for i = 1:N
        y_pred_dot(i,:) = lat_dynamics_liftdrag_c(t_seq(i), y_pred(i,:), seq_data, all_params);
    end
    
    phi_dot = y_pred_dot(:,1);
    psi_dot = y_pred_dot(:,2);
    p_dot = y_pred_dot(:,3);
    r_dot = y_pred_dot(:,4);
    v_dot = y_pred_dot(:,5);
    phi = y_pred(:,1);
    psi = y_pred(:,2);
    p = y_pred(:,3);
    r = y_pred(:,4);
    v = y_pred(:,4);
    theta = seq_data(:,6);
    q = seq_data(:,7);
    u = seq_data(:,8);
    w = seq_data(:,9);

    a_y = v_dot + r .* u - p .* w - 9.81 .* cos(theta) .* sin(phi);
    
    acc = [a_y p_dot r_dot];
end