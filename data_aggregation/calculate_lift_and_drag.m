function [L, D, c_L, c_D] = calculate_lift_and_drag(state, input, AoA_rad, V_a, acc_B, m, g)
    % Read data
    N = length(state);
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);

    % Create rotation matrices to stability frame
    q_BN = quatinv(q_NB);
    R_BN = quat2rotm(q_BN);

    eul_rad = [zeros(size(AoA_rad)) AoA_rad zeros(size(AoA_rad))];
    R_SB = eul2rotm(eul_rad); % For some reason this gives a left-handed rotation

    % Calculate gravitational force in Stability frame
    Fg_N = [0; 0; m*g];
    Fg_S = zeros(N,3);
    for i = 1:N
       R_SN_at_i = R_SB(:,:,i) .* R_BN(:,:,i);
       Fg_S_at_i = R_SN_at_i * Fg_N;
       Fg_S(i,:) = Fg_S_at_i;
    end

    % Transform acceleration from body frame to stability frame
    acc_S = zeros(size(acc_B));
    for i = 1:N
        acc_S_at_i = R_SB(:,:,i) * acc_B(i,:)';
        acc_S(i,:) = acc_S_at_i;
    end

    % Extract aerodynamic forces
    Fa_S = m * acc_S - Fg_S;
    D = -Fa_S(:,1);
    L = -Fa_S(:,3);

    % Calculate coefficients
    rho = 1.225; % kg / m^3
    dynamic_pressure = 0.5 * rho * V_a.^2;
    c_L = L ./ dynamic_pressure;
    c_D = D ./ dynamic_pressure;
end