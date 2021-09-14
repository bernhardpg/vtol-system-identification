function [L, D, c_L, c_D, Fa_B] = calculate_lift_and_drag(state, input, AoA_rad, V_a, acc_B, m, g)
    % Read data
    aircraft_properties;
    
    N = length(state);
    q_NB = state(:,1:4);
    eul = quat2eul(q_NB);
    psi = eul(:,1);
    theta = eul(:,2);
    phi = eul(:,3);
    
    % Create rotation matrices
    q_BN = quatinv(q_NB);
    R_BN = quat2rotm(q_BN);
    
    % Calculate gravitational force in measurement frame
    g_N = [0; 0; g]; % NED frame
    
    % Rotate gravity to body frame
    g_B = zeros(N,3); % Measurement frame
    g_B2 = zeros(N,3); % Measurement frame
    for i = 1:N
       % g_B(i,:) = R_BN(:,:,i) * g_N;
       % Same as:
       g_B(i,:) = [-g * sin(theta(i));
                    g * cos(theta(i)) * sin(phi(i));
                    g * cos(theta(i)) * cos(phi(i))];
    end
    
    
    % Calculate total aerodynamic forces
    % (Assumes no other forces present, i.e. no motor use)
    Fa_B = m * (acc_B - g_B);

    % Rotate aerodynamic forces to stability frame
    R_SB = eul2rotm([zeros(size(AoA_rad)) AoA_rad zeros(size(AoA_rad))]);
    
    % Rotate gravity to body frame
    Fa_S = zeros(N,3); % Measurement frame
    for i = 1:N
       Fa_S(i,:) = R_SB(:,:,i) * Fa_B(i,:)';
    end
    
    D = -Fa_S(:,1);
    L = -Fa_S(:,3);

    % Calculate coefficients
    rho = 1.225; % kg / m^3
    dynamic_pressure = 0.5 * rho * V_a.^2;
    c_L = L ./ (dynamic_pressure * planform_sqm);
    c_D = D ./ (dynamic_pressure * planform_sqm);
end