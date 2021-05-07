function [c_m, tau_y] = calculate_pitch_moment(state, V_a, ang_acc, lam_5, lam_6, Jyy, rho)
    p = state(:,5);
    q = state(:,6);
    r = state(:,7);
    q_dot = ang_acc(:,2);
    
    % Pitch moment curve
    temp = lam_5 * p .* r - lam_6 * (p .^ 2 - r .^ 2);
    tau_y = (q_dot - temp) * Jyy;

    % Calculate coefficients
    dynamic_pressure = 0.5 * rho * V_a.^2;
    c_m = tau_y ./ dynamic_pressure;
end