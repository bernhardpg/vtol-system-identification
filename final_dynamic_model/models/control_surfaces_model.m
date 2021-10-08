function delta_dot = control_surfaces_model(t, delta, calc_delta_sp)
    % Import parameters
    static_parameters;

    % Unpack variables
    delta = num2cell(delta);
    [delta_a, delta_e, delta_r] = delta{:};
    delta_sp = calc_delta_sp(t);
    delta_sp = num2cell(delta_sp);
    [delta_a_sp, delta_e_sp, delta_r_sp] = delta_sp{:};

    % Calculate control surfaces
    delta_a_dot = bound(-delta_a / T_servo + delta_a_sp / T_servo, -delta_rate_lim, delta_rate_lim);
    delta_e_dot = bound(-delta_e / T_servo + delta_e_sp / T_servo, -delta_rate_lim, delta_rate_lim);
    delta_r_dot = bound(-delta_r / T_servo + delta_r_sp / T_servo, -delta_rate_lim, delta_rate_lim);
    
    delta_dot = [delta_a_dot delta_e_dot delta_r_dot]';
end

function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y = min(max(x,bl),bu);
end