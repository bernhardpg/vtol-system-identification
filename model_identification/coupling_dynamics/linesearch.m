clc; clear all; close all;

%%% Crude line search to find the optimal value for c_m_delta_r_sq, which
%%% minimizes the squared error on the training data

%%% NOTE: This script does not work when c_m_delta_r_sq is defined in
%%% aerodynamic_coeffs.m

load("data/flight_data/selected_data/fpr_data_lat.mat");
fpr_data = fpr_data_lat;
num_maneuvers = numel(fpr_data_lat.training.yaw_211);

% Initial guess from OLS
c_m_delta_r_sq_initial = -1.494508143521402;
range = [-2 2];
c_m_delta_r_sq_candidates = linspace(range(1), range(2), 20);

costs = inf(length(c_m_delta_r_sq_candidates),1);
for c_m_i = 1:length(c_m_delta_r_sq_candidates)
    c_m_delta_r_sq = c_m_delta_r_sq_candidates(c_m_i);
    disp("c_m_delta_r_sq = " + c_m_delta_r_sq)
    y_sim_collected = [];
    y_rec_collected = [];
    for maneuver_i = 1:num_maneuvers
        disp("   maneuver_i: " + maneuver_i)
        maneuver = fpr_data.training.yaw_211(maneuver_i);

        % Prepare recorded data %
        t_seq = maneuver.Time();
        input_seq = maneuver.get_input_sp_sequence();
        y_0 = [maneuver.get_state_initial() maneuver.get_input_initial()];
        y_recorded = maneuver.get_state_sequence();
        tspan = [t_seq(1) t_seq(end)];

        % Simulate nonlinear model with real input setpoints %
        [t_sim, y_sim] = ode45(@(t,y) nonlinear_aircraft_model(t, y, @(t) calc_input_at_t(t, t_seq, input_seq), c_m_delta_r_sq), tspan, y_0);
        y_sim = interp1(t_sim, y_sim, t_seq);
        y_sim = y_sim(:,1:8); % do not use actuator dynamics

        y_sim_collected = [y_sim_collected;
                           y_sim];

        y_rec_collected = [y_rec_collected;
                           y_recorded];
    end
    
    residuals = y_sim_collected - y_rec_collected;
    cost = sum(diag(residuals'*residuals));
    costs(c_m_i) = cost;
end

[c,i] = min(costs);
c_m_delta_r_sq = c_m_delta_r_sq_candidates(i);

plot_settings;
plot(c_m_delta_r_sq_candidates, costs); hold on
scatter(c_m_delta_r_sq, c);
ylabel("Cost", 'interpreter','latex','FontSize',font_size)
xlabel("$c_{m_{\delta_r}^2}$", 'interpreter', 'latex', 'FontSize', font_size)
title("Line Search for Coupling Parameter", 'FontSize', font_size_large, 'interpreter', 'latex')

% plot_maneuver(t_sim, y_sim, t_seq, y_recorded, input_seq);

%% Helper functions
function input_at_t = calc_input_at_t(t, t_seq, input_seq)
    % Roll index forward until we get to approx where we should get
    % inputs from. This basically implements zeroth-order hold for
    % the input
    curr_index_data_seq = 1;
    while t_seq(curr_index_data_seq) < t
       curr_index_data_seq = curr_index_data_seq + 1;
    end
    
    % Get input at t
    input_at_t = input_seq(curr_index_data_seq,:);
end

function x_dot = nonlinear_aircraft_model(t, x, calc_input, c_m_delta_r_sq)
    % Import parameters
    aerodynamic_coeffs;
    static_parameters;
    trim_values;

    % Unpack states
    x = num2cell(x);
    [u, v, w, p, q, r, phi, theta, delta_a, delta_e, delta_r] = x{:};
   
    % Unpack inputs
    input = calc_input(t);
    input = num2cell(input);
    [delta_a_sp, delta_e_sp, delta_r_sp, delta_t] = input{:};

    % Calculate control surfaces
    delta_a_dot = bound(-delta_a / T_servo + delta_a_sp / T_servo, -delta_rate_lim, delta_rate_lim);
    delta_e_dot = bound(-delta_e / T_servo + delta_e_sp / T_servo, -delta_rate_lim, delta_rate_lim);
    delta_r_dot = bound(-delta_r / T_servo + delta_r_sp / T_servo, -delta_rate_lim, delta_rate_lim);

    % Model is around perturbation control surface deflections
    delta_e_pert = (delta_e - delta_e_trim);
    delta_a_pert = (delta_a - delta_a_trim);

    % Aerodynamics
    V = sqrt(u^2 + v^2 + w^2);
    q_bar = (1/2) * rho * V^2;

    alpha = atan(w/u);
    beta = asin(v/V);

    % Nondimensionalize rates
    p_hat = b * p / (2 * V_trim);
    q_hat = c_bar * q / (2 * V_trim);
    r_hat = b * r / (2 * V_trim);

    % Calculate aerodynamic coefficients
    c_D = c_D_0 + c_D_alpha * alpha + c_D_alpha_sq * alpha^2 + c_D_q_hat * q_hat + c_D_delta_e * delta_e_pert + c_D_delta_e_alpha * alpha * delta_e_pert;
    c_L = c_L_0 + c_L_alpha * alpha + c_L_alpha_sq * alpha^2 + c_L_delta_e * delta_e_pert;
    c_m = c_m_0 + c_m_alpha * alpha + c_m_q_hat * q_hat + c_m_delta_e * delta_e_pert;
    
    % Add coupling effect from rudder to pitch
    c_m = c_m + c_m_delta_r_sq * delta_r.^2;

    c_Y = c_Y_0 + c_Y_beta * beta + c_Y_p_hat * p_hat + c_Y_delta_a * delta_a_pert + c_Y_delta_r * delta_r;
    c_l = c_l_0 + c_l_beta * beta + c_l_p_hat * p_hat + c_l_r_hat * r_hat + c_l_delta_a * delta_a_pert;
    c_n = c_n_0 + c_n_beta * beta + c_n_p_hat * p_hat + c_n_r_hat * r_hat + c_n_delta_r * delta_r;

    % Calculate forces and moments
    D = q_bar * S * c_D;
    L = q_bar * S * c_L;
    m = q_bar * S * c_bar * c_m;
    X = -cos(alpha) * D + sin(alpha) * L;
    Z = -sin(alpha) * D - cos(alpha) * L;

    Y = q_bar * S * c_Y;
    l = q_bar * S * b * c_l;
    n = q_bar * S * b * c_n;

    % Propeller force
    T = rho * D_FW^4 * c_T_FW * delta_t;

    % Dynamics
    phi_dot = p + tan(theta) * (q * sin(phi) + r * cos(phi));
    theta_dot = q * cos(phi) - r * sin(phi);

    f_x = X + T - mass*g * sin(theta);
    f_y = Y + mass*g * sin(phi) * cos(theta);
    f_z = Z + mass*g * cos(phi) * cos(theta);

    u_dot = r*v - q*w + (1/mass) * (f_x);
    v_dot = p*w - r*u + (1/mass) * (f_y);
    w_dot = q*u - p*v + (1/mass) * (f_z);

    p_dot = gam(1)*p*q - gam(2)*q*r + gam(3)*l + gam(4)*n;
    q_dot = gam(5)*p*r - gam(6)*(p^2 - r^2) + (1/J_yy) * m;
    r_dot = gam(7)*p*q - gam(1)*q*r + gam(4)*l + gam(8)*n;
    
    x_dot = [u_dot v_dot w_dot p_dot q_dot r_dot phi_dot theta_dot delta_a_dot delta_e_dot delta_r_dot]';
end

function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y = min(max(x,bl),bu);
end