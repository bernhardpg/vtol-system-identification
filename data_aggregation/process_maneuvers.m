clc; clear all; close all;

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Set data params
dt_desired = 1 / 50;

% Read data recorded from logs
maneuver_type = "pitch_211";
[t_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, maneuver_start_indices] = read_experiment_data(metadata, maneuver_type);
num_maneuvers = length(maneuver_start_indices);

% Iterate through all maneuvers and calculate data
for maneuver_i = 1:num_maneuvers
    % Get correct maneuver start and end index
    maneuver_start_index = maneuver_start_indices(maneuver_i);
    if maneuver_i == num_maneuvers
        maneuver_end_index = length(t_all_maneuvers);
    else
        maneuver_end_index = maneuver_start_indices(maneuver_i + 1) - 1;
    end            

    % Extract recorded data during maneuver
    t_recorded = t_all_maneuvers(maneuver_start_index:maneuver_end_index,:);
    t_recorded = t_recorded - t_recorded(1); % Make t go from 0 to end
    q_NB = q_NB_all_maneuvers(maneuver_start_index:maneuver_end_index,:);
    v_NED = v_NED_all_maneuvers(maneuver_start_index:maneuver_end_index,:);
    
    eul_recorded = quat2eul(q_NB);
    phi_recorded = eul_recorded(:,3);
    theta_recorded = eul_recorded(:,2);
    psi_recorded = eul_recorded(:,1);
    
    v_N_recorded = v_NED(:,1);
    v_E_recorded = v_NED(:,2);
    v_D_recorded = v_NED(:,3);
    
    % Calculate all relevant states and their derivatives during the
    % maneuver
    [phi, theta, psi, p, q, r, u, v, w, u_dot, v_dot, w_dot] ...
        = calc_states_and_derivs(dt_desired, t_recorded, phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded);
    
    % Check kinematic consistency
    %check_kinematic_consistency(t, phi, theta, psi, p, q, r, t_recorded, phi_recorded, theta_recorded, psi_recorded);
    
    % Plot velocities
    %plot_velocity(t, u, v, w, t_recorded, u_recorded, v_recorded, w_recorded);
end

function [phi, theta, psi, p, q, r, u, v, w, u_dot, v_dot, w_dot] = ...
    calc_states_and_derivs(dt_desired, t_recorded, phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded)
    
    t_0 = t_recorded(1);
    t_end = t_recorded(end);
    
    % Calc u, v and w from kinematic relationships
    [u_recorded, v_recorded, w_recorded] = calc_uvw(phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded);
    
    % Smooth signals
    temp = smooth_signal([phi_recorded theta_recorded psi_recorded]);
    phi = temp(:,1);
    theta = temp(:,2);
    psi = temp(:,3);
    
    temp = smooth_signal([u_recorded v_recorded w_recorded]);
    u = temp(:,1);
    v = temp(:,2);
    w = temp(:,3);
    
    % Calculate piecewise spline approximations of signals
    phi_spline = slmengine(t_recorded, phi,'knots',t_0:.1:t_end + 0.1); % add 'plot', 'on' to see fit
    theta_spline = slmengine(t_recorded, theta,'knots',t_0:.1:t_end + 0.1);
    psi_spline = slmengine(t_recorded, psi,'knots',t_0:.1:t_end + 0.1);
        
    u_spline = slmengine(t_recorded, u,'knots',t_0:.1:t_end + 0.1, 'plot', 'off');
    v_spline = slmengine(t_recorded, v,'knots',t_0:.1:t_end + 0.1, 'plot', 'off');
    w_spline = slmengine(t_recorded, w,'knots',t_0:.1:t_end + 0.1, 'plot', 'off');
    
    % Set desired time vector
    t = (0:dt_desired:t_end)';
    
    % Calculate derivatives and attitude angles at desired times
    phi = slmeval(t, phi_spline);
    phi_dot = slmeval(t, phi_spline, 1);
    theta = slmeval(t, theta_spline);
    theta_dot = slmeval(t, theta_spline, 1);
    psi = slmeval(t, psi_spline);
    psi_dot = slmeval(t, psi_spline, 1);
    
    u = slmeval(t, u_spline);
    u_dot = slmeval(t, u_spline, 1);
    v = slmeval(t, v_spline);
    v_dot = slmeval(t, v_spline, 1);
    w = slmeval(t, w_spline);
    w_dot = slmeval(t, w_spline, 1);
    
    % Calc p, q and r from kinematic relationships
    [p, q, r] = calc_pqr(phi, theta, psi, phi_dot, theta_dot, psi_dot);
end

function [x_smoothed] = smooth_signal(x)
    order = 10;
    framelen = 31;
    x_smoothed = sgolayfilt(x,order,framelen);
end

function [] = check_kinematic_consistency(t, phi, theta, psi, p, q, r, t_recorded, phi_recorded, theta_recorded, psi_recorded)
    y0 = [phi(1) theta(1) psi(1)];
    tspan = [t(1) t(end)];
    [t_sim, y] = ode23s(@(t_sim,y) rot_kinematics(t_sim, y, t, [p q r]), tspan, y0);
    
    phi_pred = y(:,1);
    theta_pred = y(:,2);
    psi_pred = y(:,3);
    
    plot_attitude(t, phi, theta, psi, t_sim, phi_pred, theta_pred, psi_pred, t_recorded, phi_recorded, theta_recorded, psi_recorded)
end

function [p, q, r] = calc_pqr(phi, theta, psi, phi_dot, theta_dot, psi_dot)
    p = phi_dot - psi_dot .* sin(theta);
    q = theta_dot .* cos(phi) + psi_dot .* sin(phi) .* cos(theta);
    r = psi_dot .* cos(phi) .* cos(theta) - theta_dot .* sin(phi);
end

function [u, v, w] = calc_uvw(phi, theta, psi, v_N, v_E, v_D)
    u = cos(theta) .* cos(psi) .* v_N + cos(theta) .* sin(psi) .* v_E - sin(theta) .* v_D;
    v = (cos(psi) .* sin(theta) .* sin(phi) - cos(phi) .* sin(psi)) .* v_N ...
        + (cos(phi) .* cos(psi) + sin(theta) .* sin(phi) .* sin(psi)) .* v_E ...
        + cos(theta) .* sin(phi) .* v_D;
    w = (cos(psi) .* sin(theta) .* cos(phi) + sin(phi) .* sin(psi)) .* v_N ...
        + (sin(theta) .* cos(phi) .* sin(psi) - sin(phi) .* cos(psi)) .* v_E ...
        + cos(theta) .* cos(phi) .* v_D;
end

function [dy_dt] = rot_kinematics(t, y, t_u, u)
    phi = y(1);
    theta = y(2);
    psi = y(2);
    
    p = interp1(t_u, u(:,1), t);
    q = interp1(t_u, u(:,2), t);
    r = interp1(t_u, u(:,3), t);

    phi_dot = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot = (q * sin(phi) + r * cos(phi)) * sec(theta);
    
    dy_dt = [phi_dot theta_dot psi_dot]';
end

function [] = plot_attitude(t, phi, theta, psi, t_pred, phi_pred, theta_pred, psi_pred, t_recorded, phi_recorded, theta_recorded, psi_recorded)
    figure
    subplot(3,1,1)
    plot(t, rad2deg(phi), t_pred, rad2deg(phi_pred), t_recorded, rad2deg(phi_recorded), '--r')
    legend("\phi", "\phi (predicted)", "\phi (recorded)")
    
    subplot(3,1,2)
    plot(t, rad2deg(theta), t_pred, rad2deg(theta_pred), t_recorded, rad2deg(theta_recorded), '--r')
    legend("\theta", "\theta (predicted)", "\theta (recorded)")
    
    subplot(3,1,3)
    plot(t, rad2deg(psi), t_pred, rad2deg(psi_pred), t_recorded, rad2deg(psi_recorded), '--r')
    legend("\psi", "\psi (predicted)", "\psi (recorded)")
end

function [] = plot_velocity(t, u, v, w, t_recorded, u_recorded, v_recorded, w_recorded)
    figure
    subplot(3,1,1)
    plot(t, u, t_recorded, u_recorded, '--r')
    legend("u", "u (recorded)")
    
    subplot(3,1,2)
    plot(t, v, t_recorded, v_recorded, '--r')
    legend("v", "v (recorded)")
    
    subplot(3,1,3)
    plot(t, w, t_recorded, w_recorded, '--r')
    legend("w", "w (recorded)")
end
