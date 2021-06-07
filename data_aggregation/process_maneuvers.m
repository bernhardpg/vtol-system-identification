clc; clear all; close all;

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Set data params
dt_desired = 1 / 50;

% Read data recorded from logs
maneuver_type = "pitch_211";
[t_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, u_mr_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices] ...
    = read_experiment_data(metadata, maneuver_type);
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
    u_fw = u_fw_all_maneuvers(maneuver_start_index:maneuver_end_index,:);
    
    eul_recorded = quat2eul(q_NB);
    phi_recorded = eul_recorded(:,3);
    theta_recorded = eul_recorded(:,2);
    psi_recorded = eul_recorded(:,1);
    v_N_recorded = v_NED(:,1);
    v_E_recorded = v_NED(:,2);
    v_D_recorded = v_NED(:,3);
    
    delta_a_recorded = u_fw(:,1);
    delta_e_recorded = u_fw(:,2);
    delta_r_recorded = u_fw(:,3);
    n_p_recorded = u_fw(:,4);
   
    % Calculate all relevant states and their derivatives during the
    % maneuver
    [t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot] ...
        = calc_states_and_derivs(dt_desired, t_recorded, phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded);
    
    % TODO: It is very much possible that this horizon handling needs to be done more accurately
    delta_a = interp1(t_recorded, delta_a_recorded, t);
    delta_e = interp1(t_recorded, delta_e_recorded, t);
    delta_r = interp1(t_recorded, delta_r_recorded, t);
    n_p = interp1(t_recorded, n_p_recorded, t);
    
    plot_maneuver("maneuver_" + maneuver_i, t, phi, theta, psi, p, q, r, u, v, w, delta_a, delta_e, delta_r, n_p);
    
    % TODO: I need to find actual PWM to RPM scale.
    %T = calc_propeller_force(n_p);
    [c_X, c_Y, c_Z] = calc_force_coeffs(u, v, w, a_x, a_y, a_z);
    [c_l, c_m, c_n] = calc_moment_coeffs(p, q, r, u, v, w, p_dot, q_dot, r_dot);
    
    % Check kinematic consistency
    %check_kinematic_consistency(t, phi, theta, psi, p, q, r, t_recorded, phi_recorded, theta_recorded, psi_recorded);
    
    % Plot velocities
    %plot_velocity(t, u, v, w, t_recorded, u_recorded, v_recorded, w_recorded);
end

function [dyn_pressure] = calc_dyn_pressure(u, v, w)
    aircraft_properties; % to get rho

    V = sqrt(u .^ 2 + v .^ 2 + w .^ 2);
    dyn_pressure = 0.5 * rho * V .^ 2;
end

function [T] = calc_propeller_force(n_p)
    aircraft_properties; % to get rho and diam_pusher
    T = rho * prop_diam_pusher ^ 4 * c_T_0_pusher * n_p .^ 2;
end

function [c_X, c_Y, c_Z] = calc_force_coeffs(u, v, w, a_x, a_y, a_z, T)
    dyn_pressure = calc_dyn_pressure(u, v, w);
    aircraft_properties; % get mass and planform
    % TODO: Add thrust here
    c_X = (mass_kg * a_x - 0) ./ (dyn_pressure * planform_sqm);
    c_Y = (mass_kg * a_y) ./ (dyn_pressure * planform_sqm);
    c_Z = (mass_kg * a_z) ./ (dyn_pressure * planform_sqm);
end

function [c_l, c_m, c_n] = calc_moment_coeffs(p, q, r, u, v, w, p_dot, q_dot, r_dot)
    dyn_pressure = calc_dyn_pressure(u, v, w);
    aircraft_properties; % get inertias, wingspan, MAC and planform

    c_l = (Jxx * p_dot - Jxz * (r_dot + p .* q) + q .* r * (Jzz - Jyy)) ./ (dyn_pressure * wingspan_m * planform_sqm);
    c_m = (Jyy * q_dot - r .* p * (Jxx - Jzz) + Jxz * (p .^ 2 - r .^ 2)) ./ (dyn_pressure * mean_aerodynamic_chord_m * planform_sqm);
    c_n = (Jzz * r_dot - Jxz * (p_dot - q .* r) + p .* q * (Jyy - Jxx)) ./ (dyn_pressure * wingspan_m * planform_sqm);
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

function [] = plot_maneuver(fig_name, t, phi, theta, psi, p, q, r, u, v, w, delta_a, delta_e, delta_r, n_p)
        V = sqrt(u .^ 2 + v .^ 2 + w .^ 2);

        % Plot
        show_plot = true;
        fig = figure;
        if ~show_plot
            fig.Visible = 'off';
        end
        fig.Position = [100 100 1500 1000];
        num_plots = 9;

        subplot(num_plots,2,1)
        plot(t, rad2deg(phi)); 
        legend("\phi")
        ylabel("[deg]")
        ylim([-50 50])
        
        subplot(num_plots,2,3)
        plot(t, rad2deg(theta)); 
        legend("\theta")
        ylabel("[deg]")
        ylim([-20 20])

        subplot(num_plots,2,5)
        plot(t, rad2deg(psi)); 
        legend("\psi")
        ylabel("[deg]")
        
        subplot(num_plots,2,2)
        plot(t, V); 
        legend("V")
        ylabel("[m/s]")
        ylim([17 24]);

        subplot(num_plots,2,7)
        plot(t, rad2deg(p));
        legend("p")
        ylabel("[deg/s]")
        
        subplot(num_plots,2,9)
        plot(t, rad2deg(q));
        legend("q")
        ylim([-2*180/pi 2*180/pi]);
        ylabel("[deg/s]")

        subplot(num_plots,2,11)
        plot(t, rad2deg(r));
        legend("r")
        ylabel("[deg/s]")

        subplot(num_plots,2,13)
        plot(t, u);
        legend("u")
        ylabel("[m/s]")
        ylim([15 27]);
        
        subplot(num_plots,2,15)
        plot(t, v);
        legend("v")
        ylabel("[m/s]")
        
        subplot(num_plots,2,17)
        plot(t, w);
        legend("w")
        ylabel("[m/s]")
        ylim([-5 10]);

        subplot(num_plots,2,4)
        plot(t, rad2deg(delta_a));
        legend("\delta_a")
        ylabel("[deg]");
        
        subplot(num_plots,2,6)
        plot(t, rad2deg(delta_e));
        legend("\delta_e")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,8)
        plot(t, rad2deg(delta_r));
        legend("\delta_r")
        ylabel("[deg]");
        
        subplot(num_plots,2,10)
        plot(t, n_p);
        legend("n_p");
        ylabel("[rev/s]");
        
        sgtitle(fig_name);
end
