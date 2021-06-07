function [t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a, delta_e, delta_r, n_p]...
    = collect_data_from_all_maneuvers(dt_desired, t_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices,...
    save_maneuver_plot, show_maneuver_plot)

    % Initialize empty variables to contain states
    t = [];
    phi = [];
    theta = [];
    psi = [];
    p = [];
    q = [];
    r = [];
    u = [];
    v = [];
    w = [];
    a_x = [];
    a_y = [];
    a_z = [];
    p_dot = [];
    q_dot = [];
    r_dot = [];
    delta_a = [];
    delta_e = [];
    delta_r = [];
    n_p = [];

    % Iterate through all maneuvers and calculate data
    num_maneuvers = length(maneuver_start_indices);
    num_discarded_maneuvers = 0;
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
        %t_recorded = t_recorded - t_recorded(1); % Make t go from 0 to end
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
        [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, p_dot_m, q_dot_m, r_dot_m] ...
            = calc_states_and_derivs(dt_desired, t_recorded, phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded);

        % Manuever with dropout causes spikes in derivatives
        % Skip these
        error_treshold = 150 * pi / 180;
        if (max(abs(p_m)) > error_treshold) || (max(abs(q_m)) > error_treshold) || (max(abs(r_m)) > error_treshold)
            num_discarded_maneuvers = num_discarded_maneuvers + 1;
            continue;
        end

        % TODO: It is very much possible that this horizon handling needs to be done more accurately
        delta_a_m = interp1(t_recorded, delta_a_recorded, t_m);
        delta_e_m = interp1(t_recorded, delta_e_recorded, t_m);
        delta_r_m = interp1(t_recorded, delta_r_recorded, t_m);
        n_p_m = interp1(t_recorded, n_p_recorded, t_m);

        % Collect data from all maneuvers
        t = [t;
             t_m];
        phi = [phi;
               phi_m];
        theta = [theta;
                 theta_m];
        psi = [psi;
               psi_m];
        p = [p;
             p_m];
        q = [q;
             q_m];
        r = [r;
             r_m];
        u = [u;
             u_m];
        v = [v;
             v_m];
        w = [w;
             w_m];
        a_x = [a_x;
               a_x_m];
        a_y = [a_y;
               a_y_m];
        a_z = [a_z;
               a_z_m];
        p_dot = [p_dot;
                 p_dot_m];
        q_dot = [q_dot;
                 q_dot_m];
        r_dot = [r_dot;
                 r_dot_m];
        delta_a = [delta_a;
                   delta_a_m];
        delta_e = [delta_e;
                   delta_e_m];
        delta_r = [delta_r;
                   delta_r_m];
        n_p = [n_p;
               n_p_m];

        if save_maneuver_plot || show_maneuver_plot
            plot_maneuver("maneuver_" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a, delta_e, delta_r, n_p, ...
                t_recorded, phi_recorded, theta_recorded, psi_recorded, show_maneuver_plot, save_maneuver_plot, plot_location);
        end
    %     
        % Check kinematic consistency
        %check_kinematic_consistency(t, phi, theta, psi, p, q, r, t_recorded, phi_recorded, theta_recorded, psi_recorded);

        % Plot velocities
        %plot_velocity(t, u, v, w, t_recorded, u_recorded, v_recorded, w_recorded);
    end
    
    disp("Discarded " + num_discarded_maneuvers + " maneuvers due to dropout");
end