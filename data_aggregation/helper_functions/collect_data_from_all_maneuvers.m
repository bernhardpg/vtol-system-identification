function [t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a, delta_e, delta_r, n_p]...
    = collect_data_from_all_maneuvers(dt_desired, t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw,...
    save_maneuver_plot, show_maneuver_plot, plot_location)

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
    num_maneuvers = length(maneuver_start_indices_state);
    num_discarded_maneuvers = 0;
    for maneuver_i = 1:num_maneuvers
        % Get correct maneuver start and end index
        maneuver_start_index_state = maneuver_start_indices_state(maneuver_i);
        if maneuver_i == num_maneuvers
            maneuver_end_index_state = length(t_state_all_maneuvers);
        else
            maneuver_end_index_state = maneuver_start_indices_state(maneuver_i + 1) - 1;
        end            

        % Extract recorded data during maneuver
        t_recorded = t_state_all_maneuvers(maneuver_start_index_state:maneuver_end_index_state,:);
        %t_recorded = t_recorded - t_recorded(1); % Make t go from 0 to end
        q_NB = q_NB_all_maneuvers(maneuver_start_index_state:maneuver_end_index_state,:);
        v_NED = v_NED_all_maneuvers(maneuver_start_index_state:maneuver_end_index_state,:);

        eul_recorded = quat2eul(q_NB);
        phi_recorded = eul_recorded(:,3);
        theta_recorded = eul_recorded(:,2);
        psi_recorded = eul_recorded(:,1);
        v_N_recorded = v_NED(:,1);
        v_E_recorded = v_NED(:,2);
        v_D_recorded = v_NED(:,3);

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

        % Handle inputs which are on their own timestamps and start indices
        maneuver_start_index_u_fw = maneuver_start_indices_u_fw(maneuver_i);
        if maneuver_i == num_maneuvers
            maneuver_end_index_u_fw = length(t_u_fw_all_maneuvers);
        else
            maneuver_end_index_u_fw = maneuver_start_indices_u_fw(maneuver_i + 1) - 1;
        end
        
        t_u_fw_m = t_u_fw_all_maneuvers(maneuver_start_index_u_fw:maneuver_end_index_u_fw,:);
        
        u_fw_m = u_fw_all_maneuvers(maneuver_start_index_u_fw:maneuver_end_index_u_fw,:);
        delta_a_recorded = u_fw_m(:,1);
        delta_e_recorded = u_fw_m(:,2);
        delta_r_recorded = u_fw_m(:,3);
        n_p_recorded = u_fw_m(:,4);

        delta_a_m = interp1(t_u_fw_m, delta_a_recorded, t_m);
        delta_e_m = interp1(t_u_fw_m, delta_e_recorded, t_m);
        delta_r_m = interp1(t_u_fw_m, delta_r_recorded, t_m);
        n_p_m = interp1(t_u_fw_m, n_p_recorded, t_m);

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
            plot_maneuver("maneuver_" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_e_m, delta_r_m, n_p_m, t_recorded, phi_recorded, theta_recorded, psi_recorded, save_maneuver_plot, show_maneuver_plot, plot_location)
        end
    %     
        % Check kinematic consistency
        %check_kinematic_consistency(t, phi, theta, psi, p, q, r, t_recorded, phi_recorded, theta_recorded, psi_recorded);

        % Plot velocities
        %plot_velocity(t, u, v, w, t_recorded, u_recorded, v_recorded, w_recorded);
    end
    
    disp("Discarded " + num_discarded_maneuvers + " maneuvers due to dropout");
end

function [] = plot_maneuver(fig_name, t, phi, theta, psi, p, q, r, u, v, w, delta_a, delta_e, delta_r, n_p, t_recorded, phi_recorded, theta_recorded, psi_recorded, save_plot, show_plot, plot_location)

        V = sqrt(u .^ 2 + v .^ 2 + w .^ 2);

        % Plot
        fig = figure;
        if ~show_plot
            fig.Visible = 'off';
        end
        fig.Position = [100 100 1500 1000];
        num_plots = 9;

        subplot(num_plots,2,1)
        plot(t, rad2deg(phi), t_recorded, rad2deg(phi_recorded), '--'); 
        legend("\phi", "\phi (recorded)")
        ylabel("[deg]")
        ylim([-50 50])
        
        subplot(num_plots,2,3)
        plot(t, rad2deg(theta), t_recorded, rad2deg(theta_recorded), '--'); 
        legend("\theta", "\theta (recorded)")
        ylabel("[deg]")
        ylim([-30 30])

        subplot(num_plots,2,5)
        plot(t, rad2deg(psi), t_recorded, rad2deg(psi_recorded), '--'); 
        legend("\psi", "\psi (recorded)")
        ylabel("[deg]")
        psi_mean_deg = mean(rad2deg(psi));
        ylim([psi_mean_deg - 50 psi_mean_deg + 50])
        
        subplot(num_plots,2,2)
        plot(t, V); 
        legend("V")
        ylabel("[m/s]")
        ylim([17 24]);

        subplot(num_plots,2,7)
        plot(t, rad2deg(p));
        legend("p")
        ylim([-2*180/pi 2*180/pi]);
        ylabel("[deg/s]")
        
        subplot(num_plots,2,9)
        plot(t, rad2deg(q));
        legend("q")
        ylim([-2*180/pi 2*180/pi]);
        ylabel("[deg/s]")

        subplot(num_plots,2,11)
        plot(t, rad2deg(r));
        ylim([-2*180/pi 2*180/pi]);
        legend("r")
        ylabel("[deg/s]")

        subplot(num_plots,2,13)
        plot(t, u);
        legend("u")
        ylabel("[m/s]")
        ylim([15 27]);
        
        subplot(num_plots,2,15)
        plot(t, v);
        ylim([-5 5]);
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
        ylim([-28 28])
        
        subplot(num_plots,2,6)
        plot(t, rad2deg(delta_e));
        legend("\delta_e")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,8)
        plot(t, rad2deg(delta_r));
        legend("\delta_r")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,10)
        plot(t, n_p);
        legend("n_p");
        ylabel("[rev/s]");
        ylim([0 130])
        
        sgtitle(fig_name);
        
        if save_plot
            filename = fig_name;
            mkdir(plot_location);
            saveas(fig, plot_location + filename, 'epsc')
        end
end