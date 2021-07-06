function [] = evaluate_full_model(maneuvers_to_test, maneuver_types, x, save_plot, show_plot, ...
    maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_vl_sp, delta_vr_sp, delta_a, delta_vl, delta_vr, n_p)          
    
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
    plot_output_location = "model_identification/model_validation/validation_plots/full_model/";
    
    load_const_params;
    
    all_params = [const_params;
                  x'];

    t_seq = t;
    dt = t(2) - t(1);
    input_seq = [delta_a, delta_vl, delta_vr, n_p]; % Actuator dynamics simulated beforehand

    num_maneuvers = length(maneuvers_to_test);
    R_sq = zeros(num_maneuvers, 4);
    for maneuver_i = maneuvers_to_test
        % Get data for desired maneuver
        [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, delta_a_m, delta_vl_m, delta_vr_m, n_p_m]...
            = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_vl_sp, delta_vr_sp, delta_a, delta_vl, delta_vr, n_p);
        input_seq_m = [t_m delta_a_m delta_vl_m delta_vr_m n_p_m];

        % Integrate dynamics
        y0 = [phi_m(1) theta_m(1) psi_m(1) p_m(1) q_m(1) r_m(1) u_m(1) v_m(1) w_m(1)];
        tspan = [t_m(1) t_m(end)];

        % Integrate dynamics
        [t_pred, y_pred] = ode45(@(t,y) full_dynamics_liftdrag_c(t, y, input_seq_m, all_params), tspan, y0);
        y_pred = interp1(t_pred, y_pred, t_m);

        R_sq_phi = calc_R_sq(phi_m, y_pred(:,1));
        R_sq_theta = calc_R_sq(theta_m, y_pred(:,2));
        R_sq_psi = calc_R_sq(psi_m, y_pred(:,3));
        R_sq_p = calc_R_sq(p_m, y_pred(:,4));
        R_sq_q = calc_R_sq(q_m, y_pred(:,5));
        R_sq_r = calc_R_sq(r_m, y_pred(:,6));
        R_sq_u = calc_R_sq(u_m, y_pred(:,7));
        R_sq_v = calc_R_sq(v_m, y_pred(:,8));
        R_sq_w = calc_R_sq(w_m, y_pred(:,9));
        R_sq_m = [R_sq_phi R_sq_theta R_sq_psi R_sq_p R_sq_q R_sq_r R_sq_u R_sq_v R_sq_w];
        
        if save_plot || show_plot
            plot_maneuver_full("val_maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_vl_m, delta_vr_m,  delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, n_p_m,...
                t_m, y_pred,...
                save_plot, show_plot, plot_output_location, R_sq_m);
        end
    end
end


function [R_sq] = calc_R_sq(z, y_hat)
    % Calculate total Sum of Squares
    z_bar = mean(z);
    SS_T = (z - z_bar)' * (z - z_bar);
    SS_E = (z - y_hat)' * (z - y_hat); % Residual Sum of Squares
    
    % Coefficient of Determination
    R_sq = (1 - SS_E/SS_T) * 100;
end
