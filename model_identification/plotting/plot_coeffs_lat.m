function [] = plot_coeffs_lat(fig_name, x_lat, t, phi, theta, psi, p, q, r, u, v, w, delta_a, delta_vl, delta_vr, delta_a_sp, delta_vl_sp, delta_vr_sp, n_p, a_x, a_y, a_z, p_dot, q_dot, r_dot, t_pred, y_pred, save_plot, show_plot, plot_location, R_sq)    
    % Plot
    fig = figure;
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1500 1000];

    delta_e = 0.5 * (delta_vl + delta_vr);
    delta_r = 0.5 * (-delta_vl + delta_vr);
    delta_e_sp = 0.5 * (delta_vl_sp + delta_vr_sp);
    delta_r_sp = 0.5 * (-delta_vl_sp + delta_vr_sp);

    aoa_alpha = atan2(w, u);
    V = sqrt(u.^2 + v.^2 + w.^2);
    beta = asin(v ./ V);
    [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
    [c_l, c_Y, c_n] = calc_coeffs_lat(x_lat, beta, p_hat, r_hat, delta_a, delta_r);
    
    phi_pred = y_pred(:,1);
    psi_pred = y_pred(:,2);
    p_pred = y_pred(:,3);
    r_pred = y_pred(:,4);
    v_pred = y_pred(:,5);
    V_pred = sqrt(u.^2 + v_pred.^2 + w.^2);
    [p_hat_pred, q_hat_pred, r_hat_pred, ~, ~, ~] = calc_explanatory_vars(p_pred, q, r_pred, u, v_pred, w);
    beta_pred = asin(v_pred ./ V_pred);
    [c_l_pred, c_Y_pred, c_n_pred] = calc_coeffs_lat(x_lat, beta_pred, p_hat_pred, r_hat_pred, delta_a, delta_r);
    
    subplot(3,1,1)
    plot(t, c_l_pred); hold on
    plot(t, c_l, '--');
    legend("$\hat{c}_l$", "$c_l$");
    
    subplot(3,1,2)
    plot(t, c_Y_pred); hold on
    plot(t, c_Y, '--');
    legend("$\hat{c}_Y$", "$c_Y$");
    
    subplot(3,1,3)
    plot(t, c_n_pred); hold on
    plot(t, c_n, '--');
    legend("$\hat{c}_n$", "$c_n$");
    
    sgtitle(fig_name);

    if save_plot
        filename = fig_name;
        mkdir(plot_location);
        saveas(fig, plot_location + filename, 'epsc')
    end
end