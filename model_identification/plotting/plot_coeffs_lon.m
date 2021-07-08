function [] = plot_coeffs_lon(fig_name, x_lon, t, phi, theta, psi, p, q, r, u, v, w, delta_a, delta_vl, delta_vr, delta_a_sp, delta_vl_sp, delta_vr_sp, n_p, a_x, a_y, a_z, p_dot, q_dot, r_dot, t_pred, y_pred, save_plot, show_plot, plot_location, R_sq)    
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
    [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
    [c_D, c_L, c_m] = calc_coeffs(x_lon, aoa_alpha, q_hat, delta_e);
    
    q_pred = y_pred(:,2);
    u_pred = y_pred(:,3);
    w_pred = y_pred(:,4);
    [~, q_hat_pred, ~, ~, ~, ~] = calc_explanatory_vars(p, q_pred, r, u_pred, v, w_pred);
    aoa_alpha_pred = atan2(w_pred, u_pred);
    [c_D_pred, c_L_pred, c_m_pred] = calc_coeffs(x_lon, aoa_alpha_pred, q_hat_pred, delta_e);
    
    subplot(3,1,1)
    plot(t, c_D_pred); hold on
    plot(t, c_D, '--');
    legend("$\hat{c}_D$", "$c_D$");
    
    subplot(3,1,2)
    plot(t, c_L_pred); hold on
    plot(t, c_L, '--');
    legend("$\hat{c}_L$", "$c_L$");
    
    subplot(3,1,3)
    plot(t, c_m_pred); hold on
    plot(t, c_m, '--');
    legend("$\hat{c}_m$", "$c_m$");
    
    sgtitle(fig_name);

    if save_plot
        filename = fig_name;
        mkdir(plot_location);
        saveas(fig, plot_location + filename, 'epsc')
    end
end