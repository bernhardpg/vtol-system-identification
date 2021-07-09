function [] = plot_maneuver_lat(fig_name, t, phi, theta, psi, p, q, r, u, v, w, delta_a, delta_vl, delta_vr, delta_a_sp, delta_vl_sp, delta_vr_sp, n_p, a_x, a_y, a_z, p_dot, q_dot, r_dot, t_pred, y_pred, save_plot, show_plot, plot_location, R_sq)
               
        % Plot
        fig = figure;
        if ~show_plot
            fig.Visible = 'off';
        end
        fig.Position = [100 100 1500 1000];
        num_plots = 9;

        V = sqrt(u .^ 2 + v .^ 2 + w .^ 2);

        delta_e = 0.5 * (delta_vl + delta_vr);
        delta_r = 0.5 * (-delta_vl + delta_vr);
        delta_e_sp = 0.5 * (delta_vl_sp + delta_vr_sp);
        delta_r_sp = 0.5 * (-delta_vl_sp + delta_vr_sp);


        subplot(num_plots,2,1)
        plot(t, rad2deg(phi), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,1)));
        legend("$\phi$", "$\hat{\phi}$")
        ylabel("[deg]")
        ylim([-50 50])
        title("fit = " + R_sq(1) + "%");
        
        subplot(num_plots,2,3)
        plot(t, rad2deg(theta), '--'); hold on
        legend("$\theta$");
        ylabel("[deg]")
        ylim([-30 30])

        subplot(num_plots,2,5)
        plot(t, rad2deg(psi), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,2)));
        legend("$\psi$", "$\hat{\psi}$")
        ylabel("[deg]")
        psi_mean_deg = mean(rad2deg(psi));
        ylim([psi_mean_deg - 50 psi_mean_deg + 50])
        title("fit = " + R_sq(2) + "%");
        
        subplot(num_plots,2,2)
        plot(t, V);
        legend("$V$")
        ylabel("[m/s]")
        ylim([17 24]);

        subplot(num_plots,2,7)
        plot(t, rad2deg(p), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,3)));
        legend("$p$", "$\hat{p}$")
        ylim([-2*180/pi 2*180/pi]);
        ylabel("[deg/s]")
        title("fit = " + R_sq(3) + "%");
        
        subplot(num_plots,2,9)
        plot(t, rad2deg(q), '--'); hold on
        legend("$q$")
        ylim([-2*180/pi 2*180/pi]);
        ylabel("[deg/s]")

        subplot(num_plots,2,11)
        plot(t, rad2deg(r), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,4)));
        legend("$r$", "$\hat{r}$")
        ylim([-2*180/pi 2*180/pi]);
        ylabel("[deg/s]")
        title("fit = " + R_sq(4) + "%");

        subplot(num_plots,2,13)
        plot(t, u, '--'); hold on
        legend("$u$");
        ylabel("[m/s]")
        ylim([15 27]);
        
        subplot(num_plots,2,15)
        plot(t, v, '--'); hold on
        plot(t_pred, y_pred(:,5));
        legend("$v$", "$\hat{v}$")
        ylim([-10 10]);
        ylabel("[m/s]")
        title("fit = " + R_sq(5) + "%");
        
        subplot(num_plots,2,17)
        plot(t, w, '--'); hold on
        legend("$w$")
        ylabel("[m/s]")
        ylim([-5 10]);

        subplot(num_plots,2,4)
        plot(t, rad2deg(delta_a), '--'); hold on
        plot(t_pred, rad2deg(delta_a_sp));
        legend("$\delta_a$")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,6)
        plot(t, rad2deg(delta_e), '--'); hold on
        plot(t_pred, rad2deg(delta_e_sp));
        legend("$\delta_e$")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,8)
        plot(t, rad2deg(delta_r), '--'); hold on
        plot(t_pred, rad2deg(delta_r_sp));
        legend("$\delta_r$")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,10)
        plot(t, n_p);
        legend("$n_p$");
        ylabel("[rev/s]");
        ylim([0 130])
        
        subplot(num_plots,2,12)
        plot(t, a_y, '--'); hold on
        plot(t_pred, y_pred(:,6));
        legend("$a_y$", "$\hat{a_y}$");
        ylabel("[m/s^2]");
        
        subplot(num_plots,2,14)
        plot(t, p_dot, '--'); hold on
        plot(t_pred, y_pred(:,7));
        legend("$\dot{p}$", "$\hat{\dot{p}}$");
        ylabel("[rad/s^2]");
        
        subplot(num_plots,2,16)
        plot(t, r_dot, '--'); hold on
        plot(t_pred, y_pred(:,8))
        legend("$\dot{r}$", "$\hat{\dot{r}}$");
        ylabel("[rad/s^2]");
        

        sgtitle(fig_name);
        
        if save_plot
            filename = fig_name;
            mkdir(plot_location);
            saveas(fig, plot_location + filename, 'epsc')
        end
end