function [] = plot_maneuver_full(fig_name, t, phi, theta, psi, p, q, r, u, v, w, delta_a, delta_vl, delta_vr, delta_a_sp, delta_vl_sp, delta_vr_sp, n_p, t_pred, y_pred, save_plot, show_plot, plot_location, R_sq)
        V = sqrt(u .^ 2 + v .^ 2 + w .^ 2);

        % Plot
        fig = figure;
        if ~show_plot
            fig.Visible = 'off';
        end
        fig.Position = [100 100 1500 1000];
        num_plots = 9;

        subplot(num_plots,2,1)
        plot(t, rad2deg(phi), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,1)));
        legend("$\phi$", "$\hat{\phi}$");
        ylabel("[deg]")
        ylim([-65 65])
        title("fit = " + R_sq(1) + "%");
        
        subplot(num_plots,2,3)
        plot(t, rad2deg(theta), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,2)));
        legend("$\theta$", "$\hat{\theta}$");
        ylabel("[deg]")
        ylim([-40 40])
        title("fit = " + R_sq(2) + "%");

        subplot(num_plots,2,5)
        plot(t, rad2deg(psi), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,3)));
        legend("$\psi$", "$\hat{\psi}$");
        ylabel("[deg]")
        psi_mean_deg = mean(rad2deg(psi));
        ylim([psi_mean_deg - 50 psi_mean_deg + 50])
        title("fit = " + R_sq(3) + "%");

        subplot(num_plots,2,7)
        plot(t, rad2deg(p), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,4)));
        legend("$p$", "$\hat{p}$");
        ylim([-3.5*180/pi 3.5*180/pi]);
        ylabel("[deg/s]")
        title("fit = " + R_sq(4) + "%");
        
        subplot(num_plots,2,9)
        plot(t, rad2deg(q), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,5)));
        legend("$q$", "$\hat{q}$");
        ylim([-2*180/pi 2*180/pi]);
        ylabel("[deg/s]")
        title("fit = " + R_sq(5) + "%");

        subplot(num_plots,2,11)
        plot(t, rad2deg(r), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,6)));
        ylim([-2*180/pi 2*180/pi]);
        legend("$r$", "$\hat{r}$");
        ylabel("[deg/s]")
        title("fit = " + R_sq(6) + "%");

        subplot(num_plots,2,13)
        plot(t, u, '--'); hold on
        plot(t_pred, y_pred(:,7));
        legend("$u$", "$\hat{u}$")
        ylabel("[m/s]")
        ylim([15 27]);
        title("fit = " + R_sq(7) + "%");
        
        subplot(num_plots,2,15)
        plot(t, v, '--'); hold on
        plot(t_pred, y_pred(:,8));
        ylim([-10 10]);
        legend("$v$", "$\hat{v}$")
        ylabel("[m/s]")
        title("fit = " + R_sq(8) + "%");
        
        subplot(num_plots,2,17)
        plot(t, w, '--'); hold on
        plot(t_pred, y_pred(:,9));
        legend("$w$", "$\hat{w}$")
        ylabel("[m/s]")
        ylim([-5 10]);
        title("fit = " + R_sq(9) + "%");
        
        subplot(num_plots,2,2)
        plot(t, V);
        legend("$V$")
        ylabel("[m/s]")
        ylim([17 24]);

        subplot(num_plots,2,4)
        plot(t, rad2deg(delta_a), '--'); hold on
        plot(t_pred, rad2deg(delta_a_sp));
        legend("$\delta_a$")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,6)
        plot(t, rad2deg(delta_vl), '--'); hold on
        plot(t_pred, rad2deg(delta_vl_sp));
        legend("$\delta_{vl}$")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,8)
        plot(t, rad2deg(delta_vr), '--'); hold on
        plot(t_pred, rad2deg(delta_vr_sp));
        legend("$\delta_{vr}$")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,10)
        plot(t, n_p);
        legend("$n_p$");
        ylabel("[rev/s]");
        ylim([0 130])
        
        
        
        sgtitle(fig_name);
        
        if save_plot
            filename = fig_name;
            mkdir(plot_location);
            saveas(fig, plot_location + filename, 'epsc')
        end
end