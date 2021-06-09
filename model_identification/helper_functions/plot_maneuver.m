function [] = plot_maneuver(fig_name, t, phi, theta, psi, p, q, r, u, v, w, delta_a, delta_e, delta_r, n_p, t_pred, y_pred, save_plot, show_plot, plot_location)
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
        %plot(t_pred, rad2deg(y_pred(:,1)));
        legend("\phi")
        ylabel("[deg]")
        ylim([-50 50])
        
        subplot(num_plots,2,3)
        plot(t, rad2deg(theta), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,1)));
        legend("\theta")
        ylabel("[deg]")
        ylim([-30 30])

        subplot(num_plots,2,5)
        plot(t, rad2deg(psi), '--'); hold on
        %plot(t_pred, rad2deg(y_pred(:,3)));
        legend("\psi")
        ylabel("[deg]")
        psi_mean_deg = mean(rad2deg(psi));
        ylim([psi_mean_deg - 50 psi_mean_deg + 50])
        
        subplot(num_plots,2,2)
        plot(t, V);
        legend("V")
        ylabel("[m/s]")
        ylim([17 24]);

        subplot(num_plots,2,7)
        plot(t, rad2deg(p), '--'); hold on
        %plot(t_pred, rad2deg(y_pred(:,4)));
        legend("p")
        ylim([-2*180/pi 2*180/pi]);
        ylabel("[deg/s]")
        
        subplot(num_plots,2,9)
        plot(t, rad2deg(q), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,2)));
        legend("q")
        ylim([-2*180/pi 2*180/pi]);
        ylabel("[deg/s]")

        subplot(num_plots,2,11)
        plot(t, rad2deg(r), '--'); hold on
        %plot(t_pred, rad2deg(y_pred(:,6)));
        ylim([-2*180/pi 2*180/pi]);
        legend("r")
        ylabel("[deg/s]")

        subplot(num_plots,2,13)
        plot(t, u, '--'); hold on
        plot(t_pred, y_pred(:,3));
        legend("u")
        ylabel("[m/s]")
        ylim([15 27]);
        
        subplot(num_plots,2,15)
        plot(t, v, '--'); hold on
        %plot(t_pred, y_pred(:,7));
        ylim([-5 5]);
        legend("v")
        ylabel("[m/s]")
        
        subplot(num_plots,2,17)
        plot(t, w, '--'); hold on
        plot(t_pred, y_pred(:,4));
        legend("w")
        ylabel("[m/s]")
        ylim([-5 10]);

        subplot(num_plots,2,4)
        plot(t, rad2deg(delta_a));
        legend("\delta_a")
        ylabel("[deg]");
        ylim([-28 28])
        
        subplot(num_plots,2,6)
        plot(t, rad2deg(delta_e), '--'); hold on
        plot(t_pred, rad2deg(y_pred(:,5)));
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