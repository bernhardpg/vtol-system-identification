function [] = plot_maneuver(maneuver_index, t, state, input, show_plot, save_plot, plot_output_location)
    % Read measured state
    e0 = state(:,1);
    e1 = state(:,2);
    e2 = state(:,3);
    e3 = state(:,4);
    p = state(:,5);
    q = state(:,6);
    r = state(:,7);
    u = state(:,8);
    v = state(:,9);
    w = state(:,10);
    
    V_a = sqrt(u .^ 2 + v .^ 2 + w .^ 2);

    quat = [e0 e1 e2 e3];
    eul = quat2eul(quat);
    yaw = eul(:,1);
    pitch = eul(:,2);
    roll = eul(:,3);
    
    alpha = atan2(w, u);
    beta = asin(v ./ V_a);

    % Plot
    fig = figure;
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1500 1500];
    num_plots = 17;

    subplot(num_plots,2,1)
    plot(t, rad2deg(roll)); 
    legend("\phi")
    ylabel("[deg]")

    subplot(num_plots,2,3)
    plot(t, rad2deg(pitch)); 
    legend("\theta")
    ylabel("[deg]")

    subplot(num_plots,2,5)
    plot(t, rad2deg(yaw)); 
    legend("\psi")
    ylabel("[deg]")

    subplot(num_plots,2,7)
    plot(t, rad2deg(p));
    legend("p")
    ylabel("[deg/s]")

    subplot(num_plots,2,9)
    plot(t, rad2deg(q));
    legend("q")
    ylabel("[deg/s]")

    subplot(num_plots,2,11)
    plot(t, rad2deg(r));
    legend("r")
    ylabel("[deg/s]")

    subplot(num_plots,2,13)
    plot(t, u);
    legend("u")
    ylabel("[m/s]")

    subplot(num_plots,2,15)
    plot(t, v);
    legend("v")
    ylabel("[m/s]")

    subplot(num_plots,2,17)
    plot(t, w);
    legend("w")
    ylabel("[m/s]")
    
    subplot(num_plots,2,2)
    plot(t, rad2deg(alpha)); 
    legend("\alpha")
    ylabel("[deg]")

    subplot(num_plots,2,4)
    plot(t, rad2deg(beta)); 
    legend("\beta")
    ylabel("[deg]")
    
    subplot(num_plots,2,6)
    plot(t, V_a); 
    legend("V_a")
    ylabel("[m/s]")

    subplot(num_plots,2,8)
    plot(t, rad2deg(input(:,5)));
    legend("\delta_a")
    ylabel("[deg]");

    subplot(num_plots,2,10)
    plot(t, rad2deg(input(:,6)));
    legend("\delta_e")
    ylabel("[deg]");

    subplot(num_plots,2,12)
    plot(t, rad2deg(input(:,7)));
    legend("\delta_r")
    ylabel("[deg]");
    
    subplot(num_plots,2,14)
    plot(t, input(:,8));
    legend("n_P")
    ylabel("[RPM]");
    
    figure_title = "state and input, maneuver: " + maneuver_index;
    sgtitle(figure_title)

    if save_plot
        filename = maneuver_index + "_state_input";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end