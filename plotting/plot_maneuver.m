function plot_maneuver(time, y_sim, y_recorded, input_seq, vlines)
    % Extract variables
    u = y_sim(:,1);
    v = y_sim(:,2);
    w = y_sim(:,3);
    p = y_sim(:,4);
    q = y_sim(:,5);
    r = y_sim(:,6);
    phi = y_sim(:,7);
    theta = y_sim(:,8);
    delta_a = y_sim(:,9);
    delta_e = y_sim(:,10);
    delta_r = y_sim(:,11);

    u_rec = y_recorded(:,1);
    v_rec = y_recorded(:,2);
    w_rec = y_recorded(:,3);
    p_rec = y_recorded(:,4);
    q_rec = y_recorded(:,5);
    r_rec = y_recorded(:,6);
    phi_rec = y_recorded(:,7);
    theta_rec = y_recorded(:,8);
    delta_a_sp = input_seq(:,1);
    delta_e_sp = input_seq(:,2);
    delta_r_sp = input_seq(:,3);
    delta_t = input_seq(:,4);

    % Plot
    plot_settings;
    fig = figure;
    fig.Position = [0,1000,1000,1000];
    
    %%% LONGITUDINAL
    t = tiledlayout(7,1);
    nexttile
    plot(time, u, 'Color', lon_color, 'LineWidth',line_width); hold on
    plot(time, u_rec, "--", 'LineWidth',line_width,'Color', target_color)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    ylabel("u")
    set(gca,'FontSize', font_size_small)
    ylabel("$u [m/s]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([10 25])
    legend("Predicted", "Recorded",'Location','best')
    xlim([0 time(end)]);
    
    nexttile
    plot(time, w, 'Color', lon_color, 'LineWidth',line_width); hold on
    plot(time, w_rec, "--", 'LineWidth',line_width,'Color', target_color)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    ylabel("w")
    set(gca,'FontSize', font_size_small)
    ylabel("$w [m/s]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-6 6])
    xlim([0 time(end)]);
    
    nexttile
    plot(time, rad2deg(q), 'Color', lon_color, 'LineWidth',line_width); hold on
    plot(time, rad2deg(q_rec), "--", 'LineWidth',line_width,'Color', target_color)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    ylabel("q")
    set(gca,'FontSize', font_size_small)
    ylim([-100 100]);
    ylabel("$q [^\circ/s]$", 'interpreter', 'latex', 'FontSize', font_size)
    xlim([0 time(end)]);

    nexttile
    plot(time, rad2deg(theta), 'Color', lon_color, 'LineWidth',line_width); hold on
    plot(time, rad2deg(theta_rec), "--", 'LineWidth',line_width,'Color', target_color)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    ylabel("theta")
    title(t, "Full Model Simulation (Longitudinal States)", 'interpreter', 'latex', 'FontSize', font_size_large);
    set(gca,'FontSize', font_size_small)
    ylabel("$\theta [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-30 30])
    xlim([0 time(end)]);
    
    nexttile
    plot(time, rad2deg(delta_a),"k", time, rad2deg(delta_a_sp), "k--",  'LineWidth', line_width)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_a [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-32 32])
    legend("Input")
    xlim([0 time(end)]);
    
    nexttile
    plot(time, rad2deg(delta_e), "k",time, rad2deg(delta_e_sp), "k--",  'LineWidth', line_width)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_e [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-32 32])
    xlim([0 time(end)]);
    
    nexttile
    plot(time, rad2deg(delta_r), "k",time, rad2deg(delta_r_sp), "k--",  'LineWidth', line_width)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_r [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-32 32])
    xlim([0 time(end)]);
    xlabel(t, "Time $[s]$", 'interpreter', 'latex', 'FontSize', font_size)
 
    
    %%% LATERAL
    fig = figure;
    fig.Position = [800,1000,1000,1000];
    
    t = tiledlayout(7,1);
    nexttile
    plot(time, v, 'Color', lat_color, 'LineWidth',line_width); hold on
    plot(time, v_rec, "--", 'LineWidth',line_width,'Color', target_color)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    ylabel("v")
    set(gca,'FontSize', font_size_small)
    ylabel("$v [m/s]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-13 13])
    legend("Predicted", "Recorded",'Location','best')
    xlim([0 time(end)]);
    
    nexttile
    plot(time, rad2deg(p), 'Color', lat_color, 'LineWidth',line_width); hold on
    plot(time, rad2deg(p_rec), "--", 'LineWidth',line_width,'Color', target_color)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    ylabel("p")
    set(gca,'FontSize', font_size_small)
    ylabel("$p [^\circ/s]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-160 160]);    
    xlim([0 time(end)]);
    
    nexttile
    plot(time, rad2deg(r), 'Color', lat_color, 'LineWidth',line_width); hold on
    plot(time, rad2deg(r_rec), "--", 'LineWidth',line_width,'Color', target_color)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    ylabel("r")
    set(gca,'FontSize', font_size_small)
    ylim([-120 120]);
    ylabel("$r [^\circ/s]$", 'interpreter', 'latex', 'FontSize', font_size)    
    xlim([0 time(end)]);

    nexttile
    plot(time, rad2deg(phi), 'Color', lat_color, 'LineWidth',line_width); hold on
    plot(time, rad2deg(phi_rec), "--", 'LineWidth',line_width,'Color', target_color)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    ylabel("phi")
    set(gca,'FontSize', font_size_small)
    ylabel("$\phi [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-90 90])
    title(t, "Full Model Simulation (Lateral-Directional States)", 'interpreter', 'latex', 'FontSize', font_size_large);
    xlim([0 time(end)]);
    
    nexttile
    plot(time, rad2deg(delta_a),"k", time, rad2deg(delta_a_sp), "k--",  'LineWidth', line_width)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_a [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-32 32])
    legend("Input")
    xlim([0 time(end)]);
    
    nexttile
    plot(time, rad2deg(delta_e), "k",time, rad2deg(delta_e_sp), "k--",  'LineWidth', line_width)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_e [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-32 32])
    xlim([0 time(end)]);
    
    nexttile
    plot(time, rad2deg(delta_r), "k",time, rad2deg(delta_r_sp), "k--",  'LineWidth', line_width)
    for i = 1:length(vlines)
        xline(time(vlines(i)),"--");
    end
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_r [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-32 32])
    xlim([0 time(end)]);
    xlabel(t, "Time $[s]$", 'interpreter', 'latex', 'FontSize', font_size)
end