function plot_maneuver(t_sim, y_sim, t_seq, y_recorded, input_seq)
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
    fig.Position = [0,1000,800,500];
    
    t = tiledlayout(4,1);
    nexttile
    plot(t_sim, u, t_seq, u_rec, "--")
    ylabel("u")
    set(gca,'FontSize', font_size_small)
    ylabel("$u [m/s]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([10 25])
    
    nexttile
    plot(t_sim, w, t_seq, w_rec, "--")
    ylabel("w")
    set(gca,'FontSize', font_size_small)
    ylabel("$w [m/s]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-6 6])
    
    nexttile
    plot(t_sim, rad2deg(q), t_seq, rad2deg(q_rec), "--")
    ylabel("q")
    set(gca,'FontSize', font_size_small)
    ylim([-100 100]);
    ylabel("$q [^\circ/s]$", 'interpreter', 'latex', 'FontSize', font_size)

    nexttile
    plot(t_sim, rad2deg(theta), t_seq, rad2deg(theta_rec), "--")
    ylabel("theta")
    legend("Predicted", "Recorded")
    title(t, "Longitudinal States", 'interpreter', 'latex', 'FontSize', font_size_large);
    set(gca,'FontSize', font_size_small)
    ylabel("$\theta [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-30 30])
    
    fig = figure;
    fig.Position = [800,1000,800,500];
    
    t = tiledlayout(4,1);
    nexttile
    plot(t_sim, v, t_seq, v_rec, "--")
    ylabel("v")
    set(gca,'FontSize', font_size_small)
    ylabel("$v [m/s]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-13 13])
    
    nexttile
    plot(t_sim, rad2deg(p), t_seq, rad2deg(p_rec), "--")
    ylabel("p")
    set(gca,'FontSize', font_size_small)
    ylabel("$p [^\circ/s]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-160 160]);    
    
    nexttile
    plot(t_sim, rad2deg(r), t_seq, rad2deg(r_rec), "--")
    ylabel("r")
    set(gca,'FontSize', font_size_small)
    ylim([-120 120]);
    ylabel("$r [^\circ/s]$", 'interpreter', 'latex', 'FontSize', font_size)    

    nexttile
    plot(t_sim, rad2deg(phi), t_seq, rad2deg(phi_rec), "--")
    ylabel("phi")
    set(gca,'FontSize', font_size_small)
    ylabel("$\phi [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-90 90])
    legend("Predicted", "Recorded")
    title(t, "Lateral-Directional States", 'interpreter', 'latex', 'FontSize', font_size_large);

    fig = figure;
    fig.Position = [1600,1000,800,500];
    
    t = tiledlayout(4,1);
    nexttile
    plot(t_sim, rad2deg(delta_a), t_seq, rad2deg(delta_a_sp), "--")
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_a [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-32 32])
    
    nexttile
    plot(t_sim, rad2deg(delta_e), t_seq, rad2deg(delta_e_sp), "--")
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_e [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-32 32])
    
    nexttile
    plot(t_sim, rad2deg(delta_r), t_seq, rad2deg(delta_r_sp), "--")
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_r [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([-32 32])
    
    nexttile
    plot(t_seq, delta_t)
    set(gca,'FontSize', font_size_small)
    ylabel("$\delta_t [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
    ylim([0 125^2])

    title(t, "Inputs", 'interpreter', 'latex', 'FontSize', font_size_large);

end