function plot_coeffs(time, predicted_values, recorded_values, num_maneuvers_to_plot, maneuver_end_indices, coeff_names, plot_title, model_type)
    % import plot settings
    plot_settings;
    fig = figure;
    fig.Position = [0,0,1000,600];

    [~, num_coeffs] = size(predicted_values);
    
    t = tiledlayout(num_coeffs, 1, 'Padding', 'compact', 'TileSpacing', 'compact'); 
    
    for coeff_i = 1:num_coeffs
        nexttile
        plot(time, recorded_values(1:maneuver_end_indices(num_maneuvers_to_plot),coeff_i), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on
        if strcmp(model_type, "longitudinal")
            plot(time, predicted_values(1:maneuver_end_indices(num_maneuvers_to_plot),coeff_i), 'Color', lon_color); hold on
        elseif strcmp(model_type, "lateral-directional")
            plot(time, predicted_values(1:maneuver_end_indices(num_maneuvers_to_plot),coeff_i), 'Color', lat_color); hold on
        end
        grid on
        grid minor
        plot_maneuver_lines(maneuver_end_indices, num_maneuvers_to_plot, time)
        ylabel(coeff_names(coeff_i), 'interpreter', 'latex', 'FontSize', font_size_large)
        xlim([0 time(end)]);
    end
    lgd = legend(["Recorded Data" "Equation-Error Model"], 'location', 'southeast', 'FontSize', font_size_small);
    title(t, plot_title, 'FontSize', font_size_large, 'interpreter', 'latex')
    xlabel(t, "Time $[s]$", 'interpreter', 'latex', 'FontSize', font_size)
end


function plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
    for maneuver_i = 1:num_maneuvers_to_plot-1
        xline(time(maneuver_start_index(maneuver_i)),"--"); hold on
    end
end