function plot_coeffs(time, predicted_values, recorded_values, first_maneuver_index, last_maneuver_index, maneuver_end_indices, coeff_names, plot_title, model_type)
    % import plot settings
    plot_settings;
    fig = figure;
    fig.Position = [0,0,1000,600];

    [~, num_coeffs] = size(predicted_values);
    
    t = tiledlayout(num_coeffs, 1, 'Padding', 'compact', 'TileSpacing', 'compact'); 
    start_index = maneuver_end_indices(first_maneuver_index)+1;
    end_index = maneuver_end_indices(last_maneuver_index+1);
    
    for coeff_i = 1:num_coeffs
        nexttile
        plot(time, recorded_values(start_index:end_index,coeff_i), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on
        if strcmp(model_type, "longitudinal")
            plot(time, predicted_values(start_index:end_index,coeff_i),  'LineWidth',line_width,'Color', lon_color); hold on
        elseif strcmp(model_type, "lateral-directional")
            plot(time, predicted_values(start_index:end_index,coeff_i),  'LineWidth',line_width,'Color', lat_color); hold on
        end
        grid on
        grid minor
        plot_maneuver_lines(maneuver_end_indices, first_maneuver_index, last_maneuver_index, time);
        ylabel(coeff_names(coeff_i), 'interpreter', 'latex', 'FontSize', font_size_large)
        xlim([0 time(end)]);
    end
    lgd = legend(["Recorded Data" "Equation-Error Model"], 'location', 'best', 'FontSize', font_size_small);
    title(t, plot_title, 'FontSize', font_size_large, 'interpreter', 'latex')
    xlabel(t, "Time $[s]$", 'interpreter', 'latex', 'FontSize', font_size)
end


function plot_maneuver_lines(maneuver_end_indices, first_maneuver_index, last_maneuver_index, time)
    maneuver_start_index = maneuver_end_indices(first_maneuver_index) - 1;
    for maneuver_i = first_maneuver_index:last_maneuver_index
        xline(time(maneuver_end_indices(maneuver_i) - maneuver_start_index),"--"); hold on
    end
end