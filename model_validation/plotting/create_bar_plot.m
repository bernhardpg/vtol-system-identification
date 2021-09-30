function [] = create_bar_plot(model_metric, model_names, metric_name, variable_names, variable_names_latex)
    figure
    plot_settings;
    num_variables = length(variable_names);
    Y = [model_metric';
         mean(model_metric,2)'];
    b = bar(1:num_variables+1, Y); hold on
    ylim([0 max(max(Y))*1.25]);
    title(metric_name, 'interpreter', 'latex', 'FontSize',font_size_large)
    xline(num_variables + 0.5,"--"); % division line between mean and other values
    legend(model_names, "Location", "best",'interpreter','latex','FontSize',font_size)

    % Add text to top of each bar
    for i = 1:length(b)
        xtips = b(i).XEndPoints;
        ytips = b(i).YEndPoints;
        labels = string(split(num2str(b(i).YData,2)));
        text(xtips,ytips,labels,'HorizontalAlignment','center',...
            'VerticalAlignment','bottom','interpreter','latex', 'FontSize',font_size_small)
    end
    
    set(gca,'XTickLabel',[variable_names_latex "Mean"],'TickLabelInterpreter','latex', 'FontSize',font_size);
end