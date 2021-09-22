function [] = create_bar_plot(model_metric, model_names, metric_name, variable_names, variable_names_latex)
    figure
    X = categorical(variable_names);
    X = reordercats(X,string(X));
    Y = model_metric';
    b = bar(X, Y);
    legend(model_names, "Location", "SouthEast")
    title(metric_name)

    for i = 1:length(b)
        xtips = b(i).XEndPoints;
        ytips = b(i).YEndPoints;
        labels = string(split(num2str(b(i).YData,2)));
        text(xtips,ytips,labels,'HorizontalAlignment','center',...
            'VerticalAlignment','bottom')
    end
    
    set(gca,'xtick',X,'XTickLabel',variable_names_latex,'TickLabelInterpreter','latex');
end