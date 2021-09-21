function [] = create_bar_plot(model_metric, model_names, metric_name, variable_names)
    figure
    X = categorical(variable_names);
    X = reordercats(X,variable_names);
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
end