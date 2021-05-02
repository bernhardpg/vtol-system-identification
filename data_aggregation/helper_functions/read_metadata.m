function [maneuvers_to_aggregate, metadata] = read_metadata(filepath)
    % Read maneuver trims from file
    temp = readtable(filepath);
    maneuvers = temp.maneuver;
    start_s = temp.start_s;
    end_s = temp.end_s;

    % Custom data trim
    metadata = containers.Map;
    maneuvers_to_aggregate = [];

    for i = 1:length(maneuvers)
        metadata(string(maneuvers(i))) = [start_s(i) end_s(i)];
        if start_s(i) == 0 || end_s(i) == 0
            continue
        end
        maneuvers_to_aggregate = [maneuvers_to_aggregate maneuvers(i)];
    end
end