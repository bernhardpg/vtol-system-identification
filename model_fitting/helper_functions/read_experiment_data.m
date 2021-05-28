function [state, input, t, maneuver_start_indices] = read_experiment_data(metadata, maneuver_types)
    num_experiments = length(metadata.Experiments);
    experiment_data_path = "data/experiments/";

    state = [];
    input = [];
    maneuver_start_indices = [];
    t = [];
    
    for type_i = 1:length(maneuver_types)
        maneuver_type = maneuver_types(type_i);
        
        for experiment_i = 1:num_experiments
            datapath = experiment_data_path + "experiment_" + metadata.Experiments(experiment_i).Number ...
                + "/" + maneuver_type + "/output/";
            if ~exist(datapath, 'dir')
                continue
            end

            state_exp = readmatrix(datapath + "state.csv");
            input_exp = readmatrix(datapath + "input.csv");
            maneuver_start_indices_exp = readmatrix(datapath + "maneuver_start_indices.csv");
            t_exp = readmatrix(datapath + "t.csv");

            state = [state;
                     state_exp];
            input = [input;
                     input_exp];
            maneuver_start_indices = [maneuver_start_indices...
                maneuver_start_indices_exp];
            t = [t;
                 t_exp];
        end
    end
    
    disp("Loaded " + length(maneuver_start_indices) + " maneuvers.")
end