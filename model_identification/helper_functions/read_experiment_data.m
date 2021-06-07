function [t, q_NB, v_N, maneuver_start_indices] = read_experiment_data(metadata, maneuver_types)
    num_experiments = length(metadata.Experiments);
    experiment_data_path = "data/experiments/";

    t = [];
    q_NB = [];
    v_N = [];
    maneuver_start_indices = [];
    
    for type_i = 1:length(maneuver_types)
        maneuver_type = maneuver_types(type_i);
        
        for experiment_i = 1:num_experiments
            datapath = experiment_data_path + "experiment_" + metadata.Experiments(experiment_i).Number ...
                + "/" + maneuver_type + "/output/";
            if ~exist(datapath, 'dir')
                continue
            end
            
            t_exp = readmatrix(datapath + "t.csv");
            q_NB_exp = readmatrix(datapath + "q_NB.csv");
            v_N_exp = readmatrix(datapath + "v_N.csv");
            maneuver_start_indices_exp = readmatrix(datapath + "maneuver_start_indices.csv");
            
            % Make maneuver start indices continue from the previous ones
            % already loaded.
            maneuver_start_indices_exp = maneuver_start_indices_exp + length(t);
            
            q_NB = [q_NB;
                     q_NB_exp];
            v_N = [v_N;
                     v_N_exp];
            maneuver_start_indices = [maneuver_start_indices...
                maneuver_start_indices_exp];
            t = [t;
                 t_exp];
        end
    end
    
    disp("Loaded " + length(maneuver_start_indices) + " maneuvers.")
end