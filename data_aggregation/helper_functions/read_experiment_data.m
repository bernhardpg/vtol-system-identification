function [t_state, q_NB, v_N, t_u_fw, u_fw, maneuver_start_indices_state, maneuver_start_indices_u_fw] = read_experiment_data(metadata, maneuver_types)
    num_experiments = length(metadata.Experiments);
    experiment_data_path = "data/data_raw/experiments/";

    t_state = [];
    t_u_fw = [];
    q_NB = [];
    v_N = [];
    u_mr = [];
    u_fw = [];
    maneuver_start_indices_state = [];
    maneuver_start_indices_u_fw = [];
    
    for type_i = 1:length(maneuver_types)
        maneuver_type = maneuver_types(type_i);
        
        for experiment_i = 1:num_experiments
            datapath = experiment_data_path + "experiment_" + metadata.Experiments(experiment_i).Number ...
                + "/" + maneuver_type + "/output/";
            if ~exist(datapath, 'dir')
                continue
            end
            
            t_state_exp = readmatrix(datapath + "t_state.csv");
            t_u_fw_exp = readmatrix(datapath + "t_u_fw.csv");
            q_NB_exp = readmatrix(datapath + "q_NB.csv");
            v_N_exp = readmatrix(datapath + "v_N.csv");
            u_mr_exp = readmatrix(datapath + "u_mr.csv");
            u_fw_exp = readmatrix(datapath + "u_fw.csv");
            maneuver_start_indices_state_exp = readmatrix(datapath + "maneuver_start_indices_state.csv");
            maneuver_start_indices_u_fw_exp = readmatrix(datapath + "maneuver_start_indices_u_fw.csv");
            
            % Make maneuver start indices continue from the previous ones
            % already loaded.
            maneuver_start_indices_state_exp = maneuver_start_indices_state_exp + length(t_state);
            maneuver_start_indices_u_fw_exp = maneuver_start_indices_u_fw_exp + length(t_u_fw);
            
            q_NB = [q_NB;
                    q_NB_exp];
            v_N = [v_N;
                   v_N_exp];
            u_mr = [u_mr;
                    u_mr_exp];
            u_fw = [u_fw;
                    u_fw_exp];
            maneuver_start_indices_state = [maneuver_start_indices_state...
                maneuver_start_indices_state_exp];
            maneuver_start_indices_u_fw = [maneuver_start_indices_u_fw ...
                maneuver_start_indices_u_fw_exp];
            t_state = [t_state;
                       t_state_exp];
            t_u_fw = [t_u_fw;
                      t_u_fw_exp];
        end
    end
    
    disp("Loaded " + length(maneuver_start_indices_state) + " maneuvers.")
end