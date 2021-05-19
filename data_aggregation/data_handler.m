clc; clear all; close all;

metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Output data
save_output_data = true;
save_plot = false;
show_plot = false;

% Set common data time resolution
dt = 1/100;

num_experiments = length(metadata.Experiments);
for i = 1:num_experiments
    parse_data_from_flight(metadata.Experiments(i), dt, save_output_data, save_plot, show_plot);
end

%% Functions

function [] = plot_single_maneuver(experiment, maneuver_i, dt)
    show_plot = true;
    save_plot = false;

    log_file = experiment.LogName;
    maneuver_metadata = experiment.Maneuvers;
    num_maneuvers = length(fieldnames(maneuver_metadata));
    
    csv_files_location = "data/log_files/csv/";
    csv_log_file_location = csv_files_location + log_file;

    % Load aircraft parameters
    aircraft_properties;

    % Read state and input data
    [t, state, input] = read_state_and_input_from_log(csv_log_file_location, dt);
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    
    % Read accelerations
    [acc_B_unfiltered, acc_B_raw, t_acc_raw, bias_acc] = read_accelerations(csv_log_file_location, t);
    [ang_acc_unfiltered, ang_acc_raw, t_ang_acc] = read_ang_acc(csv_log_file_location, t);

    
    % Cutoff frequency for acceleration data
    f_cutoff_lin_acc = 25;
    f_cutoff_ang_acc = 10;

    % Process acceleration data
    acc_B = filter_accelerations(t, dt, acc_B_raw, t_acc_raw, f_cutoff_lin_acc);
    ang_acc = filter_accelerations(t, dt, ang_acc_raw, t_ang_acc, f_cutoff_ang_acc);
    
    % Calculate acceleration data in NED frame
    acc_N = calculate_acc_N(acc_B, q_NB);

    % Calculate Angle of Attack
    AoA_rad = atan2(v_B(:,3),v_B(:,1));
    AoA_deg = rad2deg(AoA_rad);

    % Calculate lift and drag
    v_A = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);
    [L, D, c_L, c_D] = calculate_lift_and_drag(state, input, AoA_rad, v_A, acc_B, mass_kg, g);
    
    % Calculate pitch moment
    [c_m, Tau_y] = calculate_pitch_moment(state, v_A, ang_acc, lam_5, lam_6, Jyy, rho);
    
    % Get system identification maneuvers
    sysid_indices = get_sysid_indices(csv_log_file_location, t);

    % Normal maneuver length
    default_maneuver_padding_s = 1;
    maneuver_length_s = 6;
    
    i = maneuver_i;
    % Read maneuver metadata
    curr_maneuver_name = "x" + string(i);
    curr_maneuver_metadata = maneuver_metadata.(curr_maneuver_name);

    [maneuver_should_be_aggregated,...
        maneuver_start_index,...
        maneuver_end_index] = read_maneuver_metadata(...
            curr_maneuver_metadata, default_maneuver_padding_s, t, dt, sysid_indices(i), maneuver_length_s ...
        );

    % Extract maneuver data
    t_maneuver = t(maneuver_start_index:maneuver_end_index)';
    state_maneuver = state(maneuver_start_index:maneuver_end_index,:);
    input_maneuver = input(maneuver_start_index:maneuver_end_index,:);
    AoA_deg_maneuver = AoA_deg(maneuver_start_index:maneuver_end_index);
    AoA_rad_maneuver = AoA_rad(maneuver_start_index:maneuver_end_index);
    acc_B_maneuver = acc_B(maneuver_start_index:maneuver_end_index,:);
    acc_B_unfiltered_maneuver = acc_B_unfiltered(maneuver_start_index:maneuver_end_index,:);
    acc_N_maneuver = acc_N(maneuver_start_index:maneuver_end_index,:);
    bias_acc_maneuver = bias_acc(maneuver_start_index:maneuver_end_index);
    ang_acc_maneuver = ang_acc(maneuver_start_index:maneuver_end_index,:);
    ang_acc_unfiltered_maneuver = ang_acc_unfiltered(maneuver_start_index:maneuver_end_index,:);
    L_maneuver = L(maneuver_start_index:maneuver_end_index);
    D_maneuver = D(maneuver_start_index:maneuver_end_index);
    Tau_y_maneuver = Tau_y(maneuver_start_index:maneuver_end_index);
    c_L_maneuver = c_L(maneuver_start_index:maneuver_end_index);
    c_D_maneuver = c_D(maneuver_start_index:maneuver_end_index);
    c_m_maneuver = c_m(maneuver_start_index:maneuver_end_index);

    plot_trajectory(i, t_maneuver, state_maneuver, input_maneuver, show_plot, save_plot, "");
    plot_trajectory_static_details(i, t_maneuver, state_maneuver, input_maneuver,...
        AoA_deg_maneuver, acc_B_unfiltered_maneuver, acc_B_maneuver, bias_acc_maneuver,...
        acc_N_maneuver, show_plot, save_plot, "");
    plot_accelerations(i, t_maneuver, acc_B_maneuver, acc_B_unfiltered_maneuver, bias_acc_maneuver, ang_acc_maneuver, ang_acc_unfiltered_maneuver, show_plot, save_plot, "");
    scatter_static_curves(i, t_maneuver, AoA_deg_maneuver, L_maneuver, D_maneuver, Tau_y_maneuver, c_L_maneuver, c_D_maneuver, c_m_maneuver, show_plot, save_plot, "");
end



function [] = parse_data_from_flight(experiment, dt, save_output_data, save_plot, show_plot)
    log_file = experiment.LogName;
    maneuver_metadata = experiment.Maneuvers;
    num_maneuvers = length(fieldnames(maneuver_metadata));
    
    csv_files_location = "data/log_files/csv/";
    csv_log_file_location = csv_files_location + log_file;

    % Load aircraft parameters
    aircraft_properties;

    % Read state and input data
    [t, state, input] = read_state_and_input_from_log(csv_log_file_location, dt);
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    
    % Read accelerations
    [acc_B_unfiltered, acc_B_raw, t_acc_raw, bias_acc] = read_accelerations(csv_log_file_location, t);
    [ang_acc_unfiltered, ang_acc_raw, t_ang_acc] = read_ang_acc(csv_log_file_location, t);

    % Cutoff frequency for acceleration data
    f_cutoff_lin_acc = 25;
    f_cutoff_ang_acc = 1000;

    % Process acceleration data
    acc_B = filter_accelerations(t, dt, acc_B_raw, t_acc_raw, f_cutoff_lin_acc);
    ang_acc = filter_accelerations(t, dt, ang_acc_raw, t_ang_acc, f_cutoff_ang_acc);
    
    % Calculate acceleration data in NED frame
    acc_N = calculate_acc_N(acc_B, q_NB);

    % Calculate Angle of Attack
    AoA_rad = atan2(v_B(:,3),v_B(:,1));
    AoA_deg = rad2deg(AoA_rad);

    % Calculate lift and drag
    v_A = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);
    [L, D, c_L, c_D] = calculate_lift_and_drag(state, input, AoA_rad, v_A, acc_B, mass_kg, g);
    
    % Calculate pitch moment
    [c_m, Tau_y] = calculate_pitch_moment(state, v_A, ang_acc, lam_5, lam_6, Jyy, rho);
    
    % Get system identification maneuvers
    sysid_indices = get_sysid_indices(csv_log_file_location, t);
    
    % Initialize empty output variables
    output_data = intialize_empty_output_struct();

    % Normal maneuver length
    default_maneuver_padding_s = 1;

    % Iterate through maneuvers
    num_aggregated_maneuvers = 0;
    for i = 1:num_maneuvers
        % Read maneuver metadata
        curr_maneuver_name = "x" + string(i);
        curr_maneuver_metadata = maneuver_metadata.(curr_maneuver_name);
        
        [maneuver_length_s] = get_default_maneuver_length_s(curr_maneuver_metadata.type);
        
        [maneuver_should_be_aggregated,...
            maneuver_start_index,...
            maneuver_end_index] = read_maneuver_metadata(...
                curr_maneuver_metadata, default_maneuver_padding_s, t, dt, sysid_indices(i), maneuver_length_s ...
            );

        % Extract maneuver data
        t_maneuver = t(maneuver_start_index:maneuver_end_index)';
        state_maneuver = state(maneuver_start_index:maneuver_end_index,:);
        input_maneuver = input(maneuver_start_index:maneuver_end_index,:);
        AoA_deg_maneuver = AoA_deg(maneuver_start_index:maneuver_end_index);
        AoA_rad_maneuver = AoA_rad(maneuver_start_index:maneuver_end_index);
        acc_B_maneuver = acc_B(maneuver_start_index:maneuver_end_index,:);
        acc_B_unfiltered_maneuver = acc_B_unfiltered(maneuver_start_index:maneuver_end_index,:);
        acc_N_maneuver = acc_N(maneuver_start_index:maneuver_end_index,:);
        bias_acc_maneuver = bias_acc(maneuver_start_index:maneuver_end_index);
        ang_acc_maneuver = ang_acc(maneuver_start_index:maneuver_end_index,:);
        ang_acc_unfiltered_maneuver = ang_acc_unfiltered(maneuver_start_index:maneuver_end_index,:);
        L_maneuver = L(maneuver_start_index:maneuver_end_index);
        D_maneuver = D(maneuver_start_index:maneuver_end_index);
        Tau_y_maneuver = Tau_y(maneuver_start_index:maneuver_end_index);
        c_L_maneuver = c_L(maneuver_start_index:maneuver_end_index);
        c_D_maneuver = c_D(maneuver_start_index:maneuver_end_index);
        c_m_maneuver = c_m(maneuver_start_index:maneuver_end_index);

        % Plot maneuver data
        if save_plot || show_plot
            % Determine output location
            plot_output_location = "data/experiments/experiment_" + string(experiment.Number) ...
                + "/" + string(curr_maneuver_metadata.type) + "/plots/";
            mkdir(plot_output_location);
            
            if ~maneuver_should_be_aggregated
                plot_output_location = plot_output_location + "not_used_";
            end
            
            if curr_maneuver_metadata.type == "sweep"
                continue % TODO: For now, don't generate plot or aggregate data for sweep maneuvers
                if false
                    plot_trajectory_static_details(i, t_maneuver, state_maneuver, input_maneuver,...
                        AoA_deg_maneuver, acc_B_unfiltered_maneuver, acc_B_maneuver, bias_acc_maneuver,...
                        acc_N_maneuver, show_plot, save_plot, plot_output_location);
                    plot_accelerations(i, t_maneuver, acc_B_maneuver, acc_B_unfiltered_maneuver, bias_acc_maneuver, ang_acc_maneuver, ang_acc_unfiltered_maneuver, show_plot, save_plot, plot_output_location);
                    scatter_static_curves(i, t_maneuver, AoA_deg_maneuver, L_maneuver, D_maneuver, Tau_y_maneuver, c_L_maneuver, c_D_maneuver, c_m_maneuver, show_plot, save_plot, plot_output_location);
                end
            end
            plot_trajectory(i, t_maneuver, state_maneuver, input_maneuver, show_plot, save_plot, plot_output_location);
            
        end
        
        % Store data in aggregated data matrices
        if maneuver_should_be_aggregated
            output_data.(curr_maneuver_metadata.type).t = ...
                [output_data.(curr_maneuver_metadata.type).t;
                 t_maneuver];
            output_data.(curr_maneuver_metadata.type).state = ...
                [output_data.(curr_maneuver_metadata.type).state;
                 state_maneuver];
            output_data.(curr_maneuver_metadata.type).input = ...
                [output_data.(curr_maneuver_metadata.type).input;
                 input_maneuver];
            output_data.(curr_maneuver_metadata.type).c_L = ...
                [output_data.(curr_maneuver_metadata.type).c_L;
                 c_L_maneuver];
            output_data.(curr_maneuver_metadata.type).c_D = ...
                [output_data.(curr_maneuver_metadata.type).c_D;
                 c_D_maneuver];
            output_data.(curr_maneuver_metadata.type).AoA = ...
                [output_data.(curr_maneuver_metadata.type).AoA;
                 AoA_rad_maneuver];
            output_data.(curr_maneuver_metadata.type).ang_acc = ...
                [output_data.(curr_maneuver_metadata.type).ang_acc;
                 ang_acc_maneuver];
            output_data.(curr_maneuver_metadata.type).c_m = ...
                [output_data.(curr_maneuver_metadata.type).c_m;
                 c_m_maneuver];
                
            curr_maneuver_index_in_aggregation_matrix = length(output_data.(curr_maneuver_metadata.type).t) + 1;
            output_data.(curr_maneuver_metadata.type).maneuver_start_indices =...
                [output_data.(curr_maneuver_metadata.type).maneuver_start_indices...
                 curr_maneuver_index_in_aggregation_matrix];
            output_data.(curr_maneuver_metadata.type).aggregated_maneuvers = ...
                [output_data.(curr_maneuver_metadata.type).aggregated_maneuvers i];
            num_aggregated_maneuvers = num_aggregated_maneuvers + 1;
        end
    end

    disp("Succesfully aggregated " + num_aggregated_maneuvers + " maneuvers");

    % Save to files
    if save_output_data
        save_output(experiment.Number, output_data)
    end
end

function [maneuver_length_s] = get_default_maneuver_length_s(maneuver_type) 
        if maneuver_type == "sweep"
            maneuver_length_s = 6;
        elseif (maneuver_type == "roll_211") || (maneuver_type == "roll_211_no_throttle")
            maneuver_length_s = 2;
        elseif (maneuver_type == "pitch_211") || (maneuver_type == "pitch_211_no_throttle")
            maneuver_length_s = 2;
        elseif (maneuver_type == "yaw_211") || (maneuver_type == "yaw_211_no_throttle")
            maneuver_length_s = 4;
        else
            maneuver_length_s = 6;
        end
end

function [] = save_output(experiment_number, output_data)
        if ~isempty(output_data.sweep.state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/sweep/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.sweep.state, data_output_location + 'state.csv');
            writematrix(output_data.sweep.input, data_output_location + 'input.csv');
            writematrix(output_data.sweep.t, data_output_location + 't.csv');
            writematrix(output_data.sweep.AoA, data_output_location + 'aoa_rad.csv');
            writematrix(output_data.sweep.c_D, data_output_location + 'cd.csv');
            writematrix(output_data.sweep.c_L, data_output_location + 'cl.csv');
            writematrix(output_data.sweep.c_m, data_output_location + 'cm.csv');
            writematrix(output_data.sweep.ang_acc, data_output_location + 'ang_acc.csv');
            writematrix(output_data.sweep.maneuver_start_indices, data_output_location + 'maneuver_start_indices.csv');
            writematrix(output_data.sweep.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.roll_211.state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/roll_211/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.roll_211.state, data_output_location + 'state.csv');
            writematrix(output_data.roll_211.input, data_output_location + 'input.csv');
            writematrix(output_data.roll_211.t, data_output_location + 't.csv');
            writematrix(output_data.roll_211.AoA, data_output_location + 'aoa_rad.csv');
            writematrix(output_data.roll_211.c_D, data_output_location + 'cd.csv');
            writematrix(output_data.roll_211.c_L, data_output_location + 'cl.csv');
            writematrix(output_data.roll_211.c_m, data_output_location + 'cm.csv');
            writematrix(output_data.roll_211.ang_acc, data_output_location + 'ang_acc.csv');
            writematrix(output_data.roll_211.maneuver_start_indices, data_output_location + 'maneuver_start_indices.csv');
            writematrix(output_data.roll_211.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.roll_211_no_throttle.state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/roll_211_no_throttle/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.roll_211_no_throttle.state, data_output_location + 'state.csv');
            writematrix(output_data.roll_211_no_throttle.input, data_output_location + 'input.csv');
            writematrix(output_data.roll_211_no_throttle.t, data_output_location + 't.csv');
            writematrix(output_data.roll_211_no_throttle.AoA, data_output_location + 'aoa_rad.csv');
            writematrix(output_data.roll_211_no_throttle.c_D, data_output_location + 'cd.csv');
            writematrix(output_data.roll_211_no_throttle.c_L, data_output_location + 'cl.csv');
            writematrix(output_data.roll_211_no_throttle.c_m, data_output_location + 'cm.csv');
            writematrix(output_data.roll_211_no_throttle.ang_acc, data_output_location + 'ang_acc.csv');
            writematrix(output_data.roll_211_no_throttle.maneuver_start_indices, data_output_location + 'maneuver_start_indices.csv');
            writematrix(output_data.roll_211_no_throttle.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.pitch_211.state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/pitch_211/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.pitch_211.state, data_output_location + 'state.csv');
            writematrix(output_data.pitch_211.input, data_output_location + 'input.csv');
            writematrix(output_data.pitch_211.t, data_output_location + 't.csv');
            writematrix(output_data.pitch_211.AoA, data_output_location + 'aoa_rad.csv');
            writematrix(output_data.pitch_211.c_D, data_output_location + 'cd.csv');
            writematrix(output_data.pitch_211.c_L, data_output_location + 'cl.csv');
            writematrix(output_data.pitch_211.c_m, data_output_location + 'cm.csv');
            writematrix(output_data.pitch_211.ang_acc, data_output_location + 'ang_acc.csv');
            writematrix(output_data.pitch_211.maneuver_start_indices, data_output_location + 'maneuver_start_indices.csv');
            writematrix(output_data.pitch_211.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.pitch_211_no_throttle.state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/pitch_211_no_throttle/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.pitch_211_no_throttle.state, data_output_location + 'state.csv');
            writematrix(output_data.pitch_211_no_throttle.input, data_output_location + 'input.csv');
            writematrix(output_data.pitch_211_no_throttle.t, data_output_location + 't.csv');
            writematrix(output_data.pitch_211_no_throttle.AoA, data_output_location + 'aoa_rad.csv');
            writematrix(output_data.pitch_211_no_throttle.c_D, data_output_location + 'cd.csv');
            writematrix(output_data.pitch_211_no_throttle.c_L, data_output_location + 'cl.csv');
            writematrix(output_data.pitch_211_no_throttle.c_m, data_output_location + 'cm.csv');
            writematrix(output_data.pitch_211_no_throttle.ang_acc, data_output_location + 'ang_acc.csv');
            writematrix(output_data.pitch_211_no_throttle.maneuver_start_indices, data_output_location + 'maneuver_start_indices.csv');
            writematrix(output_data.pitch_211_no_throttle.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.yaw_211.state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/yaw_211/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.yaw_211.state, data_output_location + 'state.csv');
            writematrix(output_data.yaw_211.input, data_output_location + 'input.csv');
            writematrix(output_data.yaw_211.t, data_output_location + 't.csv');
            writematrix(output_data.yaw_211.AoA, data_output_location + 'aoa_rad.csv');
            writematrix(output_data.yaw_211.c_D, data_output_location + 'cd.csv');
            writematrix(output_data.yaw_211.c_L, data_output_location + 'cl.csv');
            writematrix(output_data.yaw_211.c_m, data_output_location + 'cm.csv');
            writematrix(output_data.yaw_211.ang_acc, data_output_location + 'ang_acc.csv');
            writematrix(output_data.yaw_211.maneuver_start_indices, data_output_location + 'maneuver_start_indices.csv');
            writematrix(output_data.yaw_211.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.yaw_211_no_throttle.state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/yaw_211_no_throttle/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.yaw_211_no_throttle.state, data_output_location + 'state.csv');
            writematrix(output_data.yaw_211_no_throttle.input, data_output_location + 'input.csv');
            writematrix(output_data.yaw_211_no_throttle.t, data_output_location + 't.csv');
            writematrix(output_data.yaw_211_no_throttle.AoA, data_output_location + 'aoa_rad.csv');
            writematrix(output_data.yaw_211_no_throttle.c_D, data_output_location + 'cd.csv');
            writematrix(output_data.yaw_211_no_throttle.c_L, data_output_location + 'cl.csv');
            writematrix(output_data.yaw_211_no_throttle.c_m, data_output_location + 'cm.csv');
            writematrix(output_data.yaw_211_no_throttle.ang_acc, data_output_location + 'ang_acc.csv');
            writematrix(output_data.yaw_211_no_throttle.maneuver_start_indices, data_output_location + 'maneuver_start_indices.csv');
            writematrix(output_data.yaw_211_no_throttle.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
end

function [output_struct] = intialize_empty_output_struct()
    output_struct.sweep = {};

    output_struct.sweep.maneuver_start_indices = [];
    output_struct.sweep.t = [];
    output_struct.sweep.input = [];
    output_struct.sweep.state = [];
    output_struct.sweep.c_D = [];
    output_struct.sweep.c_L = [];
    output_struct.sweep.c_m = [];
    output_struct.sweep.AoA = [];
    output_struct.sweep.ang_acc = [];
    output_struct.sweep.aggregated_maneuvers = [];

    output_struct.roll_211.maneuver_start_indices = [];
    output_struct.roll_211.t = [];
    output_struct.roll_211.input = [];
    output_struct.roll_211.state = [];
    output_struct.roll_211.c_D = [];
    output_struct.roll_211.c_L = [];
    output_struct.roll_211.c_m = [];
    output_struct.roll_211.AoA = [];
    output_struct.roll_211.ang_acc = [];
    output_struct.roll_211.aggregated_maneuvers = [];
    
    output_struct.roll_211_no_throttle.maneuver_start_indices = [];
    output_struct.roll_211_no_throttle.t = [];
    output_struct.roll_211_no_throttle.input = [];
    output_struct.roll_211_no_throttle.state = [];
    output_struct.roll_211_no_throttle.c_D = [];
    output_struct.roll_211_no_throttle.c_L = [];
    output_struct.roll_211_no_throttle.c_m = [];
    output_struct.roll_211_no_throttle.AoA = [];
    output_struct.roll_211_no_throttle.ang_acc = [];
    output_struct.roll_211_no_throttle.aggregated_maneuvers = [];

    output_struct.pitch_211.maneuver_start_indices = [];
    output_struct.pitch_211.t = [];
    output_struct.pitch_211.input = [];
    output_struct.pitch_211.state = [];
    output_struct.pitch_211.c_D = [];
    output_struct.pitch_211.c_L = [];
    output_struct.pitch_211.c_m = [];
    output_struct.pitch_211.AoA = [];
    output_struct.pitch_211.ang_acc = [];
    output_struct.pitch_211.aggregated_maneuvers = [];

    output_struct.pitch_211_no_throttle.maneuver_start_indices = [];
    output_struct.pitch_211_no_throttle.t = [];
    output_struct.pitch_211_no_throttle.input = [];
    output_struct.pitch_211_no_throttle.state = [];
    output_struct.pitch_211_no_throttle.c_D = [];
    output_struct.pitch_211_no_throttle.c_L = [];
    output_struct.pitch_211_no_throttle.c_m = [];
    output_struct.pitch_211_no_throttle.AoA = [];
    output_struct.pitch_211_no_throttle.ang_acc = [];
    output_struct.pitch_211_no_throttle.aggregated_maneuvers = [];
    
    output_struct.yaw_211.maneuver_start_indices = [];
    output_struct.yaw_211.t = [];
    output_struct.yaw_211.input = [];
    output_struct.yaw_211.state = [];
    output_struct.yaw_211.c_D = [];
    output_struct.yaw_211.c_L = [];
    output_struct.yaw_211.c_m = [];
    output_struct.yaw_211.AoA = [];
    output_struct.yaw_211.ang_acc = [];
    output_struct.yaw_211.aggregated_maneuvers = [];
    
    output_struct.yaw_211_no_throttle.maneuver_start_indices = [];
    output_struct.yaw_211_no_throttle.t = [];
    output_struct.yaw_211_no_throttle.input = [];
    output_struct.yaw_211_no_throttle.state = [];
    output_struct.yaw_211_no_throttle.c_D = [];
    output_struct.yaw_211_no_throttle.c_L = [];
    output_struct.yaw_211_no_throttle.c_m = [];
    output_struct.yaw_211_no_throttle.AoA = [];
    output_struct.yaw_211_no_throttle.ang_acc = [];
    output_struct.yaw_211_no_throttle.aggregated_maneuvers = [];
    
    output_struct.not_set.maneuver_start_indices = [];
    output_struct.not_set.t = [];
    output_struct.not_set.input = [];
    output_struct.not_set.state = [];
    output_struct.not_set.c_D = [];
    output_struct.not_set.c_L = [];
    output_struct.not_set.c_m = [];
    output_struct.not_set.AoA = [];
    output_struct.not_set.ang_acc = [];
    output_struct.not_set.aggregated_maneuvers = [];


end

function [maneuver_should_be_aggregated,...
    maneuver_start_index,...
    maneuver_end_index] = read_maneuver_metadata(...
        curr_maneuver_metadata, default_maneuver_padding_s, t, dt, sysid_index, maneuver_length_s ...
    )
        % Use sysid index for maneuver if no maneuver start and end time is
        % set in metadata
        maneuver_time_not_set = (curr_maneuver_metadata.start_s == -1) || (curr_maneuver_metadata.end_s == -1);
        maneuver_should_be_aggregated = ~((curr_maneuver_metadata.start_s == 0) || (curr_maneuver_metadata.start_s == 0));
        % Use default maneuver length
        if  maneuver_time_not_set || ~maneuver_should_be_aggregated
            maneuver_start_index = max(...
                [sysid_index - round(default_maneuver_padding_s / dt)...
                1]);
            maneuver_end_index = max(...
                [maneuver_start_index + maneuver_length_s / dt + round((default_maneuver_padding_s * 2) / dt)...
                1]);
        % Aggregate maneuver with set start and end time
        else
            maneuver_start_index = round((curr_maneuver_metadata.start_s - t(1)) / dt);
            maneuver_end_index = round((curr_maneuver_metadata.end_s - t(1)) / dt);
        end
end

function [] = read_state_input_data_in_time_interval(start_time_s, end_time_s, csv_log_file_location, dt, save_plot, show_plot, plot_output_location, save_output_data, data_output_location)
    % Read data
    [t, state, input] = read_state_and_input_from_log(csv_log_file_location, dt);

    % Cut data


    % Find indexes in t corresponding to start and end time
    start_index = round(interp1(t, 1:length(t), start_time_s));
    end_index = round(interp1(t, 1:length(t), end_time_s));

    % Cut data
    t = t(start_index:end_index);
    state = state(start_index:end_index,:);
    input = input(start_index:end_index,:);

    % Plot data
    name = "freehand";
    plot_state_input(t, state, input, show_plot, save_plot, plot_output_location, name)   

    % Save to files
    if save_output_data
        writematrix(t, data_output_location + 't.csv');
        writematrix(state, data_output_location + 'state.csv');
        writematrix(input, data_output_location + 'input.csv');
    end

end

function [t, state, input] = read_state_and_input_from_log(csv_log_file_location, dt)
    ekf_data = readtable(csv_log_file_location + '_' + "estimator_status_0" + ".csv");
    angular_velocity = readtable(csv_log_file_location + '_' + "vehicle_angular_velocity_0" + ".csv");
    actuator_controls_mr = readtable(csv_log_file_location + '_' + "actuator_controls_0_0" + ".csv");
    actuator_controls_fw = readtable(csv_log_file_location + '_' + "actuator_controls_1_0" + ".csv");
    
    %%%
    % Create common time vector
    
    t0 = ekf_data.timestamp(1) / 1e6;
    t_end = ekf_data.timestamp(end) / 1e6;

    t = t0:dt:t_end;
    N = length(t);
    
    %%%
    % Extract data from ekf2
    % NOTE: EKF2 runs at a delayed time horizon, determined by the largest
    % EKF2_*_DELAY parameter. This is currently set to 175 ms.
    ekf2_delay_s = 0.175;

    t_ekf = ekf_data.timestamp / 1e6 - ekf2_delay_s;

    % q_NB = unit quaternion describing vector rotation from NED to Body. i.e.
    % describes transformation from Body to NED frame.
    % Note: This is the same as the output_predictor quaternion. Something is
    % wrong with documentation

    q0_raw = ekf_data.states_0_;
    q1_raw = ekf_data.states_1_;
    q2_raw = ekf_data.states_2_;
    q3_raw = ekf_data.states_3_;
    q_NB_raw = [q0_raw q1_raw q2_raw q3_raw];
    

    % Extrapolate to correct time
    q_NB = interp1q(t_ekf, q_NB_raw, t');
    % q_NB is the quat we are looking for for our state vector
    q0 = q_NB(:,1);
    q1 = q_NB(:,2);
    q2 = q_NB(:,3);
    q3 = q_NB(:,4);

    v_n = ekf_data.states_4_;
    v_e = ekf_data.states_5_;
    v_d = ekf_data.states_6_;
    v_N_raw = [v_n v_e v_d];

    % Extrapolate to correct time
    v_N = interp1q(t_ekf, v_N_raw, t');

    p_n = ekf_data.states_7_;
    p_e = ekf_data.states_8_;
    p_d = ekf_data.states_9_;
    p_N_raw = [p_n p_e p_d];
    p_N = interp1q(t_ekf, p_N_raw, t');

    %%%
    % Extract angular velocities from output predictor

    t_ang_vel = angular_velocity.timestamp / 1e6;

    % Angular velocity around body axes
    p_raw = angular_velocity.xyz_0_;
    q_raw = angular_velocity.xyz_1_;
    r_raw = angular_velocity.xyz_2_;
    w_B_raw = [p_raw q_raw r_raw];
    
    % Extrapolate to correct time
    w_B = interp1q(t_ang_vel, w_B_raw, t');
    p = w_B(:,1);
    q = w_B(:,2);
    r = w_B(:,3);

    %%%
    % Convert velocity to correct frames

    % This is rotated with rotation matrices only for improved readability,
    % as both the PX4 docs is ambiguous in describing q, and quatrotate() is
    % pretty ambigious too.
    R_NB = quat2rotm(q_NB);

    q_BN = quatinv(q_NB);
    R_BN = quat2rotm(q_BN);

    v_B = zeros(N, 3);
    for i = 1:N
       % Notice how the Rotation matrix has to be inverted here to get the
       % right result, indicating that q is in fact q_NB and not q_BN.
       v_B(i,:) = (R_BN(:,:,i) * v_N(i,:)')';
    end
    u = v_B(:,1);
    v = v_B(:,2);
    w = v_B(:,3);
    
    % The following line gives the same result, indicating that quatrotate() does not perform a
    % simple quaternion product: q (x) v (x) q_inv
    % v_B = quatrotate(q_NB, v_N);
    
    %%%
    % Extract input data
    t_u_mr = actuator_controls_mr.timestamp / 1e6;
    u_roll_mr = actuator_controls_mr.control_0_;
    u_pitch_mr = actuator_controls_mr.control_1_;
    u_yaw_mr = actuator_controls_mr.control_2_;
    u_throttle_mr = actuator_controls_mr.control_3_;
    u_mr_raw = [u_roll_mr u_pitch_mr u_yaw_mr u_throttle_mr];

    u_mr = interp1q(t_u_mr, u_mr_raw, t');
    % TODO: Convert these from PX4 inputs to actual RPMs of each motor

    t_u_fw = actuator_controls_fw.timestamp / 1e6;
    % Inputs in [0,1] interval
    fw_aileron_input = actuator_controls_fw.control_0_;
    fw_elevator_input = actuator_controls_fw.control_1_;
    fw_rudder_input = actuator_controls_fw.control_2_;
    fw_throttle_input = actuator_controls_fw.control_3_;
    
    % Convert from [0,1] interval to angles
    [aileron_angle_rad, elevator_angle_rad, rudder_angle_rad] = ...
        calculate_control_surface_angles_rad(fw_aileron_input, fw_elevator_input, fw_rudder_input);
    [fw_throttle_rpm] = calculate_rpm_pusher_motor(fw_throttle_input);
    
    u_fw_raw = [aileron_angle_rad elevator_angle_rad rudder_angle_rad fw_throttle_rpm];
    u_fw = interp1q(t_u_fw, u_fw_raw, t');
    
    
    %%%
    % Create state and input
    %   state structure: [att ang_vel_B vel_B] = [q0 q1 q2 q3 p q r u v w]
    %   input structure: [top_rpm_1 top_rpm_2 top_rpm_3 top_rpm_4 aileron elevator rudder pusher_rpm]
    %                       = [nt1 nt2 nt3 nt4 np delta_a delta_e delta_r]
    
    state = [q0 q1 q2 q3 p q r u v w];
    input = [u_mr u_fw];
    
end

function [ang_acc, ang_acc_raw, t_ang_acc] = read_ang_acc(csv_log_file_location, t)
    % Load data
    ang_acc_data = readtable(csv_log_file_location + '_' + "vehicle_angular_acceleration_0" + ".csv");
    
    % Read raw sensor data
    ang_acc_raw = [ang_acc_data.xyz_0_ ang_acc_data.xyz_1_ ang_acc_data.xyz_2_];
    t_ang_acc = ang_acc_data.timestamp / 1e6;
    
    % Fuse to common time horizon
    ang_acc = interp1q(t_ang_acc, ang_acc_raw, t');
end


function [sysid_indices] = get_sysid_indices(csv_log_file_location, t)
    input_rc = readtable(csv_log_file_location + '_' + "input_rc_0" + ".csv");
    
    % Extract RC sysid switch log
    sysid_rc_switch_raw = input_rc.values_6_; % sysid switch mapped to button 6
    t_rc = input_rc.timestamp / 1e6;
    RC_TRESHOLD = 1800;

    % Find times when switch was switched
    MAX_SYSID_MANEUVERS = 500;
    sysid_times = zeros(MAX_SYSID_MANEUVERS,1);
    sysid_maneuver_num = 1;
    sysid_found = false;
    for i = 1:length(t_rc)
      % Add time if found a new rising edge
      if sysid_rc_switch_raw(i) >= RC_TRESHOLD && not(sysid_found)
          sysid_times(sysid_maneuver_num) = t_rc(i);
          sysid_found = true;
          sysid_maneuver_num = sysid_maneuver_num + 1;
      end
      
      % If found a falling edge, start looking again
      if sysid_found && sysid_rc_switch_raw(i) < RC_TRESHOLD
         sysid_found = false; 
      end
    end
    
    % Plot
    if 0
        plot(t_rc, sysid_rc_switch_raw); hold on;
        plot(sysid_times, 1000, 'r*');
    end

    % Find corresponding indices in time vector
    sysid_indices = round(interp1(t,1:length(t),sysid_times));
    sysid_indices = sysid_indices(1:sysid_maneuver_num - 1);
end

function [wind_B] = get_wind_body_frame(csv_log_file_location, state, t)
    % Load data
    wind_data = readtable(csv_log_file_location + '_' + "wind_estimate_1" + ".csv");
    
     % Read raw sensor data
    t_wind = wind_data.timestamp / 1e6;
    wind_N_raw = [wind_data.windspeed_north wind_data.windspeed_east zeros(size(wind_data.windspeed_north))];
    
    wind_N = interp1q(t_wind, wind_N_raw, t');

    % Rotate to body frame
    q_NB = state(:,1:4);

    q_BN = quatinv(q_NB);
    R_BN = quat2rotm(q_BN);
    wind_B = zeros(size(wind_N));
    for i = 1:length(wind_N)
       % Notice how the Rotation matrix has to be inverted here to get the
       % right result, indicating that q is in fact q_NB and not q_BN.
       wind_B(i,:) = (R_BN(:,:,i) * wind_N(i,:)')';
    end 
end



function [acc_B, acc_B_raw, t_acc, bias_acc] = read_accelerations(csv_log_file_location, t)
    % Load data
    sensor_combined = readtable(csv_log_file_location + '_' + "sensor_combined_0" + ".csv");
    sensor_bias = readtable(csv_log_file_location + '_' + "estimator_sensor_bias_0" + ".csv");

    % Read raw sensor data
    t_acc = sensor_combined.timestamp / 1e6;
    acc_B_raw = [sensor_combined.accelerometer_m_s2_0_ sensor_combined.accelerometer_m_s2_1_ sensor_combined.accelerometer_m_s2_2_];

    % Check if significant bias
    t_sensor_bias = sensor_bias.timestamp / 1e6;
    bias_acc_raw = [sensor_bias.accel_bias_0_ sensor_bias.accel_bias_1_ sensor_bias.accel_bias_2_];
    bias_acc = interp1q(t_sensor_bias, bias_acc_raw, t');

    if 0
        figure
        subplot(2,1,1)
        plot(t, acc_B(:,1)); hold on
        plot(t, bias_acc(:,1));
        legend('acc x','bias')

        subplot(2,1,2)
        plot(t, acc_B(:,3)); hold on
        plot(t, bias_acc(:,3));
        legend('acc z','bias')
    end

    % Fuse to common time horizon
    acc_B = interp1q(t_acc, acc_B_raw, t');
end

function [acc_filtered] = filter_accelerations(t, dt, acc_raw, t_acc_raw, f_cutoff)
    % Filter data
    T_c = 1/f_cutoff;
    temp = filloutliers(t_acc_raw(2:end) - t_acc_raw(1:end-1), 'linear');
    dt_acc = mean(temp);
    alpha = dt_acc / (T_c + dt_acc);

    acc_raw_filtered = zeros(size(acc_raw));
    acc_raw_filtered(1,:) = acc_raw(1,:);
    for i = 2:length(acc_raw)
       acc_raw_filtered(i,:) = alpha * acc_raw(i,:) + (1 - alpha) * acc_raw_filtered(i-1,:);
    end

    % Frequency analysis
    if 0
        plot_fft(acc_raw, dt);
        plot_fft(acc_raw_filtered, dt);
    end

    % Fuse to common time horizon
    acc_filtered = interp1q(t_acc_raw, acc_raw_filtered, t');
end

function [] = differentiate_signal()
    % NOTE: Function not working, code snippet only kept here for reference
    % Alternative way of obtaining ang acceleration data
    Nf = 50; 
    Fpass = 10; 
    Fstop = 15;

    dt = 1/100;
    Fs = 1/dt;

    %pwelch(q,[],[],[],Fs)

    %
    d = designfilt('differentiatorfir','FilterOrder',Nf, ...
        'PassbandFrequency',Fpass,'StopbandFrequency',Fstop, ...
        'SampleRate',Fs);


    q = w_B(:,2);
    vq = filter(d,q)/dt;
    ang_acc = [zeros(length(vq),1) vq zeros(length(vq),1)];
    
end

function [V_a, V_a_validated] = read_airspeed(csv_log_file_location, t)
    airspeed_data = readtable(csv_log_file_location + '_' + "airspeed_0" + ".csv");
    airspeed_validated_data = readtable(csv_log_file_location + '_' + "airspeed_validated_0" + ".csv");

    t_airspeed = airspeed_data.timestamp / 1e6;
    V_a_raw = airspeed_data.true_airspeed_m_s;
    V_a = interp1q(t_airspeed, V_a_raw, t');

    t_airspeed_validated = airspeed_validated_data.timestamp / 1e6;
    V_a_validated_raw = airspeed_validated_data.true_airspeed_m_s;
    V_a_validated = interp1q(t_airspeed_validated, V_a_validated_raw, t');
end

function [] = plot_state_input(t, state, input, show_plot, save_plot, plot_output_location, name)        
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    
    eul = quat2eul(q_NB);
    eul_deg = rad2deg(eul);

    fig = figure;
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 1000];
    num_plots = 9; 
   
    subplot(num_plots,1,1);
    plot(t, eul_deg(:,2:3));
    legend('pitch','roll');
    title("attitude")
    
    subplot(num_plots,1,2);
    plot(t, eul_deg(:,1));
    legend('yaw');
    title("attitude")

    subplot(num_plots,1,3);
    plot(t, w_B);
    legend('p','q','r');
    max_ang_rate = max(max(max(abs(state(:,5:7)))), 1); % Never let ang rate axis be smaller than 1
    ylim([-max_ang_rate max_ang_rate])
    title("ang vel body")

    subplot(num_plots,1,4);
    plot(t, v_B(:,1));
    legend('u');
    title("vel body")
    
    subplot(num_plots,1,5);
    plot(t, v_B(:,2));
    legend('v');
    title("vel body")
    
    subplot(num_plots,1,6);
    plot(t, v_B(:,3));
    legend('w');
    title("vel body")

    subplot(num_plots,1,7);
    plot(t, u_mr);
    legend('delta_a','delta_e','delta_r', 'T_mr');
    title("mr inputs")
    
    subplot(num_plots,1,8);
    plot(t, u_fw(:,1:3));
    legend('delta_a','delta_e','delta_r');
    title("fw inputs")
    
    subplot(num_plots,1,9);
    plot(t, u_fw(:,4));
    legend('T_fw');
    title("fw inputs")
    
    figure_title = "state and input: " + name;
    sgtitle(figure_title)

    if save_plot
        filename = name + "_state_input";
        saveas(fig, plot_output_location + filename, 'epsc')
        savefig(plot_output_location + filename + '.fig')
    end
end

function [] = plot_trajectory(maneuver_index, t, state, input, show_plot, save_plot, plot_output_location)        
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    
    eul = quat2eul(q_NB);
    eul_deg = rad2deg(eul);

    fig = figure;
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 2000];
    num_plots = 10;
    
    subplot(num_plots,1,1);
    plot(t, eul_deg(:,3));
    legend('roll');
    title("attitude")
    
    
    subplot(num_plots,1,2);
    plot(t, eul_deg(:,2));
    legend('pitch');
    title("attitude")
    

    subplot(num_plots,1,3);
    plot(t, eul_deg(:,1));
    legend('yaw');
    title("attitude")

    subplot(num_plots,1,4);
    plot(t, w_B(:,1));
    legend('p')
    max_ang_rate = 0.8;
    ylim([-max_ang_rate max_ang_rate])
    title("ang vel body")
    
    subplot(num_plots,1,5);
    plot(t, w_B(:,2));
    legend('q')
    max_ang_rate = 0.8;
    ylim([-max_ang_rate max_ang_rate])
    title("ang vel body")
    
    subplot(num_plots,1,6);
    plot(t, w_B(:,3));
    legend('r')
    max_ang_rate = 0.8;
    ylim([-max_ang_rate max_ang_rate])
    title("ang vel body")

    subplot(num_plots,1,7);
    plot(t, v_B(:,1)); hold on
    legend('u');
    title("vel body")
    
    subplot(num_plots,1,8);
    plot(t, v_B(:,2:3)); hold on;
    legend('v','w');
    title("vel body")

    subplot(num_plots,1,9);
    plot(t, u_fw(:,1:3));
    legend('delta_a','delta_e','delta_r');
    title("inputs")
    
    subplot(num_plots,1,10);
    plot(t, u_fw(:,4));
    legend('T_fw');
    title("inputs")
    
    figure_title = "state and input, maneuver: " + maneuver_index;
    sgtitle(figure_title)

    if save_plot
        filename = maneuver_index + "_state_input";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end

function [] = plot_trajectory_static_details(maneuver_index, t, state, input, AoA, ...
    acc_B, acc_B_filtered, bias_acc, acc_N, ...
    show_plot, save_plot, plot_output_location)
    
% Figure details
    fig = figure; 
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 1000];
    num_plots = 5; 

    % Extract all state values
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    V = sqrt(sum(v_B.^2'));

    eul = quat2eul(q_NB);
    eul_deg = rad2deg(eul);
    
    subplot(num_plots,1,1);
    plot(t, AoA);
    legend('AoA');
    title("Angle of Attack")

    subplot(num_plots,1,2);
    plot(t, V);
    legend('V body');
    title("Airspeed (assuming no wind)")

    subplot(num_plots,1,3);
    plot(t, acc_B(:,1)); hold on
    plot(t, acc_B_filtered(:,1)); hold on
    plot(t, bias_acc(:,1));
    legend('acc B', 'acc B filtered', 'bias');
    title("a_x")

    subplot(num_plots,1,4);
    plot(t, acc_B(:,3)); hold on
    plot(t, acc_B_filtered(:,3));
    plot(t, bias_acc(:,3));
    legend('acc B', 'acc B filtered', 'bias');
    title("a_z")

    subplot(num_plots,1,5);
    g = 9.81;
    plot(t, abs(acc_N(:,3) + g));
    title("acc z compared to 1 g")
    
    figure_title = "Lift and drag details, maneuver: " + maneuver_index;
    sgtitle(figure_title)
    
    if save_plot
        filename = maneuver_index + "_details";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end

function [] = plot_accelerations(maneuver_index, t, acc_B, acc_B_unfiltered, bias_acc, ang_acc, ang_acc_unfiltered, show_plot, save_plot, plot_output_location)
    % Figure details
    fig = figure; 
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 1000];
    num_plots = 6; 
    
    subplot(num_plots,1,1);
    plot(t, acc_B_unfiltered(:,1)); hold on
    plot(t, acc_B(:,1)); hold on
    plot(t, bias_acc(:,1));
    legend('a_x', 'a_x (filtered)', 'bias');
    title("Linear acceleration")
    
    subplot(num_plots,1,2);
    plot(t, acc_B_unfiltered(:,2)); hold on
    plot(t, acc_B(:,2)); hold on
    plot(t, bias_acc(:,2));
    legend('a_y', 'a_y (filtered)', 'bias');
    title("Linear acceleration")
    
    subplot(num_plots,1,3);
    plot(t, acc_B_unfiltered(:,3)); hold on
    plot(t, acc_B(:,3)); hold on
    plot(t, bias_acc(:,3));
    legend('a_z', 'a_z (filtered)', 'bias');
    title("Linear acceleration")

    subplot(num_plots,1,4);
    plot(t, ang_acc_unfiltered(:,1)); hold on
    plot(t, ang_acc(:,1));
    legend('a_x', 'a_x (filtered)');
    title("Angular acceleration")
    
    subplot(num_plots,1,5);
    plot(t, ang_acc_unfiltered(:,2)); hold on
    plot(t, ang_acc(:,2));
    legend('a_y', 'a_y (filtered)');
    title("Angular acceleration")
    
    subplot(num_plots,1,6);
    plot(t, ang_acc_unfiltered(:,3)); hold on
    plot(t, ang_acc(:,3));
    legend('a_z', 'a_z (filtered)');
    title("Angular acceleration")

    figure_title = "Accelerations, maneuver: " + maneuver_index;
    sgtitle(figure_title)
    
    if save_plot
        filename = maneuver_index + "_accelerations";
        saveas(fig, plot_output_location + filename, 'epsc')
    end
end


function [] = plot_wind(maneuver_index, t, wind_B, ...
    show_plot, save_plot, plot_output_location)
    
    % Figure details
    fig = figure; 
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 600];

    plot(t, wind_B);
    legend('wind_x', 'wind_y', 'wind_z');
    
    figure_title = "estimated wind, maneuver: " + maneuver_index;
    title(figure_title)
    
    if save_plot
        filename = maneuver_index + "_wind";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end

function [] = scatter_static_curves(maneuver_index, t, AoA_deg, L, D, Tau_y, c_L, c_D, c_m, show_plot, save_plot, plot_output_location)
    % Figure details
    fig = figure; 
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 1000];

    subplot(3,2,1)
    scatter(AoA_deg, L, [], t)
    xlabel('AoA [deg]')
    ylabel('L')
    title("Lift")
    colorbar
    
    subplot(3,2,2)
    scatter(AoA_deg, c_L, [], t)
    xlabel('AoA [deg]')
    ylabel('c_L')
    title("Lift coeff")
    colorbar
    
    subplot(3,2,3)
    scatter(AoA_deg, D, [], t)
    xlabel('AoA [deg]')
    ylabel('D')
    title("Drag")
    colorbar
    
    subplot(3,2,4)
    scatter(AoA_deg, c_D, [], t)
    xlabel('AoA [deg]')
    ylabel('c_D')
    title("Drag coeff")
    colorbar
        
    subplot(3,2,5)
    scatter(AoA_deg, Tau_y, [], t)
    xlabel('AoA [deg]')
    ylabel('Tau_y')
    title("Pitch moment")
    colorbar
     
    subplot(3,2,6)
    scatter(AoA_deg, c_m, [], t)
    xlabel('AoA [deg]')
    ylabel('c_m')
    title("Pitch moment coeff")
    colorbar
    
    figure_title = "static curves, maneuver: " + maneuver_index;
    sgtitle(figure_title)
    
    if save_plot
        filename = maneuver_index + "_static_curves";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end


function [maneuver_start_index] = find_exact_start_index_sweep_maneuver(input, guess_sysid_index, dt, maneuver_period)
    % Find correct maneuver start index
    time_to_search_before_maneuver = 1; %s
    time_to_search_after_maneuver_start = 3; %s

    indices_to_search_before_maneuver = time_to_search_before_maneuver / dt;
    indices_to_search_after_maneuver_start = time_to_search_after_maneuver_start / dt;
    max_start_index = guess_sysid_index - indices_to_search_before_maneuver;
    max_end_index = guess_sysid_index + indices_to_search_after_maneuver_start;
    % Move to correct start index
    for j = 1:maneuver_period/dt
        [~, maneuver_top_index] = max(input(max_start_index:max_end_index));
        maneuver_top_index = max_start_index + maneuver_top_index; % What is this line doing??
    end
    
    maneuver_start_index = maneuver_top_index - maneuver_period / dt;
end


% TODO
function [maneuver_passed_data_test] = does_data_pass_validation_tests(state_maneuver, airspeed_maneuver, maneuver_index, AIRSPEED_TRESHOLD_MIN, AIRSPEED_TRESHOLD_MAX)  
    q_NB = state_maneuver(:,1:4);
    w_B = state_maneuver(:,5:7);
    v_B = state_maneuver(:,8:10);
    u_mr = state_maneuver(:,1:4);
    u_fw = state_maneuver(:,5:8);
    
    maneuver_passed_data_test = true;    
    if (airspeed_maneuver(1) < AIRSPEED_TRESHOLD_MIN)
       display("skipping maneuver " + maneuver_index + ": airspeed too low");
       maneuver_passed_data_test = false;
    end
    if (airspeed_maneuver(1) > AIRSPEED_TRESHOLD_MAX)
       display("skipping maneuver " + maneuver_index + ": airspeed too high");
       maneuver_passed_data_test = false;
    end
    
    % Calculate data cleanliness
    std_ang_rates = std(w_B);
    std_body_vel = std(v_B);
    total_std = std_ang_rates(1) + std_ang_rates(3 ) + std_body_vel(2);
    
    if (total_std > 0.7)
       display("skipping maneuver " + maneuver_index + ": total std: " + total_std);
       maneuver_passed_data_test = false;
    end
end

function [acc_N] = calculate_acc_N(acc_B, q_NB)
    R_NB = quat2rotm(q_NB);
    % Calculate NED acceleration
    acc_N = zeros(size(acc_B));
    for i = 1:length(acc_B)
       acc_N(i,:) = (R_NB(:,:,i) * acc_B(i,:)')';
    end
end

function [] = plot_fft(data, dt)
    % Frequency analysis
    figure
    Y = fft(data);
    L = length(data);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    Fs = 1/dt;
    f = Fs*(0:(L/2))/L;
    plot(f,P1)
    title('Single-Sided Amplitude Spectrum of X(t)')
    xlabel('f (Hz)')
    ylabel('|P1(f)|')
end