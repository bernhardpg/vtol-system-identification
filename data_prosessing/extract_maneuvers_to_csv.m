clc; clear all; close all;

% This script parses ulogs (already converted to .csv format) from "raw_data/log_files/csv/*"
% according to metadata.json, and saves the relevant data in "raw_data/experiments/*"
% as .csv files

metadata_filename = "data/flight_data/metadata.json";
metadata = read_metadata(metadata_filename);

% Output data
save_output_data = true;
show_plot = false;

% Experiments
num_experiments = length(metadata.Experiments);
experiments_to_parse = 1:num_experiments;
maneuver_types_to_parse = [
    "pitch_211","pitch_211_no_throttle",...
    ];
    %"freehand",...
    %"roll_211","roll_211_no_throttle",...
    %"pitch_211","pitch_211_no_throttle",...
    %"yaw_211","yaw_211_no_throttle",...
    %];

for i = experiments_to_parse
    parse_experiment_data(metadata.Experiments(i), save_output_data, maneuver_types_to_parse);
end

%% Functions

function [t, q_NB, v_N] = read_attitude_and_vel(csv_log_file_location)
    ekf_data = readtable(csv_log_file_location + '_' + "estimator_status_0" + ".csv");
    
    %%%
    % Extract data from ekf2
    % NOTE: EKF2 runs at a delayed time horizon, determined by the largest
    % EKF2_*_DELAY parameter. This is currently set to 175 ms.
    ekf2_delay_s = 0.175;

    t = ekf_data.timestamp / 1e6 - ekf2_delay_s;

    % q_NB = unit quaternion describing vector rotation from NED to Body. i.e.
    % describes transformation from Body to NED frame.
    % Note: This is the same as the output_predictor quaternion. Something is
    % wrong with PX4 documentation

    q0 = ekf_data.states_0_;
    q1 = ekf_data.states_1_;
    q2 = ekf_data.states_2_;
    q3 = ekf_data.states_3_;
    q_NB = [q0 q1 q2 q3];

    v_n = ekf_data.states_4_;
    v_e = ekf_data.states_5_;
    v_d = ekf_data.states_6_;
    v_N = [v_n v_e v_d];
end


function [t_u_mr, u_mr, t_u_fw, u_fw] = read_input(csv_log_file_location)
    actuator_controls_mr = readtable(csv_log_file_location + '_' + "actuator_controls_0_0" + ".csv");

    t_u_mr = actuator_controls_mr.timestamp / 1e6;
    u_roll_mr = actuator_controls_mr.control_0_;
    u_pitch_mr = actuator_controls_mr.control_1_;
    u_yaw_mr = actuator_controls_mr.control_2_;
    u_throttle_mr = actuator_controls_mr.control_3_;
    u_mr = [u_roll_mr u_pitch_mr u_yaw_mr u_throttle_mr];
    % TODO: Convert these from PX4 inputs to actual RPMs of each motor

    actuator_controls_fw = readtable(csv_log_file_location + '_' + "actuator_controls_1_0" + ".csv");
    t_u_fw = actuator_controls_fw.timestamp / 1e6;
    % Inputs in [0,1] interval
    fw_aileron_input = actuator_controls_fw.control_0_;
    fw_elevator_input = actuator_controls_fw.control_1_;
    fw_rudder_input = actuator_controls_fw.control_2_;
    fw_throttle_input = actuator_controls_fw.control_3_;
    
    % Convert from [0,1] interval to angles
    [aileron_angle_rad, elevator_angle_rad, rudder_angle_rad] = ...
        calculate_control_surface_angles_rad(fw_aileron_input, fw_elevator_input, fw_rudder_input);
    [fw_throttle_rev_per_s] = calculate_rev_per_s_pusher_motor(fw_throttle_input);
    
    u_fw = [aileron_angle_rad elevator_angle_rad rudder_angle_rad fw_throttle_rev_per_s];
end

function [] = parse_experiment_data(experiment, save_output_data, maneuver_types_to_parse)
    exp_num = experiment.Number;
    log_file = experiment.LogName;
    maneuver_metadata = experiment.Maneuvers;
    num_maneuvers = length(fieldnames(maneuver_metadata));
    
    csv_files_location = "data/flight_data/raw_data/log_files/csv/";
    csv_log_file_location = csv_files_location + log_file;

    % Read state and input data
    [t_state, q_NB, v_N] = read_attitude_and_vel(csv_log_file_location);
    %dt = mean(rmoutliers(t_state(2:end) - t_state(1:end-1))); % Calculate dt for conversion between index and time
    [t_u_mr, u_mr, t_u_fw, u_fw] = read_input(csv_log_file_location);
    
    % Get system identification maneuvers
    sysid_times_s = get_sysid_times(csv_log_file_location, t_state);
    
    % If no maneuvers in metadata, use sysid switch to get number of
    % maneuvers
    if num_maneuvers == 0
       num_maneuvers = length(sysid_times_s); 
    end
    
    % Initialize empty output variables
    output_data = intialize_empty_output_struct();

    % Iterate through maneuvers
    num_aggregated_maneuvers = 0;
    for maneuver_i = 1:num_maneuvers
        % No metadata available
        if isempty(fieldnames(maneuver_metadata))
            curr_maneuver_metadata = {};
            curr_maneuver_metadata.end_s = -1;
            curr_maneuver_metadata.start_s = -1;
            curr_maneuver_metadata.type = "not_set";
        else
            % Read maneuver metadata
            curr_maneuver_name = "x" + string(maneuver_i);
            curr_maneuver_metadata = maneuver_metadata.(curr_maneuver_name);
        end
        
        if ~ismember(curr_maneuver_metadata.type, maneuver_types_to_parse)
            continue;
        end
        
        [maneuver_length_s] = get_default_maneuver_length_s(curr_maneuver_metadata.type);
        [padding_before_s, padding_after_s] = get_default_maneuver_padding_s(curr_maneuver_metadata.type);
            
        if maneuver_i > length(sysid_times_s)
            maneuver_start_guess_s = -1; % this will not get used
        else
            maneuver_start_guess_s = sysid_times_s(maneuver_i);
        end
        
        [maneuver_start_s, maneuver_end_s] = ...
            get_maneuver_start_end_time(...
                curr_maneuver_metadata, padding_before_s, padding_after_s, t_state, maneuver_start_guess_s, maneuver_length_s ...
            );

        % Find maneuver indices for state
        maneuver_start_index_state = floor(interp1(t_state, 1:length(t_state), maneuver_start_s)) + 1;
        maneuver_end_index_state = floor(interp1(t_state, 1:length(t_state), maneuver_end_s)) - 1;
        
        % Interpolate start and end time, to make sure that we save values
        % for start and end of maneuver
        t_state_maneuver = [maneuver_start_s;
                            t_state(maneuver_start_index_state:maneuver_end_index_state);
                            maneuver_end_s];
        q_NB_maneuver = [interp1(t_state, q_NB, maneuver_start_s);
                         q_NB(maneuver_start_index_state:maneuver_end_index_state,:);
                         interp1(t_state, q_NB, maneuver_end_s)];
        v_N_maneuver = [interp1(t_state, v_N, maneuver_start_s);
                        v_N(maneuver_start_index_state:maneuver_end_index_state,:);
                        interp1(t_state, v_N, maneuver_end_s)];
        
        % Extract input data
        % we dont care about MR inputs yet
        %u_mr_maneuver = u_mr(maneuver_start_index:maneuver_end_index,:);
        
        maneuver_start_index_u_fw = floor(interp1(t_u_fw, 1:length(t_u_fw), maneuver_start_s)) + 1;
        maneuver_end_index_u_fw = floor(interp1(t_u_fw, 1:length(t_u_fw), maneuver_end_s)) - 1;
        
        t_u_fw_maneuver = [maneuver_start_s;
                         t_u_fw(maneuver_start_index_u_fw:maneuver_end_index_u_fw);
                         maneuver_end_s];
        % Interpolate start and end time, to make sure that we save values
        % for start and end of maneuver
        u_fw_maneuver = [interp1(t_u_fw, u_fw, maneuver_start_s);
                         u_fw(maneuver_start_index_u_fw:maneuver_end_index_u_fw,:);
                         interp1(t_u_fw, u_fw, maneuver_end_s)];
        
        % Store data in aggregated data matrices
        curr_maneuver_state_index_in_aggregation_matrix = length(output_data.(curr_maneuver_metadata.type).t_state) + 1;
        curr_maneuver_u_fw_index_in_aggregation_matrix = length(output_data.(curr_maneuver_metadata.type).t_u_fw) + 1;

        output_data.(curr_maneuver_metadata.type).maneuver_start_indices_state =...
            [output_data.(curr_maneuver_metadata.type).maneuver_start_indices_state...
             curr_maneuver_state_index_in_aggregation_matrix];
        output_data.(curr_maneuver_metadata.type).maneuver_start_indices_u_fw =...
            [output_data.(curr_maneuver_metadata.type).maneuver_start_indices_u_fw...
             curr_maneuver_u_fw_index_in_aggregation_matrix];

        output_data.(curr_maneuver_metadata.type).t_state = ...
            [output_data.(curr_maneuver_metadata.type).t_state;
             t_state_maneuver];
        output_data.(curr_maneuver_metadata.type).q_NB = ...
            [output_data.(curr_maneuver_metadata.type).q_NB;
             q_NB_maneuver];
        output_data.(curr_maneuver_metadata.type).v_N = ...
            [output_data.(curr_maneuver_metadata.type).v_N;
             v_N_maneuver];
        output_data.(curr_maneuver_metadata.type).u_mr = ...
            [output_data.(curr_maneuver_metadata.type).u_mr;
             ];
        output_data.(curr_maneuver_metadata.type).t_u_fw = ...
            [output_data.(curr_maneuver_metadata.type).t_u_fw;
             t_u_fw_maneuver];
        output_data.(curr_maneuver_metadata.type).u_fw = ...
            [output_data.(curr_maneuver_metadata.type).u_fw;
             u_fw_maneuver];

        output_data.(curr_maneuver_metadata.type).aggregated_maneuvers = ...
            [output_data.(curr_maneuver_metadata.type).aggregated_maneuvers maneuver_i];
        num_aggregated_maneuvers = num_aggregated_maneuvers + 1;
    end

    disp("Exp: " + exp_num + ": " + "Succesfully aggregated " + num_aggregated_maneuvers + " maneuvers");

    % Save to files
    if save_output_data
        save_output(experiment.Number, output_data)
    end
end

function [maneuver_length_s] = get_default_maneuver_length_s(maneuver_type) 
        if maneuver_type == "sweep"
            maneuver_length_s = 8;
        elseif (maneuver_type == "roll_211") || (maneuver_type == "roll_211_no_throttle")
            maneuver_length_s = 3;
        elseif (maneuver_type == "pitch_211") || (maneuver_type == "pitch_211_no_throttle")
            maneuver_length_s = 3;
        elseif (maneuver_type == "yaw_211") || (maneuver_type == "yaw_211_no_throttle")
            maneuver_length_s = 5.5;
        else
            maneuver_length_s = 6;
        end
end


function [padding_start_s, padding_end_s] = get_default_maneuver_padding_s(maneuver_type) 
        if maneuver_type == "sweep"
            padding_start_s = 1;
            padding_end_s = 1;
        elseif (maneuver_type == "pitch_211") || (maneuver_type == "pitch_211_no_throttle")
            padding_start_s = 2;
            padding_end_s = 2;
        else
            padding_start_s = 2;
            padding_end_s = 2;
        end
end

function [] = save_output(experiment_number, output_data)
        experiment_location = "data/flight_data/raw_data/experiments/";

        if ~isempty(output_data.sweep.t_state)
            data_output_location = experiment_location + "experiment_" + string(experiment_number) ...
                + "/sweep/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.sweep.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.sweep.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.sweep.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.sweep.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.sweep.t_u_fw, data_output_location + 't_u_fw.csv');  
            writematrix(output_data.sweep.u_fw, data_output_location + 'u_fw.csv');
 
            writematrix(output_data.sweep.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.sweep.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.sweep.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.roll_211.t_state)
            data_output_location = experiment_location + "experiment_" + string(experiment_number) ...
                + "/roll_211/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.roll_211.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.roll_211.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.roll_211.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.roll_211.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.roll_211.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.roll_211.t_u_fw, data_output_location + 't_u_fw.csv');  
 
            writematrix(output_data.roll_211.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.roll_211.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.roll_211.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.roll_211_no_throttle.t_state)
            data_output_location = experiment_location + "experiment_" + string(experiment_number) ...
                + "/roll_211_no_throttle/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.roll_211_no_throttle.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.roll_211_no_throttle.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.roll_211_no_throttle.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.roll_211_no_throttle.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.roll_211_no_throttle.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.roll_211_no_throttle.t_u_fw, data_output_location + 't_u_fw.csv');  
            
            writematrix(output_data.roll_211_no_throttle.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.roll_211_no_throttle.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.roll_211_no_throttle.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.pitch_211.t_state)
            data_output_location = experiment_location + "experiment_" + string(experiment_number) ...
                + "/pitch_211/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.pitch_211.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.pitch_211.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.pitch_211.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.pitch_211.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.pitch_211.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.pitch_211.t_u_fw, data_output_location + 't_u_fw.csv');  
            
            writematrix(output_data.pitch_211.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.pitch_211.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.pitch_211.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.pitch_211_no_throttle.t_state)
            data_output_location = experiment_location + "experiment_" + string(experiment_number) ...
                + "/pitch_211_no_throttle/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.pitch_211_no_throttle.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.pitch_211_no_throttle.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.pitch_211_no_throttle.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.pitch_211_no_throttle.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.pitch_211_no_throttle.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.pitch_211_no_throttle.t_u_fw, data_output_location + 't_u_fw.csv');  
            
            writematrix(output_data.pitch_211_no_throttle.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.pitch_211_no_throttle.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.pitch_211_no_throttle.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.yaw_211.t_state)
            data_output_location = experiment_location + "experiment_" + string(experiment_number) ...
                + "/yaw_211/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.yaw_211.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.yaw_211.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.yaw_211.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.yaw_211.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.yaw_211.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.yaw_211.t_u_fw, data_output_location + 't_u_fw.csv');  
            
            writematrix(output_data.yaw_211.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.yaw_211.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.yaw_211.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.yaw_211_no_throttle.t_state)
            data_output_location = experiment_location + "experiment_" + string(experiment_number) ...
                + "/yaw_211_no_throttle/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.yaw_211_no_throttle.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.yaw_211_no_throttle.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.yaw_211_no_throttle.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.yaw_211_no_throttle.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.yaw_211_no_throttle.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.yaw_211_no_throttle.t_u_fw, data_output_location + 't_u_fw.csv');  
            
            writematrix(output_data.yaw_211_no_throttle.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.yaw_211_no_throttle.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.yaw_211_no_throttle.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.freehand.t_state)
            data_output_location = experiment_location + "experiment_" + string(experiment_number) ...
                + "/freehand/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.freehand.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.freehand.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.freehand.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.freehand.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.freehand.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.freehand.t_u_fw, data_output_location + 't_u_fw.csv');  
            
            writematrix(output_data.freehand.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.freehand.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.freehand.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.cruise.t_state)
            data_output_location = experiment_location + "experiment_" + string(experiment_number) ...
                + "/cruise/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.cruise.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.cruise.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.cruise.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.cruise.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.cruise.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.cruise.t_u_fw, data_output_location + 't_u_fw.csv');  
            
            writematrix(output_data.cruise.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.cruise.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.cruise.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
end

function [output_struct] = intialize_empty_output_struct()
    output_struct.sweep = {};

    output_struct.sweep.maneuver_start_indices_state = [];
    output_struct.sweep.maneuver_start_indices_u_fw = [];
    output_struct.sweep.t_state = [];
    output_struct.sweep.t_u_fw = [];
    output_struct.sweep.q_NB = [];
    output_struct.sweep.v_N = [];
    output_struct.sweep.u_mr = [];
    output_struct.sweep.u_fw = [];
    output_struct.sweep.aggregated_maneuvers = [];

    output_struct.roll_211.maneuver_start_indices_state = [];
    output_struct.roll_211.maneuver_start_indices_u_fw = [];
    output_struct.roll_211.t_state = [];
    output_struct.roll_211.t_u_fw = [];
    output_struct.roll_211.q_NB = [];
    output_struct.roll_211.v_N = [];
    output_struct.roll_211.u_mr = [];
    output_struct.roll_211.u_fw = [];
    output_struct.roll_211.aggregated_maneuvers = [];
    
    output_struct.roll_211_no_throttle.maneuver_start_indices_state = [];
    output_struct.roll_211_no_throttle.maneuver_start_indices_u_fw = [];
    output_struct.roll_211_no_throttle.t_state = [];
    output_struct.roll_211_no_throttle.t_u_fw = [];
    output_struct.roll_211_no_throttle.q_NB = [];
    output_struct.roll_211_no_throttle.v_N = [];
    output_struct.roll_211_no_throttle.u_mr = [];
    output_struct.roll_211_no_throttle.u_fw = [];
    output_struct.roll_211_no_throttle.aggregated_maneuvers = [];

    output_struct.pitch_211.maneuver_start_indices_state = [];
    output_struct.pitch_211.maneuver_start_indices_u_fw = [];
    output_struct.pitch_211.t_state = [];
    output_struct.pitch_211.t_u_fw = [];
    output_struct.pitch_211.q_NB = [];
    output_struct.pitch_211.v_N = [];
    output_struct.pitch_211.u_mr = [];
    output_struct.pitch_211.u_fw = [];
    output_struct.pitch_211.aggregated_maneuvers = [];

    output_struct.pitch_211_no_throttle.maneuver_start_indices_state = [];
    output_struct.pitch_211_no_throttle.maneuver_start_indices_u_fw = [];
    output_struct.pitch_211_no_throttle.t_state = [];
    output_struct.pitch_211_no_throttle.t_u_fw = [];
    output_struct.pitch_211_no_throttle.q_NB = [];
    output_struct.pitch_211_no_throttle.v_N = [];
    output_struct.pitch_211_no_throttle.u_mr = [];
    output_struct.pitch_211_no_throttle.u_fw = [];
    output_struct.pitch_211_no_throttle.aggregated_maneuvers = [];
    
    output_struct.yaw_211.maneuver_start_indices_state = [];
    output_struct.yaw_211.maneuver_start_indices_u_fw = [];
    output_struct.yaw_211.t_state = [];
    output_struct.yaw_211.t_u_fw = [];
    output_struct.yaw_211.q_NB = [];
    output_struct.yaw_211.v_N = [];
    output_struct.yaw_211.u_mr = [];
    output_struct.yaw_211.u_fw = [];
    output_struct.yaw_211.aggregated_maneuvers = [];
    
    output_struct.yaw_211_no_throttle.maneuver_start_indices_state = [];
    output_struct.yaw_211_no_throttle.maneuver_start_indices_u_fw = [];
    output_struct.yaw_211_no_throttle.t_state = [];
    output_struct.yaw_211_no_throttle.t_u_fw = [];
    output_struct.yaw_211_no_throttle.q_NB = [];
    output_struct.yaw_211_no_throttle.v_N = [];
    output_struct.yaw_211_no_throttle.u_mr = [];
    output_struct.yaw_211_no_throttle.u_fw = [];
    output_struct.yaw_211_no_throttle.aggregated_maneuvers = [];
    
    output_struct.freehand.maneuver_start_indices_state = [];
    output_struct.freehand.maneuver_start_indices_u_fw = [];
    output_struct.freehand.t_state = [];
    output_struct.freehand.t_u_fw = [];
    output_struct.freehand.q_NB = [];
    output_struct.freehand.v_N = [];
    output_struct.freehand.u_mr = [];
    output_struct.freehand.u_fw = [];
    output_struct.freehand.aggregated_maneuvers = [];
    
    output_struct.cruise.maneuver_start_indices_state = [];
    output_struct.cruise.maneuver_start_indices_u_fw = [];
    output_struct.cruise.t_state = [];
    output_struct.cruise.t_u_fw = [];
    output_struct.cruise.q_NB = [];
    output_struct.cruise.v_N = [];
    output_struct.cruise.u_mr = [];
    output_struct.cruise.u_fw = [];
    output_struct.cruise.aggregated_maneuvers = [];
    
    output_struct.not_set.maneuver_start_indices_state = [];
    output_struct.not_set.maneuver_start_indices_u_fw = [];
    output_struct.not_set.t_state = [];
    output_struct.not_set.t_u_fw = [];
    output_struct.not_set.q_NB = [];
    output_struct.not_set.v_N = [];
    output_struct.not_set.u_mr = [];
    output_struct.not_set.u_fw = [];
    output_struct.not_set.aggregated_maneuvers = [];


end

function [maneuver_start_s, maneuver_end_s]...
    = get_maneuver_start_end_time(...
        curr_maneuver_metadata, default_maneuver_padding_start_s, default_maneuver_padding_end_s, t, start_time_s, maneuver_length_s ...
    )
        % Use sysid index for maneuver if no maneuver start and end time is
        % set in metadata
        maneuver_start_time_not_set = curr_maneuver_metadata.start_s == -1;
        maneuver_end_time_not_set = curr_maneuver_metadata.end_s == -1;
        
        % Use default maneuver start or end if not set, or if maneuver
        % should not be aggregated
        if maneuver_start_time_not_set
            maneuver_start_s = max(...
                [start_time_s - default_maneuver_padding_start_s...
                 t(1)]); % never start maneuver before first timestamp
        else
            maneuver_start_s = curr_maneuver_metadata.start_s;
        end
        if maneuver_end_time_not_set
            maneuver_end_s = max(...
                [maneuver_start_s + maneuver_length_s + default_maneuver_padding_start_s + default_maneuver_padding_end_s...
                 t(1)]); % do not end maneuver before first timestamp
        else
            maneuver_end_s = curr_maneuver_metadata.end_s;
        end
end


function [sysid_times_s] = get_sysid_times(csv_log_file_location, t)
    input_rc = readtable(csv_log_file_location + '_' + "input_rc_0" + ".csv");
    
    % Extract RC sysid switch log
    sysid_rc_switch_raw = input_rc.values_6_; % sysid switch mapped to button 6
    t_rc = input_rc.timestamp / 1e6;
    RC_TRESHOLD = 1800;

    % Find times when switch was switched
    MAX_SYSID_MANEUVERS = 500;
    sysid_times_s = zeros(MAX_SYSID_MANEUVERS,1);
    sysid_maneuver_num = 1;
    sysid_found = false;
    for i = 1:length(t_rc)
      % Add time if found a new rising edge
      if sysid_rc_switch_raw(i) >= RC_TRESHOLD && not(sysid_found)
          sysid_times_s(sysid_maneuver_num) = t_rc(i);
          sysid_found = true;
          sysid_maneuver_num = sysid_maneuver_num + 1;
      end
      
      % If found a falling edge, start looking again
      if sysid_found && sysid_rc_switch_raw(i) < RC_TRESHOLD
         sysid_found = false; 
      end
    end
    
    sysid_times_s = sysid_times_s(1:sysid_maneuver_num - 1); % cut away zeros
end