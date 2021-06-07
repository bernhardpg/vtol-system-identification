clc; clear all; close all;

metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Output data
save_output_data = true;
show_plot = false;

% Experiments
num_experiments = length(metadata.Experiments);
%experiments_to_parse = 1:num_experiments;
experiments_to_parse = [1:6];
maneuver_types_to_parse = [...
    %"roll_211", "roll_211_no_throttle",...
    "pitch_211", %"pitch_211_no_throttle",...
    %"yaw_211", "yaw_211_no_throttle",...
    %"freehand", "cruise",...
    %"sweep",...
    %"not_set"
    ];

for i = experiments_to_parse
    parse_maneuver_new(metadata.Experiments(i), save_output_data, maneuver_types_to_parse);
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
    % wrong with documentation

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

function [] = parse_maneuver_new(experiment, save_output_data, maneuver_types_to_parse)
    exp_num = experiment.Number;
    log_file = experiment.LogName;
    maneuver_metadata = experiment.Maneuvers;
    num_maneuvers = length(fieldnames(maneuver_metadata));
    
    csv_files_location = "data/log_files/csv/";
    csv_log_file_location = csv_files_location + log_file;

    % Read state and input data
    [t_state, q_NB, v_N] = read_attitude_and_vel(csv_log_file_location);
    dt = mean(rmoutliers(t_state(2:end) - t_state(1:end-1))); % Calculate dt for conversion between index and time
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
            
        maneuver_start_guess_s = sysid_times_s(maneuver_i);
        
        [maneuver_should_be_aggregated, maneuver_start_s, maneuver_end_s] = ...
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
        u_fw_maneuver = [interp1(t_u_fw, u_fw, maneuver_start_s);
                         u_fw(maneuver_start_index_u_fw:maneuver_end_index_u_fw,:);
                         interp1(t_u_fw, u_fw, maneuver_end_s)];
        
        % Store data in aggregated data matrices
        if maneuver_should_be_aggregated
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
    end

    disp("Exp: " + exp_num + ": " + "Succesfully aggregated " + num_aggregated_maneuvers + " maneuvers");

    % Save to files
    if save_output_data
        save_output(experiment.Number, output_data)
    end
end




function [] = animate_forces(dt, L, D, AoA_rad)
    for i = 1:length(L)
        alpha = AoA_rad(i);
        
        D_x = -D(i) * cos(alpha);
        D_z = D(i) * sin(alpha);
        L_x = -L(i) * sin(alpha);
        L_z = -L(i) * cos(alpha);
        
        quiver(0,0,L_x,-L_z); hold on
        quiver(0,0,D_x,-D_z);
        quiver(0,0,30*cos(alpha),30*sin(alpha),"ShowArrowHead",'off','LineWidth',1.5);
        hold off
        axis equal;
        xlim([-100 100])
        ylim([-100 100])
        pause(dt)
    end
end

function [maneuver_length_s] = get_default_maneuver_length_s(maneuver_type) 
        if maneuver_type == "sweep"
            maneuver_length_s = 4;
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
            padding_end_s = 0;
        elseif (maneuver_type == "pitch_211") || (maneuver_type == "pitch_211_no_throttle")
            padding_start_s = 2;
            padding_end_s = 2;
        else
            padding_start_s = 2;
            padding_end_s = 2;
        end
end

function [] = save_output(experiment_number, output_data)
        if ~isempty(output_data.sweep.t_state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/sweep/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.sweep.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.sweep.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.sweep.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.sweep.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.sweep.t, data_output_location + 't_u_fw.csv');  
            writematrix(output_data.sweep.u_fw, data_output_location + 'u_fw.csv');
 
            writematrix(output_data.sweep.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.sweep.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.sweep.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.roll_211.t_state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/roll_211/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.roll_211.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.roll_211.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.roll_211.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.roll_211.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.roll_211.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.roll_211.t, data_output_location + 't_u_fw.csv');  
 
            writematrix(output_data.roll_211.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.roll_211.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.roll_211.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.roll_211_no_throttle.t_state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
                + "/roll_211_no_throttle/output/";
            mkdir(data_output_location);
            
            writematrix(output_data.roll_211_no_throttle.t_state, data_output_location + 't_state.csv');   
            writematrix(output_data.roll_211_no_throttle.q_NB, data_output_location + 'q_NB.csv');
            writematrix(output_data.roll_211_no_throttle.v_N, data_output_location + 'v_N.csv');
            writematrix(output_data.roll_211_no_throttle.u_mr, data_output_location + 'u_mr.csv');
            writematrix(output_data.roll_211_no_throttle.u_fw, data_output_location + 'u_fw.csv');
            writematrix(output_data.roll_211.t_u_fw, data_output_location + 't_u_fw.csv');  
            
            writematrix(output_data.roll_211_no_throttle.maneuver_start_indices_state, data_output_location + 'maneuver_start_indices_state.csv');
            writematrix(output_data.roll_211_no_throttle.maneuver_start_indices_u_fw, data_output_location + 'maneuver_start_indices_u_fw.csv');
            writematrix(output_data.roll_211_no_throttle.aggregated_maneuvers, data_output_location + 'aggregated_maneuvers.csv');
        end
        
        if ~isempty(output_data.pitch_211.t_state)
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
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
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
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
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
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
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
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
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
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
            data_output_location = "data/experiments/experiment_" + string(experiment_number) ...
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

function [maneuver_should_be_aggregated, maneuver_start_s, maneuver_end_s]...
    = get_maneuver_start_end_time(...
        curr_maneuver_metadata, default_maneuver_padding_start_s, default_maneuver_padding_end_s, t, start_time_s, maneuver_length_s ...
    )
        
        maneuver_should_be_aggregated = ~curr_maneuver_metadata.skip;
        
        % Use sysid index for maneuver if no maneuver start and end time is
        % set in metadata
        maneuver_start_time_not_set = curr_maneuver_metadata.start_s == -1;
        maneuver_end_time_not_set = curr_maneuver_metadata.end_s == -1;
        
        % Use default maneuver start or end if not set, or if maneuver
        % should not be aggregated
        if maneuver_start_time_not_set || ~maneuver_should_be_aggregated
            maneuver_start_s = max(...
                [start_time_s - default_maneuver_padding_start_s...
                 t(1)]); % never start maneuver before first timestamp
        else
            maneuver_start_s = curr_maneuver_metadata.start_s;
        end
        if maneuver_end_time_not_set || ~maneuver_should_be_aggregated
            maneuver_end_s = max(...
                [maneuver_start_s + maneuver_length_s + default_maneuver_padding_start_s + default_maneuver_padding_end_s...
                 t(1)]); % do not end maneuver before first timestamp
        else
            maneuver_end_s = curr_maneuver_metadata.end_s;
        end
end

function [] = read_state_input_data_in_time_interval(start_time_s, end_time_s, csv_log_file_location, dt, save_plot, show_plot, plot_output_location, save_output_data, data_output_location)
    % Read data
    [t, state, input] = read_state_and_input_from_log(csv_log_file_location, dt);

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
        writematrix(t, data_output_location + 't_input.csv');
        writematrix(state, data_output_location + 'state.csv');
        writematrix(input, data_output_location + 'input_input.csv');
    end

end

function [t, state, input, v_N] = read_state_and_input_from_log(csv_log_file_location, dt)
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

    input_offset_s = 0.035;
    % There seems to be a delay between the time the input is commanded and
    % the time it takes before before the system reacts. Either that, or
    % there is some delay in the logging (which is not documented anywhere)
    t_u_fw = actuator_controls_fw.timestamp / 1e6 + input_offset_s;
    % Inputs in [0,1] interval
    fw_aileron_input = actuator_controls_fw.control_0_;
    fw_elevator_input = actuator_controls_fw.control_1_;
    fw_rudder_input = actuator_controls_fw.control_2_;
    fw_throttle_input = actuator_controls_fw.control_3_;
    
    % Convert from [0,1] interval to angles
    [aileron_angle_rad, elevator_angle_rad, rudder_angle_rad] = ...
        calculate_control_surface_angles_rad(fw_aileron_input, fw_elevator_input, fw_rudder_input);
    [fw_throttle_rev_per_s] = calculate_rev_per_s_pusher_motor(fw_throttle_input);
    
    u_fw_raw = [aileron_angle_rad elevator_angle_rad rudder_angle_rad fw_throttle_rev_per_s];
    u_fw = interp1q(t_u_fw, u_fw_raw, t');
    
    
    %%%
    % Create state and input
    %   state structure: [att ang_vel_B vel_B] = [q0 q1 q2 q3 p q r u v w]
    %   input structure: [top_rpm_1 top_rpm_2 top_rpm_3 top_rpm_4 aileron elevator rudder pusher_rpm]
    %                       = [nt1 nt2 nt3 nt4 np delta_a delta_e delta_r]
    
    state = [q0 q1 q2 q3 p q r u v w];
    input = [u_mr u_fw];
    
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
    sysid_indices = sysid_indices(1:sysid_maneuver_num - 1); % cut away zeros
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


function [acc_B, acc_B_unfiltered] = read_accelerations_B(csv_log_file_location, t, state, input)
    % Load data
    sensor_combined = readtable(csv_log_file_location + '_' + "sensor_combined_0" + ".csv");
    sensor_bias = readtable(csv_log_file_location + '_' + "estimator_sensor_bias_0" + ".csv");

    % Read raw sensor data
    t_acc = sensor_combined.timestamp / 1e6;
    acc_meas_raw = [sensor_combined.accelerometer_m_s2_0_ sensor_combined.accelerometer_m_s2_1_ sensor_combined.accelerometer_m_s2_2_];

%     % Check if significant bias
%     t_sensor_bias = sensor_bias.timestamp / 1e6;
%     bias_acc_raw = [sensor_bias.accel_bias_0_ sensor_bias.accel_bias_1_ sensor_bias.accel_bias_2_];
%     bias_acc = interp1q(t_sensor_bias, bias_acc_raw, t');

    % Filter accelerations
    f_cutoff_hz = 25;
    acc_meas_filtered_raw = filter_accelerations(acc_meas_raw, t_acc, f_cutoff_hz);

    % Fuse to common time horizon
    acc_meas_filtered = interp1q(t_acc, acc_meas_filtered_raw, t');
    acc_meas_unfiltered = interp1q(t_acc, acc_meas_raw, t');
    
    % Calculate total acceleration in body frame
    acc_B = calc_acc_body(t, state, input, acc_meas_filtered);
    acc_B_unfiltered = calc_acc_body(t, state, input, acc_meas_unfiltered); 
end


function [acc] = differentiate_vel(dt, vel, type)
    if type == "filter"
        Fs = 1 / dt;

        Nf = 50; 
        Fpass = 15; 
        Fstop = 17.5;

        % From this source:
        % https://se.mathworks.com/help/signal/ug/take-derivatives-of-a-signal.html#:~:text=You%20want%20to%20differentiate%20a,use%20a%20differentiator%20filter%20instead.

        d = designfilt('differentiatorfir','FilterOrder',Nf, ...
            'PassbandFrequency',Fpass,'StopbandFrequency',Fstop, ...
            'SampleRate',Fs);

        dv_dt = filter(d, vel) / dt;

        % Filtered signal is delayed, find this delay
        filter_delay = mean(grpdelay(d));

        % Compensate for delay by discarding samples
        % Shift signal delay forward
        acc = [dv_dt(filter_delay+1:end,:);
                 zeros(filter_delay,3)];
             
    elseif type == "numeric"
        diff_order = 1;
        acc = diff(vel, diff_order) / dt;
        acc = [acc;
               zeros(diff_order, 3)]; % TODO: 3 is state size, for now hardcoded
    end
end

function [acc_B] = calc_acc_body(t, state, input, acc_meas)
    aircraft_properties; % Import g

    % Read data
    N = length(state);
    q_NB = state(:,1:4);
    w_B = state(:,5:7);
    v_B = state(:,8:10);
    u_mr = input(:,1:4);
    u_fw = input(:,5:8);
    
    % Create rotation matrices
    q_BN = quatinv(q_NB);
    R_BN = quat2rotm(q_BN);
    eul = quat2eul(q_NB);
    
    % Extract acceleration without graviational component
    % (accelerometers always measure the gravitational acceleration)

    % Calculate gravitational force in measurement frame
    g_N = [0; 0; g]; % NED frame
    g_M = zeros(N,3); % Measurement frame
    R_MB = diag([1 -1 -1]); % Accelerometer measures in ENU body frame
    R_BM = inv(R_MB); % NOTE not really needed, but kept for clarity.
    for i = 1:N
       g_M(i,:) = R_MB * R_BN(:,:,i) * g_N;
    end
    
    acc_M = acc_meas - g_M;
    
    % Rotate acc to body frame
    acc_B = zeros(N,3);
    for i = 1:N
       acc_B(i,:) = R_BM * acc_M(i,:)';
    end
end

function [acc_filtered] = filter_accelerations(acc, t_acc, f_cutoff)
    % Filter data
    T_c = 1/f_cutoff;
    temp = filloutliers(t_acc(2:end) - t_acc(1:end-1), 'linear');
    dt_acc = mean(temp);
    alpha = dt_acc / (T_c + dt_acc);

    acc_filtered = zeros(size(acc));
    acc_filtered(1,:) = acc(1,:);
    for i = 2:length(acc)
       acc_filtered(i,:) = alpha * acc(i,:) + (1 - alpha) * acc_filtered(i-1,:);
    end

    % Frequency analysis
    if 0
        plot_fft(acc, dt);
        plot_fft(acc_filtered, dt);
    end
end


function [] = plot_ned_frame_states(maneuver_index, t, acc_N, vel_N,...
    show_plot, save_plot, plot_output_location)
    
% Figure details
    fig = figure; 
    if ~show_plot
        fig.Visible = 'off';
    end
    fig.Position = [100 100 1600 1000];
    num_plots = 4; 

    subplot(num_plots,1,1);
    plot(t, vel_N(:,3));
    title("vel z NED frame")
    
    subplot(num_plots,1,2);
    plot(t, acc_N(:,1));
    title("acc x NED frame")
    
    subplot(num_plots,1,3);
    plot(t, acc_N(:,3));
    title("acc z NED frame")

    subplot(num_plots,1,4);
    g = 9.81;
    plot(t, acc_N(:,3) + g);
    title("acc z NED frame compared to 1 g")
    
    figure_title = "NED Accelerations, maneuver: " + maneuver_index;
    sgtitle(figure_title)
    
    if save_plot
        filename = maneuver_index + "_ned_states";
        saveas(fig, plot_output_location + filename, 'epsc')
        %savefig(plot_output_location + filename + '.fig')
    end
end

function [] = plot_accelerations(maneuver_index, t, acc_B, acc_B_unfiltered, ang_acc, ang_acc_unfiltered, show_plot, save_plot, plot_output_location)
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
    legend('a_x', 'a_x (filtered)');
    title("Linear acceleration")
    
    subplot(num_plots,1,2);
    plot(t, acc_B_unfiltered(:,2)); hold on
    plot(t, acc_B(:,2)); hold on
    legend('a_y', 'a_y (filtered)');
    title("Linear acceleration")
    
    subplot(num_plots,1,3);
    plot(t, acc_B_unfiltered(:,3)); hold on
    plot(t, acc_B(:,3)); hold on
    legend('a_z', 'a_z (filtered)');
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