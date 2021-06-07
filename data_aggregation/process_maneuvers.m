clc; clear all; close all;

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Set data params
dt_desired = 1 / 50;

% Read data recorded from logs
maneuver_type = "pitch_211";
[t_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, maneuver_start_indices] = read_experiment_data(metadata, maneuver_type);
num_maneuvers = length(maneuver_start_indices);

% Iterate through all maneuvers and calculate data
for maneuver_i = 1:num_maneuvers
    % Get correct maneuver start and end index
    maneuver_start_index = maneuver_start_indices(maneuver_i);
    if maneuver_i == num_maneuvers
        maneuver_end_index = length(t_all_maneuvers);
    else
        maneuver_end_index = maneuver_start_indices(maneuver_i + 1) - 1;
    end            

    % Extract recorded data during maneuver
    t_recorded = t_all_maneuvers(maneuver_start_index:maneuver_end_index,:);
    t_recorded = t_recorded - t_recorded(1); % Make t go from 0 to end
    q_NB = q_NB_all_maneuvers(maneuver_start_index:maneuver_end_index,:);
    v_NED = v_NED_all_maneuvers(maneuver_start_index:maneuver_end_index,:);
    
    eul_recorded = quat2eul(q_NB);
    phi_recorded = eul_recorded(:,3);
    theta_recorded = eul_recorded(:,2);
    psi_recorded = eul_recorded(:,1);
    v_N_recorded = v_NED(:,1);
    v_E_recorded = v_NED(:,2);
    v_D_recorded = v_NED(:,3);
    
    % Calculate all relevant states and their derivatives during the
    % maneuver
    [phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z] ...
        = calc_states_and_derivs(dt_desired, t_recorded, phi_recorded, theta_recorded, psi_recorded, v_N_recorded, v_E_recorded, v_D_recorded);
    
    % Check kinematic consistency
    %check_kinematic_consistency(t, phi, theta, psi, p, q, r, t_recorded, phi_recorded, theta_recorded, psi_recorded);
    
    % Plot velocities
    %plot_velocity(t, u, v, w, t_recorded, u_recorded, v_recorded, w_recorded);
end

function [] = plot_velocity(t, u, v, w, t_recorded, u_recorded, v_recorded, w_recorded)
    figure
    subplot(3,1,1)
    plot(t, u, t_recorded, u_recorded, '--r')
    legend("u", "u (recorded)")
    
    subplot(3,1,2)
    plot(t, v, t_recorded, v_recorded, '--r')
    legend("v", "v (recorded)")
    
    subplot(3,1,3)
    plot(t, w, t_recorded, w_recorded, '--r')
    legend("w", "w (recorded)")
end
