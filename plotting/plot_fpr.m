clc; clear all; close all;

% The script will automoatically load all maneuvers in "data_raw/experiments/*"

% This script takes in raw maneuver data from csv files and calculates all
% the required signals and their derivatives. This includes states, their
% derivatives, coefficients, and inputs.
 
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

%disp("Setting random seed to default to guarantee reproducability");
rng default % Always shuffle the maneuvers in the same way for reproducability

% Load metadata
metadata_filename = "data/flight_data/metadata.json";
metadata = read_metadata(metadata_filename);

% Data settings
time_resolution = 0.02; % 50 Hz
knot_points_for_spline_derivation_dt = [0.1 0.4];

% Plot settings
save_raw_plots = false;
save_kinematic_consistency_plots = false;
save_lateral_signal_plots = false;
save_longitudinal_signal_plots = false;

model_type = "lateral-directional";

% These maneuvers are skipped either because they contain dropouts or
% because they are part of another maneuver sequence.
% The entire data loading scheme would benefit from being reconsidered, but
% there is not time for this.
maneuvers_to_skip = {};
maneuvers_to_skip.("roll_211") = [6 11 12 14 20];
maneuvers_to_skip.("yaw_211") = [1:11 18 22:27 29 30 35:39 41];
maneuvers_to_skip.("pitch_211") = [2 3 7 8 9 11 14 17 18 19 20 21 24 25 32 35 40];

% Save raw maneuver data
fpr_data = {};
maneuver_type = "roll_211";
maneuver_i = 1;

maneuvers_to_skip_for_curr_type = maneuvers_to_skip.(maneuver_type);
disp("Processing " + maneuver_type + " maneuvers.");

% Read raw data recorded from logs
[t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, p_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw] ...
    = read_data_from_experiments(metadata, maneuver_type);

selected_maneuvers = create_maneuver_objects_from_raw_data(maneuver_type, t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, p_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw);

% FPR for different knot values
maneuver_for_different_knot_values = [];
for knot_value = knot_points_for_spline_derivation_dt
    maneuver_for_different_knot_values = [maneuver_for_different_knot_values selected_maneuvers(maneuver_i).calc_fpr_from_rawdata(time_resolution, knot_value)];
end


figure
plot(maneuver_for_different_knot_values(1).RawData.Time, rad2deg(maneuver_for_different_knot_values(1).RawData.EulTheta), '--',...
    "LineWidth", 2);
hold on;
for sim_i = 1:length(knot_points_for_spline_derivation_dt)
    [t_sim, phi_sim, theta_sim, psi_sim, u_sim, v_sim, w_sim] = maneuver_for_different_knot_values(sim_i).simulate_kinematic_consistency();
    plot(t_sim, rad2deg(theta_sim), "LineWidth", 1.2); hold on
end
plot_settings;
legend(["Measured data" "Reconstructed data ($dt = 0.1$ s)" "Reconstructed data ($dt = 0.4$ s)"], 'interpreter','latex','FontSize',font_size);
title("Effect of varying the spline intervals", 'interpreter','latex','FontSize',font_size_large);
xlabel("Time [s]", 'interpreter','latex','FontSize',font_size);
ylabel("$\theta [^\circ]$", 'interpreter','latex','FontSize',font_size);

