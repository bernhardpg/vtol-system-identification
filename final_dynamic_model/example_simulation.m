clc; clear all; close all;

%%%%%%%
% This script provides an example simulation with the model, where a real
% maneuver is loaded. The model is simulated with the recorded input
% setpoints, and the result is plotted together with the simulated
% response.
%%%%%

% Select some maneuver to test %
load("data/flight_data/selected_data/fpr_data_lon.mat");
load("data/flight_data/selected_data/fpr_data_lat.mat");

% Selected validation maneuvers for final plot
maneuvers = {fpr_data_lat.validation.roll_211(1)...
             fpr_data_lon.validation.pitch_211(1)...
             fpr_data_lat.validation.yaw_211(1)};
         
% Find maneuver separate lines
maneuver_lengths = zeros(3,1);
for i=1:3
    maneuver_lengths(i) = length(maneuvers{i}.Time);
end
vlines = cumsum(maneuver_lengths(1:2));
         
dt = maneuvers{1}.Time(2) - maneuvers{1}.Time(1);
         
y_sim_collected = [];
y_rec_collected = [];
input_collected = [];
for maneuver_i = 1:numel(maneuvers)
    maneuver = maneuvers{maneuver_i};

    % Prepare recorded data %
    t_seq = maneuver.Time();
    input_seq = maneuver.get_input_sp_sequence();
    y_0 = [maneuver.get_state_initial() maneuver.get_input_initial()];
    y_recorded = maneuver.get_state_sequence();
    tspan = [t_seq(1) t_seq(end)];

    % Simulate nonlinear model with real input setpoints %
    [t_sim, y_sim] = ode45(@(t,y) nonlinear_aircraft_model(t, y, @(t) calc_input_at_t(t, t_seq, input_seq)), tspan, y_0);
    y_sim = interp1(t_sim, y_sim, t_seq);
    
    % Store values
    y_sim_collected = [y_sim_collected;
                       y_sim];
    y_rec_collected = [y_rec_collected
                       y_recorded];
    input_collected = [input_collected;
                       input_seq];
   
end

t_plot = 0:dt:(length(y_sim_collected))*dt-dt;
plot_maneuver(t_plot, y_sim_collected, y_rec_collected, input_collected, vlines);

function input_at_t = calc_input_at_t(t, t_seq, input_seq)
    % Roll index forward until we get to approx where we should get
    % inputs from. This basically implements zeroth-order hold for
    % the input
    curr_index_data_seq = 1;
    while t_seq(curr_index_data_seq) < t
       curr_index_data_seq = curr_index_data_seq + 1;
    end
    
    % Get input at t
    input_at_t = input_seq(curr_index_data_seq,:);
end