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
fpr_data = fpr_data_lat;
maneuver = fpr_data.validation.roll_211(1);

% Prepare recorded data %
t_seq = maneuver.Time();
input_seq = maneuver.get_input_sp_sequence();
y_0 = [maneuver.get_state_initial() maneuver.get_input_initial()];
y_recorded = maneuver.get_state_sequence();
tspan = [t_seq(1) t_seq(end)];

% Simulate nonlinear model with real input setpoints %
[t_sim, y_sim] = ode45(@(t,y) nonlinear_aircraft_model(t, y, @(t) calc_input_at_t(t, t_seq, input_seq)), tspan, y_0);
plot_maneuver(t_sim, y_sim, t_seq, y_recorded, input_seq);



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