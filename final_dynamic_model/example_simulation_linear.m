clc; clear all; close all;

% Import parameters
aerodynamic_coeffs;
static_parameters;
trim_values;

%%%%%%%
% This script provides an example simulation with the linear model, where a real
% maneuver is loaded. The model is simulated with the recorded input
% setpoints, and the result is plotted together with the simulated
% response.
%%%%%

% Select some maneuver to test %
load("data/flight_data/selected_data/fpr_data_lon.mat");
load("data/flight_data/selected_data/fpr_data_lat.mat");
fpr_data = fpr_data_lat;
maneuver = fpr_data.validation.roll_211(3);

% Prepare recorded data %
t_seq = maneuver.Time();
tspan = [t_seq(1) t_seq(end)];
input_seq_sp = maneuver.get_input_sp_sequence();

% Simulate control surfaces separately
[t_sim, delta_sim] = ode45(@(t,y) ...
    control_surfaces_model(t, y, ...
    @(t) calc_input_at_t(t, t_seq, input_seq_sp(:,1:3))),...
    tspan, input_seq_sp(1,1:3));
delta_sim = interp1(t_sim, delta_sim, t_seq);
input_seq = [delta_sim input_seq_sp(:,4)];

% Prepare aircraft simulation
y_0 = [maneuver.get_state_initial() maneuver.get_input_initial()];
y_recorded = maneuver.get_state_sequence();

% Prepare perturbation quantities
u_lon_trim = [delta_e_trim delta_t_trim];
u_lon = input_seq(:,[2 4]) - u_lon_trim;
y_lon_trim = [u_trim w_trim 0 theta_trim];
y_0_lon = y_0(:,[1 3 5 8]) - y_lon_trim;

u_lat_trim = [delta_a_trim 0];
u_lat = input_seq(:,[1 3]) - u_lat_trim;
y_0_lat = y_0(:,[2 4 6 7]);

% Simulate linear model
linear_model;
y_lon_sim = lsim(ss_lon, u_lon, t_seq, y_0_lon) + y_lon_trim;
y_lat_sim = lsim(ss_lat, u_lat, t_seq, y_0_lat);

y_sim = [y_lon_sim(:,1) y_lat_sim(:,1) y_lon_sim(:,2)...
         y_lat_sim(:,2) y_lon_sim(:,3) y_lat_sim(:,3)...
         y_lat_sim(:,4) y_lon_sim(:,4)...
         input_seq(:,1) input_seq(:,2) input_seq(:,3)];

plot_maneuver(t_seq, y_sim, t_seq, y_recorded, input_seq_sp);

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


