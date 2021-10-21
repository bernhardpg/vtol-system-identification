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
maneuver = fpr_data.validation.yaw_211(1);

% Pitch: validation 1
% Roll: validation 1
% Yaw: validation 1

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
% 
% function plot(rec_data)
%     fig = figure;
%     num_plots_rows = 5;
%     t = tiledlayout(num_plots_rows,1, 'Padding', 'compact', 'TileSpacing', 'compact'); 
% 
%     nexttile
%     plot(time, rec_data(:,1), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on;
%     for i = 1:num_models
%         plot(time, model_data.(model_names(i))(:,1),'LineWidth',line_width, 'Color', lon_colors(i,:)); hold on
%     end
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     grid on
%     grid minor
%     set(gca,'FontSize', font_size_small)
%     ylabel("$u [m/s]$", 'interpreter', 'latex', 'FontSize', font_size)
%     ylim([10 25])
%     xlim([0 time(end)]);
% 
%     nexttile
%     plot(time, rec_data(:,2), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on
%     for i = 1:num_models
%         plot(time, model_data.(model_names(i))(:,2),'LineWidth',line_width,'Color', lon_colors(i,:)); hold on
%     end
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     grid on
%     grid minor
%     set(gca,'FontSize', font_size_small)
%     ylabel("$w [m/s]$", 'interpreter', 'latex', 'FontSize', font_size)
%     ylim([-6 6])
%     xlim([0 time(end)]);
% 
%     nexttile
%     plot(time, rad2deg(rec_data(:,3)), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on
%     for i = 1:num_models
%         plot(time, rad2deg(model_data.(model_names(i))(:,3)),'LineWidth',line_width,'Color', lon_colors(i,:)); hold on
%     end
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     grid on
%     grid minor
%     set(gca,'FontSize', font_size_small)
%     ylim([-100 100]);
%     xlim([0 time(end)]);
%     ylabel("$q [^\circ/s]$", 'interpreter', 'latex', 'FontSize', font_size)
% 
%     nexttile
%     plot(time, rad2deg(rec_data(:,4)), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on
%     for i = 1:num_models
%         plot(time, rad2deg(model_data.(model_names(i))(:,4)),'LineWidth',line_width,'Color', lon_colors(i,:)); hold on
%     end
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     grid on
%     grid minor
%     set(gca,'FontSize', font_size_small)
%     ylabel("$\theta [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
%     ylim([-30 30])
%     xlim([0 time(end)]);
%     lgd = legend(["Recorded Data" model_names_to_display], 'location', 'southeast', 'FontSize', font_size_small);
% 
% 
%     nexttile
%     plot(time, rad2deg(input(:,1)), 'black', 'LineWidth', line_width); hold on
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     grid on
%     grid minor
%     set(gca,'FontSize', font_size_small)
%     ylabel("$\delta_e [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
%     ylim([-32 32])
%     xlim([0 time(end)]);
%     lgd = legend("Input", 'location', 'best', 'FontSize', font_size_small);
% 
%     title(t, plot_title, 'FontSize', font_size_large, 'interpreter', 'latex')
%     xlabel(t, "Time $[s]$", 'interpreter', 'latex', 'FontSize', font_size)
%               
%     
%     fig = figure;
%     num_plots_rows = 6;
%     t = tiledlayout(num_plots_rows,1, 'Padding', 'compact', 'TileSpacing', 'compact'); 
% 
%     nexttile
%     plot(time, rec_data(:,1), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on;
%     for i = 1:num_models
%         plot(time, model_data.(model_names(i))(:,1), 'LineWidth',line_width,'Color', lat_colors(i,:)); hold on
%     end
%     grid on
%     grid minor
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     set(gca,'FontSize', font_size_small)
%     ylabel("$v [m/s]$", 'interpreter', 'latex', 'FontSize', font_size)
%     ylim([-13 13])
%     xlim([0 time(end)]);
% 
%     nexttile
%     plot(time, rad2deg(rec_data(:,2)), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on
%     for i = 1:num_models
%         plot(time, rad2deg(model_data.(model_names(i))(:,2)), 'LineWidth',line_width, 'Color', lat_colors(i,:)); hold on
%     end
%     grid on
%     grid minor
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     set(gca,'FontSize', font_size_small)
%     ylabel("$p [^\circ/s]$", 'interpreter', 'latex', 'FontSize', font_size)
%     ylim([-160 160]);
%     xlim([0 time(end)]);
% 
%     nexttile
%     plot(time, rad2deg(rec_data(:,3)), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on
%     for i = 1:num_models
%         plot(time, rad2deg(model_data.(model_names(i))(:,3)), 'LineWidth',line_width, 'Color', lat_colors(i,:)); hold on
%     end
%     grid on
%     grid minor
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     set(gca,'FontSize', font_size_small)
%     ylim([-120 120]);
%     xlim([0 time(end)]);
%     ylabel("$r [^\circ/s]$", 'interpreter', 'latex', 'FontSize', font_size)
% 
%     nexttile
%     plot(time, rad2deg(rec_data(:,4)), plot_style_recorded_data, 'LineWidth',line_width, 'Color', target_color); hold on
%     for i = 1:num_models
%         plot(time, rad2deg(model_data.(model_names(i))(:,4)),  'LineWidth',line_width, 'Color', lat_colors(i,:)); hold on
%     end
%     grid on
%     grid minor
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     set(gca,'FontSize', font_size_small)
%     ylabel("$\phi [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
%     ylim([-90 90])
%     xlim([0 time(end)]);
%     lgd = legend(["Recorded Data" model_names_to_display], 'location', 'southeast', 'FontSize', font_size_small);
% 
% 
%     nexttile
%     plot(time, rad2deg(input(:,1)), 'black', 'LineWidth', line_width); hold on
%     grid on
%     grid minor
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     set(gca,'FontSize', font_size_small)
%     ylabel("$\delta_a [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
%     ylim([-32 32])
%     xlim([0 time(end)]);
% 
%     nexttile
%     plot(time, rad2deg(input(:,2)), 'black', 'LineWidth', line_width); hold on
%     grid on
%     grid minor
%     plot_maneuver_lines(maneuver_start_index, num_maneuvers_to_plot, time)
%     set(gca,'FontSize', font_size_small)
%     ylabel("$\delta_r [^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
%     ylim([-32 32])
%     xlim([0 time(end)]);
%     lgd = legend("Input", 'location', 'best', 'FontSize', font_size_small);
% 
%     title(t, plot_title, 'FontSize', font_size_large, 'interpreter', 'latex')
%     xlabel(t, "Time $[s]$", 'interpreter', 'latex', 'FontSize', font_size)
% end