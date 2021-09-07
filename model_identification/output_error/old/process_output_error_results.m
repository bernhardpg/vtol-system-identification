clc; clear all; close all;

% Load parameters from all runs
%results_path = "model_identification/output_error/results/";
%xs = readmatrix(results_path + "4_full_model_params.csv");

xs = readmatrix("lon_params_free.txt");

n_bins = 40;

param_names = [
    "c_X_0", "c_X_w", "c_X_w^2", "c_X_q", "c_X_q^2", "c_X_delta_e",...
    "c_Z_0", "c_Z_w", "c_Z_w^2", "c_Z_delta_e",...
    "c_m_0", "c_m_w", "c_m_q", "c_m_delta_e", "c_m_delta_e^2",...
    "c_Y_0", "c_Y_p", "c_Y_v", "c_Y_delta_a", "c_Y_delta_r",...
    "c_l_0", "c_l_p", "c_l_r", "c_l_v", "c_l_delta_a",...
    "c_n_0", "c_n_p", "c_n_r", "c_n_v", "c_n_delta_r",...
    ];
[~, n_params] = size(xs);
figure
for i = 1:n_params
    subplot(5,round(n_params/5),i)
    histogram(xs(:,i), n_bins);
    %title(param_names(i));
end
sgtitle("All parameters")
xs = rmoutliers(xs);
x = median(xs);



%%
%results_path = "model_identification/output_error/results/";
%xs_lon = readmatrix(results_path + "lon_multiple_runs/run_1.csv");
xs_lon = readmatrix("lon_params_free.txt");
xs_lon = rmoutliers(xs_lon);
%x_lon = median(xs_lon);
% lon_param_mads = mad(xs_lon);
% writematrix(x_lon, results_path + "lon_multiple_runs/run_1_medians.csv");
% writematrix(lon_param_mads, results_path + "lon_multiple_runs/run_1_mads.csv");

n_bins = 10;

param_names_lon = [
    "c_X_0", "c_X_w", "c_X_w^2", "c_X_q", "c_X_q^2", "c_X_delta_e",...
    "c_Z_0", "c_Z_w", "c_Z_w^2", "c_Z_delta_e",...
    "c_m_0", "c_m_w", "c_m_q", "c_m_delta_e", "c_m_delta_e^2",...
    ];
[~, n_params_lon] = size(xs_lon);
figure
for i = 1:n_params_lon
    subplot(3,round(n_params_lon/3),i)
    histogram(xs_lon(:,i), n_bins);
    title(param_names_lon(i));
end
sgtitle("Longitudinal parameters")

%%
xs_lat = readmatrix(results_path + "2_lat_params_all_lat_maneuvers.csv");
param_names_lat = [
     "c_Y_0", "c_Y_p", "c_Y_v", "c_Y_delta_a", "c_Y_delta_r",...
     "c_l_0", "c_l_p", "c_l_r", "c_l_v", "c_l_delta_a",...
     "c_n_0", "c_n_p", "c_n_r", "c_n_v", "c_n_delta_r",...
     ];
[~, n_params_lat] = size(xs_lat);
figure
for i = 1:n_params_lat
    subplot(1,n_params_lat,i)
    histogram(xs_lat(:,i), n_bins);
    title(param_names_lat(i));
end
sgtitle("Lateral parameters")



% Calculate median parameters and use this as param
% Longitudinal model fit to pure lon model where lat states are taken as they are
xs_lon = rmoutliers(xs_lon);
x_lon = median(xs_lon);
lon_param_mads = mad(xs_lon);

% Lateral model fit with lon params only fit to lateral data
xs_lat = rmoutliers(xs_lat);
x_lat = median(xs_lat);
lat_param_mads = mad(xs_lat);

x = [x_lon x_lat];

writematrix(x, results_path + "3_chosen_model_params.csv");

%%
% Test model
data_type = "val";
maneuver_types = ["roll_211" "pitch_211" "yaw_211"];
load_data; % Loads all states and inputs
maneuvers_to_test = [1 2 3 11 12 13 21 22 23];
save_plot = true;
show_plot = false;
evaluate_full_model(maneuvers_to_test, maneuver_types, x, save_plot, show_plot,...
    maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_e_sp, delta_r_sp, delta_a, delta_e, delta_r, n_p);

%%

figure
num_params = length(x0);
for i = 1:num_params
    subplot(3,5,i)
    errorbar(0,x_lat(i),param_mads(i),'-s','MarkerSize',10,...
        'MarkerEdgeColor','red','MarkerFaceColor','red')
    title(param_names(i));
end
