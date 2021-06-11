clc; clear all; close all;

% Load parameters from all runs
results_path = "model_identification/output_error/results/";
xs_lon = readmatrix(results_path + "1_lon_params_all_lon_maneuvers.csv");
xs_lat = readmatrix(results_path + "2_lat_params_all_lat_maneuvers.csv");

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
x = [-0.0947565530079021,0.139398703947964,2.40029553727352,-11.7708407170177,639.316946524285,-0.0396226409839468,-0.477503070278427,-4.9928307378831,3.94250638810824,-0.610510762194976,0.0137134335471995,-1.35850824924436,-26.8341777193958,-0.946623650779093,-0.707394391883844,0.0229901460175752,0.696444364574421,-0.625911653228454,-0.200249675223384,0.420355763175705,-0.00536477126410978,-0.371710607005147,0.156507176398939,-0.0450429658949077,0.150446307969574,9.52922483673506e-05,-0.0932418846766781,-0.120284959296711,0.117734628396492,-0.0492868472124431];

% Test model
data_type = "train";
maneuver_types = ["roll_211"];
load_data; % Loads all states and inputs
maneuvers_to_test = [1 2];
save_plot = false;
show_plot = true;
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
