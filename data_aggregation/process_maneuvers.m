clc; clear all; close all;

% Load metadata
metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Maneuver settings
maneuver_type = "pitch_211";

% Plot settings
plot_location = "data/maneuver_plots/" + maneuver_type + "/";
save_maneuver_plot = false;
show_maneuver_plot = false;

% Set data params
dt_desired = 1 / 50;

% Read data recorded from logs
[t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw] ...
    = read_experiment_data(metadata, maneuver_type);

% Calculate states and their derivatives using splines
% TODO: Save maneuers for validation
[t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a, delta_e, delta_r, n_p, maneuver_start_indices]...
    = collect_data_from_all_maneuvers(dt_desired, t_state_all_maneuvers, q_NB_all_maneuvers, v_NED_all_maneuvers, t_u_fw_all_maneuvers, u_fw_all_maneuvers, maneuver_start_indices_state, maneuver_start_indices_u_fw,...
        save_maneuver_plot, show_maneuver_plot, plot_location);

% TODO: I need to find actual PWM to RPM scale.
%T = calc_propeller_force(n_p);
[c_X, c_Y, c_Z] = calc_force_coeffs(u, v, w, a_x, a_y, a_z);
[c_l, c_m, c_n] = calc_moment_coeffs(p, q, r, u, v, w, p_dot, q_dot, r_dot);

% Explanatory variables for equation-error
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);

% Create a time vector for plotting of all maneuvers
t_plot = 0:dt_desired:length(t)*dt_desired - dt_desired;

%%
c_X_0 = 0.1276;
c_X_u = -0.3511;
c_X_w = 0.1757;
c_X_w_sq = 2.7162;
c_X_q = -3.2355;
c_X_n_p = 0.0025;
c_Z_0 = -0.5322;
c_Z_w = -5.1945;
c_Z_w_sq = 5.7071;
c_Z_delta_e = -0.3440;
c_m_0 = 0.0266;
c_m_w = -1.0317;
c_m_q = 1.0616;
c_m_delta_e = -0.3329;

aircraft_properties;
params = [rho, mass_kg, g, wingspan_m, mean_aerodynamic_chord_m, planform_sqm, V_nom,...
     gam_1, gam_2, gam_3, gam_4, gam_5, gam_6, gam_7, gam_8, J_yy,...
     c_X_0, c_X_u, c_X_w, c_X_w_sq, c_X_q, c_X_n_p,...
     c_Z_0, c_Z_w, c_Z_w_sq, c_Z_delta_e,...
     c_m_0, c_m_w, c_m_q, c_m_delta_e,...
     ]';

for maneuver_i = 2:length(maneuver_start_indices)
    [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_m, delta_e_m, delta_r_m, n_p_m]...
     = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a, delta_e, delta_r, n_p);
    
    % Get inputs
    input_seq = [delta_a_m, delta_e_m, delta_r_m, n_p_m];
    
    % Take lat states as they are
    lat_state_seq = [phi_m, psi_m, p_m, r_m, v_m];
    
    tspan = [t_m(1) t_m(end)];
    
    y0 = [theta_m(1) q_m(1) u_m(1) w_m(1)];
    [t_pred, y_pred] = ode23s(@(t,y) aircraft_dynamics_lon(t, y, t_m, input_seq, lat_state_seq, params), tspan, y0);
        
    plot_maneuver("maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_e_m, delta_r_m, n_p_m,...
        t_pred, y_pred,...
        false, true, "");
end


%%
%%%
% Find most relevant terms for c_Z:
%%%

clc;

N = length(c_Z);

% 1st
X = [ones(N, 1)]; % Regressor
z = c_Z; % Output measurements
indep_vars_str = "1";
vars_to_test_str = "u_hat w_hat q_hat delta_e n_p";
vars_to_test = [u_hat w_hat q_hat delta_e n_p];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

% Add w as independent variable
X = [ones(N, 1) w_hat]; % Regressor
indep_vars_str = "1 w_hat";
vars_to_test_str = "u_hat q_hat delta_e n_p";
vars_to_test = [u_hat q_hat delta_e n_p];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

% Add q_hat as independent variable
X = [ones(N, 1) w_hat delta_e]; % Regressor
indep_vars_str = "1 w_hat delta_e";
vars_to_test_str = "u_hat q_hat n_p";
vars_to_test = [u_hat q_hat n_p];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

disp("Try nonlinear terms")
vars_to_test_str = "w_hat.^2 q_hat.^2 delta_e.^2";
vars_to_test = [w_hat.^2 q_hat.^2 delta_e.^2];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat w_hat.^2 delta_e]; % Regressor
indep_vars_str = "1 w_hat w_hat.^2 delta_e";
[c_Z_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str);
th_hat = num2cell(th_hat);
[c_Z_0, c_Z_w, c_Z_w_sq, c_Z_delta_e] = th_hat{:}

fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_Z, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_Z_hat); hold on
legend("c_Z", "c_Z (predicted)")

%%

%%%
% Find most relevant terms for c_m:
%%%

clc;
N = length(c_m);

X = [ones(N, 1)]; % Regressor
z = c_m; % Output measurements
indep_vars_str = "1";
vars_to_test_str = "u_hat w_hat q_hat delta_e n_p";
vars_to_test = [u_hat w_hat q_hat delta_e n_p];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat]; % Regressor
indep_vars_str = "1 w_hat";
vars_to_test_str = "u_hat q_hat delta_e n_p";
vars_to_test = [u_hat q_hat delta_e n_p];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat delta_e]; % Regressor
indep_vars_str = "1 w_hat delta_e";
vars_to_test_str = "u_hat q_hat n_p";
vars_to_test = [u_hat q_hat n_p];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

vars_to_test_str = "w_hat.^2 delta_e.^2";
vars_to_test = [w_hat.^2 delta_e.^2];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat q_hat delta_e]; % Regressor
indep_vars_str = "1 w_hat delta_e q_hat";
vars_to_test_str = "u_hat q_hat n_p";
vars_to_test = [u_hat q_hat n_p];
[c_m_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);
th_hat = num2cell(th_hat);
[c_m_0, c_m_w, c_m_q, c_m_delta_e] = th_hat{:}

fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_m, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_m_hat); hold on
legend("c_m", "c_m (predicted)")


%%

%%%
% Find most relevant terms for c_X:
%%%

clc;
N = length(c_X);

X = [ones(N, 1)]; % Regressor
z = c_X; % Output measurements
indep_vars_str = "1";
vars_to_test_str = "u_hat w_hat q_hat delta_e n_p";
vars_to_test = [u_hat w_hat q_hat delta_e n_p];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat]; % Regressor
indep_vars_str = "1 u_hat";
vars_to_test_str = " w_hat q_hat delta_e n_p";
vars_to_test = [w_hat q_hat delta_e n_p];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat n_p]; % Regressor
indep_vars_str = "1 u_hat n_p";
vars_to_test_str = " w_hat q_hat delta_e";
vars_to_test = [w_hat q_hat delta_e];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat w_hat n_p]; % Regressor
indep_vars_str = "1 u_hat w_hat n_p";
vars_to_test_str = "w_hat.^2";
vars_to_test = [w_hat.^2];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat w_hat w_hat.^2 n_p]; % Regressor
indep_vars_str = "1 u_hat w_hat w_hat.^2 n_p";
stepwise_regression_round(X, z, indep_vars_str);
disp(" ")

X = [ones(N, 1) u_hat w_hat w_hat.^2 q_hat n_p]; % Regressor
indep_vars_str = "1 u_hat w_hat w_hat.^2 n_p q_hat";
[c_X_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str);
th_hat = num2cell(th_hat);
[c_X_0, c_X_u, c_X_w, c_X_w_sq c_X_q c_X_n_p] = th_hat{:}

fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_X, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_X_hat); hold on
legend("c_X", "c_X (predicted)")



%% LATERAL MODES
% NOTE: FOR THIS I FIRST NEED TO LOAD LAT DATA!

%%%
% Find most relevant terms for c_Y:
%%%

clc;
N = length(c_Y);

X = [ones(N, 1)]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1";
vars_to_test_str = "v_hat p_hat r_hat delta_a delta_r";
vars_to_test = [v_hat p_hat r_hat delta_a delta_r];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat";
vars_to_test_str = "p_hat r_hat delta_a delta_r";
vars_to_test = [p_hat r_hat delta_a delta_r];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat delta_r]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat delta_r";
vars_to_test_str = "p_hat r_hat delta_a";
vars_to_test = [p_hat r_hat delta_a];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat delta_r delta_a]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat delta_r delta_a";
vars_to_test_str = "p_hat r_hat";
vars_to_test = [p_hat r_hat];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat delta_r delta_a]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat p_hat delta_r delta_a";
vars_to_test_str = "r_hat";
vars_to_test = [r_hat];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat r_hat delta_r delta_a]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat p_hat r_hat delta_r delta_a";
[c_Y_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str);

fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_Y, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_Y_hat); hold on
legend("c_X", "c_X (predicted)")

%%

%%%
% Find most relevant terms for c_l:
%%%

clc;
N = length(c_l);

X = [ones(N, 1)]; % Regressor
z = c_l; % Output measurements
indep_vars_str = "1";
vars_to_test_str = "v_hat p_hat r_hat delta_a delta_r";
vars_to_test = [v_hat p_hat r_hat delta_a delta_r];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) delta_a]; % Regressor
z = c_l; % Output measurements
indep_vars_str = "1 delta_a";
vars_to_test_str = "v_hat p_hat r_hat delta_r";
vars_to_test = [v_hat p_hat r_hat delta_r];
[c_l_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) r_hat delta_a]; % Regressor
z = c_l; % Output measurements
indep_vars_str = "1 r_hat delta_a";
vars_to_test_str = "v_hat p_hat delta_r";
vars_to_test = [v_hat p_hat delta_r];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) p_hat r_hat delta_a]; % Regressor
z = c_l; % Output measurements
indep_vars_str = "1 p_hat r_hat delta_a";
vars_to_test_str = "v_hat delta_r";
vars_to_test = [v_hat delta_r];
stepwise_regression_round(X, z, indep_vars_str);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat r_hat delta_a delta_r]; % Regressor
z = c_l; % Output measurements
indep_vars_str = "1 v_hat p_hat r_hat delta_a delta_r";
stepwise_regression_round(X, z, indep_vars_str);

fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_l, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_l_hat); hold on
legend("c_X", "c_X (predicted)")

%%
function [y_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_variables_str)
    [y_hat, F0, R_sq, cov_th, th_hat] = regression_analysis(X, z);

    disp("Independent variables: [" + indep_variables_str + "]")
    fprintf("F0: ")
    fprintf([repmat('%4.2f ',1,length(F0)) '\n'], F0);
    disp("R_sq: " + R_sq);
end

function [] = explore_next_var(z, variables_to_test, variables_to_test_str)
    disp("Testing new terms:")
    r = calculate_partial_correlation(variables_to_test, z);
    disp("r: " + variables_to_test_str);
    fprintf([repmat('%5.3f ',1,length(r)) '\n'], r);
    disp(" ")
end

function [y_hat, F0, R_sq, cov_th, th_hat] = regression_analysis(X, z)
    [N, n_p] = size(X);
    D = (X' * X)^(-1);
    d = diag(D);
    th_hat = D * X' * z;
    
    y_hat = X * th_hat; % Estimated output
    
    v = z - y_hat; % Residuals
    sig_sq_hat = v' * v / (N - n_p); % Estimated noise variance
    cov_th = sig_sq_hat * d; % Estimates parameter variance
    
    % Calculate partial F metrix F0
    F0 = th_hat .^ 2 ./ cov_th;
    
    % Calculate R^2
    z_bar = mean(z);
    R_sq = (y_hat' * z - N * z_bar^2) / (z' * z - N * z_bar^2);
end

function [r] = calculate_partial_correlation(X, z)
    X_bar = mean(X);
    N = length(z);
    z_bar = mean(z);
    
    cov_Xz = (X - X_bar)' * (z - z_bar) / (N - 1);
    var_X = diag((X - X_bar)' * (X - X_bar)) / (N - 1);
    var_z = diag((z - z_bar)' * (z - z_bar)) / (N - 1);
    r = cov_Xz ./ sqrt(var_X * var_z);
end

function [p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w)
    aircraft_properties; % Get V_nom, wingspan and MAC
    
    u_hat = u / V_nom;
    v_hat = v / V_nom;
    w_hat = w / V_nom;
    p_hat = p * (wingspan_m / (2 * V_nom));
    q_hat = q * (mean_aerodynamic_chord_m / (2 * V_nom));
    r_hat = r * (wingspan_m / (2 * V_nom));
end

function [dyn_pressure] = calc_dyn_pressure(u, v, w)
    aircraft_properties; % to get rho

    V = sqrt(u .^ 2 + v .^ 2 + w .^ 2);
    dyn_pressure = 0.5 * rho * V .^ 2;
end

function [T] = calc_propeller_force(n_p)
    aircraft_properties; % to get rho and diam_pusher
    T = rho * prop_diam_pusher ^ 4 * c_T_0_pusher * n_p .^ 2;
end

function [c_X, c_Y, c_Z] = calc_force_coeffs(u, v, w, a_x, a_y, a_z, T)
    dyn_pressure = calc_dyn_pressure(u, v, w);
    aircraft_properties; % get mass and planform
    % TODO: Add thrust here
    T = 0;
    c_X = (mass_kg * a_x - T) ./ (dyn_pressure * planform_sqm);
    c_Y = (mass_kg * a_y) ./ (dyn_pressure * planform_sqm);
    c_Z = (mass_kg * a_z) ./ (dyn_pressure * planform_sqm);
end

function [c_l, c_m, c_n] = calc_moment_coeffs(p, q, r, u, v, w, p_dot, q_dot, r_dot)
    dyn_pressure = calc_dyn_pressure(u, v, w);
    aircraft_properties; % get inertias, wingspan, MAC and planform

    c_l = (J_xx * p_dot - J_xz * (r_dot + p .* q) + q .* r * (J_zz - J_yy)) ./ (dyn_pressure * wingspan_m * planform_sqm);
    c_m = (J_yy * q_dot - r .* p * (J_xx - J_zz) + J_xz * (p .^ 2 - r .^ 2)) ./ (dyn_pressure * mean_aerodynamic_chord_m * planform_sqm);
    c_n = (J_zz * r_dot - J_xz * (p_dot - q .* r) + p .* q * (J_yy - J_xx)) ./ (dyn_pressure * wingspan_m * planform_sqm);
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


