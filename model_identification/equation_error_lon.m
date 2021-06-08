clc; clear all; close all;

% Maneuver settings
maneuver_type = "pitch_211";
data_path = "data/aggregated_data/" + maneuver_type + "/";

% Load data
[t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a, delta_e, delta_r, n_p, c_X, c_Y, c_Z, c_l, c_m, c_n, maneuver_start_indices]...
    = load_variables_from_file(data_path);
dt = t(2) - t(1);

% Explanatory variables for equation-error
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);

% Create a time vector for plotting of all maneuvers
t_plot = 0:dt:length(t)*dt - dt;

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

function [T] = calc_propeller_force(n_p)
    aircraft_properties; % to get rho and diam_pusher
    T = rho * prop_diam_pusher ^ 4 * c_T_0_pusher * n_p .^ 2;
end
