clc; clear all; close all;
maneuver_type = "roll_211";
load_data;

% TODO: Remove maneuver with CRAZY high c_Y!!
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
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat";
vars_to_test_str = "p_hat r_hat delta_a delta_r";
vars_to_test = [p_hat r_hat delta_a delta_r];
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat p_hat";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "r_hat delta_a delta_r";
vars_to_test = [r_hat delta_a delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat r_hat]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat p_hat r_hat";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "delta_a delta_r";
vars_to_test = [delta_a delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat r_hat delta_a]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat p_hat r_hat delta_a";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "delta_r";
vars_to_test = [delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat r_hat delta_a delta_r]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat p_hat r_hat delta_a delta_r";
[c_Y_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

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
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) delta_a]; % Regressor
z = c_l; % Output measurements
indep_vars_str = "1 delta_a";
vars_to_test_str = "v_hat p_hat r_hat delta_r";
vars_to_test = [v_hat p_hat r_hat delta_r];
[c_l_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str, 0);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) r_hat delta_a]; % Regressor
z = c_l; % Output measurements
indep_vars_str = "1 r_hat delta_a";
vars_to_test_str = "v_hat p_hat delta_r";
vars_to_test = [v_hat p_hat delta_r];
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) p_hat r_hat delta_a]; % Regressor
z = c_l; % Output measurements
indep_vars_str = "1 p_hat r_hat delta_a";
vars_to_test_str = "v_hat delta_r";
vars_to_test = [v_hat delta_r];
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat r_hat delta_a delta_r]; % Regressor
z = c_l; % Output measurements
indep_vars_str = "1 v_hat p_hat r_hat delta_a delta_r";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);

fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_l, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_l_hat); hold on
legend("c_X", "c_X (predicted)")

