clc; clear all; close all;
maneuver_types = ["pitch_211"];
load_data;

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

z = c_Z; % output
N = length(z);

X = [ones(N, 1)]; % regressors

r = 


function [r] = calc_corr_coeff(X, z)
    X_bar = mean(X);
    N = length(z);
    z_bar = mean(z);
    
    cov_Xz = (X - X_bar)' * (z - z_bar) / (N - 1);
    var_X = diag((X - X_bar)' * (X - X_bar)) / (N - 1);
    var_z = diag((z - z_bar)' * (z - z_bar)) / (N - 1);
    r = cov_Xz ./ sqrt(var_X * var_z);
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
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);
explore_next_var(z, vars_to_test, vars_to_test_str);

% Add w as independent variable
X = [ones(N, 1) w_hat]; % Regressor
indep_vars_str = "1 w_hat";
vars_to_test_str = "u_hat q_hat delta_e n_p";
vars_to_test = [u_hat q_hat delta_e n_p];
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
explore_next_var(z, vars_to_test, vars_to_test_str);

% Add q_hat as independent variable
X = [ones(N, 1) w_hat q_hat]; % Regressor
indep_vars_str = "1 w_hat q_hat";
vars_to_test_str = "u_hat delta_e n_p";
vars_to_test = [u_hat delta_e n_p];
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
explore_next_var(z, vars_to_test, vars_to_test_str);

% Add q_hat as independent variable
X = [ones(N, 1) w_hat q_hat delta_e]; % Regressor
indep_vars_str = "1 w_hat q_hat delta_e";
vars_to_test_str = "u_hat n_p";
vars_to_test = [u_hat n_p];
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
explore_next_var(z, vars_to_test, vars_to_test_str);

disp("Try nonlinear terms")
vars_to_test_str = "w_hat.^2 q_hat.^2 delta_e.^2";
vars_to_test = [w_hat.^2 q_hat.^2 delta_e.^2];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat w_hat.^2 q_hat delta_e]; % Regressor
indep_vars_str = "1 w_hat w_hat.^2 q_hat delta_e";
[c_Z_hat, th_hat, cov_th, F0, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "q_hat.^2 delta_e.^2";
vars_to_test = [q_hat.^2 delta_e.^2];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat w_hat.^2 q_hat delta_e delta_e.^2]; % Regressor
indep_vars_str = "1 w_hat w_hat.^2 q_hat delta_e delta_e.^2";
[c_Z_hat, th_hat, cov_th, F0, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "q_hat.^2";
vars_to_test = [delta_e.^2];
explore_next_var(z, vars_to_test, vars_to_test_str);

% Becomes too low:
% X = [ones(N, 1) w_hat w_hat.^2 q_hat q_hat.^2 delta_e delta_e.^2]; % Regressor
% indep_vars_str = "1 w_hat w_hat.^2 q_hat q_hat.^2 delta_e delta_e.^2";
% [c_Z_hat, th_hat, cov_th, F0, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

th_hat = num2cell(th_hat);
[c_Z_0, c_Z_w, c_Z_w_sq, c_Z_q, c_Z_delta_e, c_Z_delta_e_sq] = th_hat{:}

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
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat]; % Regressor
indep_vars_str = "1 w_hat";
vars_to_test_str = "u_hat q_hat delta_e n_p";
vars_to_test = [u_hat q_hat delta_e n_p];
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat delta_e]; % Regressor
indep_vars_str = "1 w_hat delta_e";
vars_to_test_str = "u_hat q_hat n_p";
vars_to_test = [u_hat q_hat n_p];
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat q_hat delta_e]; % Regressor
indep_vars_str = "1 w_hat delta_e q_hat";
[c_m_hat, th_hat, cov_th, F0, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "w_hat.^2 delta_e.^2";
vars_to_test = [w_hat.^2 delta_e.^2];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) w_hat q_hat delta_e delta_e.^2]; % Regressor
indep_vars_str = "1 w_hat delta_e delta_e.^2 q_hat";
[c_m_hat, th_hat, cov_th, F0, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

% Gives too low change in R_sq
% X = [ones(N, 1) w_hat w_hat.^2 q_hat delta_e]; % Regressor
% indep_vars_str = "1 w_hat w_hat.^2 delta_e q_hat";
% [c_m_hat, th_hat, cov_th, F0, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

th_hat = num2cell(th_hat);
[c_m_0, c_m_w, c_m_q, c_m_delta_e c_m_delta_e_sq] = th_hat{:}

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
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat]; % Regressor
indep_vars_str = "1 u_hat";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
vars_to_test_str = " w_hat q_hat delta_e n_p";
vars_to_test = [w_hat q_hat delta_e n_p];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat w_hat]; % Regressor
indep_vars_str = "1 u_hat w_hat";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
vars_to_test_str = "q_hat delta_e";
vars_to_test = [q_hat delta_e];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat w_hat q_hat]; % Regressor
indep_vars_str = "1 u_hat w_hat q_hat";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
vars_to_test_str = "delta_e";
vars_to_test = [delta_e];
explore_next_var(z, vars_to_test, vars_to_test_str);

disp("Nonlinear terms");

vars_to_test_str = "u_hat.^2 w_hat.^2 n_p.^2 n_p * u_hat";
vars_to_test = [u_hat.^2 w_hat.^2 n_p.^2 n_p .* u_hat];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat w_hat w_hat.^2 q_hat]; % Regressor
indep_vars_str = "1 u_hat w_hat w_hat.^2 q_hat";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
vars_to_test_str = "u_hat.^2 n_p.^2 n_p .* u_hat";
vars_to_test = [u_hat.^2 n_p.^2 n_p .* u_hat];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat u_hat.^2 w_hat w_hat.^2 q_hat]; % Regressor
indep_vars_str = "1 u_hat u_hat.^2 w_hat w_hat.^2 q_hat";
[c_X_hat, th_hat, cov_th, F0, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "n_p.^2";
vars_to_test = [n_p.^2];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) u_hat u_hat.^2 w_hat w_hat.^2 q_hat n_p]; % Regressor
indep_vars_str = "1 u_hat u_hat.^2 w_hat w_hat.^2 q_hat n_p";
[c_X_hat, th_hat, cov_th, F0, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
vars_to_test_str = "n_p.^2";
vars_to_test = [n_p.^2];
explore_next_var(z, vars_to_test, vars_to_test_str);

% Seems to be better with linear n_p term
% X = [ones(N, 1) u_hat u_hat.^2 w_hat w_hat.^2 q_hat n_p.^2]; % Regressor
% indep_vars_str = "1 u_hat u_hat.^2 w_hat w_hat.^2 q_hat n_p.^2";
% [c_X_hat, th_hat, cov_th, F0, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

th_hat = num2cell(th_hat);
[c_X_0, c_X_u, c_X_u_sq, c_X_w, c_X_w_sq, c_X_q, c_X_n_p] = th_hat{:}

fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_X, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_X_hat); hold on
legend("c_X", "c_X (predicted)")