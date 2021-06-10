clc; clear all; close all;
maneuver_types = ["roll_211" "yaw_211"];
load_data;

%%
%%%
% Find most relevant terms for c_Y:
%%%

clc;
N = length(c_Y);

X = [ones(N, 1)]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);

vars_to_test_str = "v_hat p_hat r_hat delta_a delta_r";
vars_to_test = [v_hat p_hat r_hat delta_a delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "p_hat r_hat delta_a delta_r";
vars_to_test = [p_hat r_hat delta_a delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat delta_r]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat delta_r";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "p_hat r_hat delta_a";
vars_to_test = [p_hat r_hat delta_a];
explore_next_var(z, vars_to_test, vars_to_test_str);

% delta_a gives too low increase in R_sq
X = [ones(N, 1) v_hat delta_a delta_r]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat delta_a delta_r";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "p_hat r_hat delta_a";
vars_to_test = [p_hat r_hat delta_a];
explore_next_var(z, vars_to_test, vars_to_test_str);

% r_hat gives too low increase
X = [ones(N, 1) v_hat r_hat delta_r]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat r_hat delta_r";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

% p_hat gives somewhat good increase
X = [ones(N, 1) v_hat p_hat delta_r]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat p_hat delta_r";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

disp("Trying nonlinear terms")
vars_to_test_str = "v_hat.^2 p_hat.^2 delta_r.^2";
vars_to_test = [v_hat.^2 p_hat.^2 delta_r.^2];
explore_next_var(z, vars_to_test, vars_to_test_str);

% Final model design
X = [ones(N, 1) v_hat p_hat delta_r]; % Regressor
z = c_Y; % Output measurements
indep_vars_str = "1 v_hat p_hat delta_r";
[c_Y_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

th_hat = num2cell(th_hat);
[c_Y_0, c_Y_v, c_Y_p, c_Y_delta_r] = th_hat{:}

fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_Y, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_Y_hat); hold on
legend("c_Y", "c_Y (predicted)")

%%

%%%
% Find most relevant terms for c_l:
%%%

clc;
N = length(c_l);
z = c_l; % Output measurements

X = [ones(N, 1)]; % Regressor
indep_vars_str = "1";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);

vars_to_test_str = "v_hat p_hat r_hat delta_a delta_r";
vars_to_test = [v_hat p_hat r_hat delta_a delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) delta_a]; % Regressor
indep_vars_str = "1 delta_a";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "v_hat p_hat r_hat delta_r";
vars_to_test = [v_hat p_hat r_hat delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) r_hat delta_a]; % Regressor
indep_vars_str = "1 r_hat delta_a";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "v_hat p_hat delta_r";
vars_to_test = [v_hat p_hat delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) p_hat r_hat delta_a]; % Regressor
indep_vars_str = "1 p_hat r_hat delta_a";
[c_l_hat, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "v_hat delta_r";
vars_to_test = [v_hat delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

% Final model design
X = [ones(N, 1) v_hat p_hat r_hat delta_a]; % Regressor
indep_vars_str = "1 v_hat p_hat r_hat delta_a";
[c_l_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

% vars_to_test_str = "delta_r";
% vars_to_test = [delta_r];
% explore_next_var(z, vars_to_test, vars_to_test_str);
% % 
% % Too low with delta_r, do not include this
% X = [ones(N, 1) v_hat p_hat r_hat delta_a delta_r]; % Regressor
% indep_vars_str = "1 v_hat p_hat r_hat delta_a delta_r";
% [c_l_hat, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

% disp("Trying non-linear terms")
% vars_to_test_str = "v_hat.^2 p_hat.^ r_hat.^2 delta_a.^2";
% vars_to_test = [v_hat.^2 p_hat.^2 r_hat.^2 delta_a.^2];
% explore_next_var(z, vars_to_test, vars_to_test_str);

% % Too low with delta_a.^2
% X = [ones(N, 1) v_hat p_hat r_hat delta_a delta_a.^2]; % Regressor
% indep_vars_str = "1 v_hat p_hat r_hat delta_a delta_a.^2";
% [c_l_hat, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
% 
% % Too low with v_hat.^2
% X = [ones(N, 1) v_hat v_hat.^2 p_hat r_hat delta_a]; % Regressor
% indep_vars_str = "1 v_hat v_hat.^2 p_hat r_hat delta_a";
% [c_l_hat, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
% 
% % Too low with p_hat.^2
% X = [ones(N, 1) v_hat p_hat p_hat.^2 r_hat delta_a]; % Regressor
% indep_vars_str = "1 v_hat p_hat p_hat.^2 r_hat delta_a";
% [c_l_hat, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
% 
% % Too low with r_hat.^2
% X = [ones(N, 1) v_hat p_hat r_hat.^2 r_hat delta_a]; % Regressor
% indep_vars_str = "1 v_hat p_hat r_hat.^2 r_hat delta_a";
% [c_l_hat, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);



th_hat = num2cell(th_hat);
[c_l_0, c_l_v, c_l_p, c_l_r, c_l_delta_a] = th_hat{:}


fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_l, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_l_hat); hold on
legend("c_l", "c_l (predicted)")


%%

%%%
% Find most relevant terms for c_n:
%%%

clc;
N = length(c_n);
z = c_n; % Output measurements

X = [ones(N, 1)]; % Regressor
indep_vars_str = "1";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, 0);

vars_to_test_str = "v_hat p_hat r_hat delta_a delta_r";
vars_to_test = [v_hat p_hat r_hat delta_a delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat]; % Regressor
indep_vars_str = "1 v_hat";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "p_hat r_hat delta_a delta_r";
vars_to_test = [p_hat r_hat delta_a delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat]; % Regressor
indep_vars_str = "1 v_hat p_hat";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

vars_to_test_str = "r_hat delta_a delta_r";
vars_to_test = [r_hat delta_a delta_r];
explore_next_var(z, vars_to_test, vars_to_test_str);

X = [ones(N, 1) v_hat p_hat delta_r]; % Regressor
indep_vars_str = "1 v_hat p_hat delta_r";
[~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

X = [ones(N, 1) v_hat p_hat r_hat delta_r]; % Regressor
indep_vars_str = "1 v_hat p_hat r_hat delta_r";
[c_n_hat, th_hat, cov_th, F0, R_sq] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

% delta_a gives too little increase in R_sq
% X = [ones(N, 1) v_hat p_hat r_hat delta_a delta_r]; % Regressor
% indep_vars_str = "1 v_hat p_hat r_hat delta_a delta_r";
% [~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);

% little effect with nonlinear terms
% X = [ones(N, 1) v_hat p_hat r_hat delta_r delta_r.^2]; % Regressor
% indep_vars_str = "1 v_hat p_hat r_hat delta_r delta_r.^2";
% [~, ~, ~, ~, R_sq_prev] = stepwise_regression_round(X, z, indep_vars_str, R_sq_prev);
% 

th_hat = num2cell(th_hat);
[c_n_0, c_n_v, c_n_p, c_n_r, c_n_delta_r] = th_hat{:}


fig = figure;
fig.Position = [100 100 1700 500];
plot(t_plot, c_n, '--'); hold on;
xlabel("time [s]")
plot(t_plot, c_n_hat); hold on
legend("c_n", "c_n (predicted)")
