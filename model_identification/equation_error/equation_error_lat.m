clc; clear all; close all;
%%%
% Load training data
%%%
maneuver_types = ["roll_211" "yaw_211"];
data_type = "train";
load_data;

% Create explanatory variables
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
t_plot = 0:dt:length(t)*dt-dt;

% Basis regressors
delta_r = (-delta_vl + delta_vr) / 2; % Actually use control surface deflections, not commanded rudder input
regr = [p_hat r_hat beta delta_a delta_r]; % Basis regressors
regr_names = ["p" "r" "beta" "delta_a" "delta_r"];

% Dependent variables
zs = [c_Y c_l c_n];

% Force n moment to include function dependency on r
N = length(c_n);
X_n_moment = [ones(N,1) beta p_hat r_hat delta_a delta_r];
th_names = ["1" "beta" "p" "r" "delta_a" "delta_r"];
th_hat = LSE(X_n_moment, c_n);
y_hat = X_n_moment * th_hat;
RSS = calc_RSS(y_hat, c_n);
R_sq = calc_R_sq(y_hat, c_n);
print_eq_error_params("c_n", th_hat, th_names);

%%%
% Load validation data
%%%
maneuver_types = ["roll_211" "yaw_211"];
data_type = "val";
load_data;

[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);

delta_r = (-delta_vl + delta_vr) / 2;
regr_val = [p_hat r_hat beta delta_a delta_r]; % Basis regressors
% Dependent variables
zs_val = [c_Y c_l c_n];



%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%


min_r_sq_change = 0.65; % Demand at least 2% improvement to add a regressor

z = zs(:,1);
z_val = zs_val(:,1);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_Y", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_Y")


z = zs(:,2);
z_val = zs_val(:,2);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_l", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_l")

z = zs(:,3);
z_val = zs_val(:,3);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_n", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_n")

function [RSS] = calc_RSS(y_hat, z)
    % Calculate Regression Sum of Squares
    RSS = (y_hat - z)' * (y_hat - z);
end

function [TSS] = calc_TSS(z)
   % Calculate total Sum of Squares
    z_bar = mean(z);
    TSS = (z - z_bar)' * (z - z_bar);
end

function [R_sq] = calc_R_sq(y_hat, z)
    RSS = calc_RSS(y_hat, z);
    TSS = calc_TSS(z);
    
    % Coefficient of Determination
    R_sq = (1 - (RSS / TSS)) * 100;
end

function [th_hat] = LSE(X, z)
    D = (X' * X)^(-1);
    th_hat = D * X' * z;
end