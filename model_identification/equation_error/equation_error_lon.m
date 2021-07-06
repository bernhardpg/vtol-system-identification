clc; clear all; close all;
% Load training data
maneuver_types = ["pitch_211"];
data_type = "train";
load_data;

% Create explanatory variables
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
t_plot = 0:dt:length(t)*dt-dt;

% Basis regressors
delta_e = (delta_vl + delta_vr) / 2;
regr = [alpha q_hat delta_e];
regr_names = ["alpha" "q" "delta_e"];

% Dependent variables
zs = [c_D c_L c_m];

% Force L to include function dependency on q
% TODO: Clean up this code part?
N = length(c_L);
X_c_L = [ones(N,1) alpha alpha.^2 q_hat delta_e];
th_names = ["1" "alpha" "alpha_sq" "q_hat" "delta_e"];
th_hat = LSE(X_c_L, c_L);
y_hat = X_c_L * th_hat;
RSS = calc_RSS(y_hat, c_L);
R_sq = calc_R_sq(y_hat, c_L);
print_eq_error_params("c_L", th_hat, th_names);

% Load validation data
data_type = "val";
load_data;

[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
delta_e = (delta_vl + delta_vr) / 2;
regr_val = [alpha q_hat delta_e];
zs_val = [c_D c_L c_m];

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

min_r_sq_change = 1.5; % Demand at least 2% improvement to add a regressor

z = zs(:,1);
z_val = zs_val(:,1);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_D", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_D")


z = zs(:,2);
z_val = zs_val(:,2);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_L", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_L")

z = zs(:,3);
z_val = zs_val(:,3);
[th_hat, th_names, y_hat, R_sq] = stepwise_regression(z, z_val, regr, regr_val, regr_names, min_r_sq_change);
print_eq_error_params("c_m", th_hat, th_names);

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_m")

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
