clc; clear all; close all;
%% Finding coefficients;
% c_D and c_M are found automatically by using stepwise regression, see stepwise_regr_lon.m
% c_L is found manually in this script

% Load training data
maneuver_types = ["pitch_211"];
data_type = "train";
load_data;

% Create explanatory variables
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
t_plot = 0:dt:length(t)*dt-dt;

% Basis regressors
delta_e = (delta_vl + delta_vr) / 2;
regr = [aoa_alpha q_hat delta_e];
regr_names = ["alpha" "q" "delta_e"];

% Dependent variables
zs = [c_D c_L c_m];


% Force L to include function dependency on q
N = length(c_L);
X_c_L = [ones(N,1) aoa_alpha aoa_alpha.^2 q_hat delta_e];
th_names = ["1" "alpha" "alpha_sq" "q_hat" "delta_e"];
th_hat = LSE(X_c_L, c_L);
y_hat = X_c_L * th_hat;
RSS = calc_RSS(y_hat, c_L);
R_sq = calc_R_sq(y_hat, c_L);
print_eq_error_params("c_L", th_hat, th_names);
