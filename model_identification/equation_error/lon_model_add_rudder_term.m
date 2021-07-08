%% Find delta_r dependence of c_M

clc; clear all; close all;

% Load training data
maneuver_types = ["yaw_211"];
data_type = "train";
load_data;

% Create explanatory variables
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);
t_plot = 0:dt:length(t)*dt-dt;

delta_e = (delta_vl + delta_vr) / 2;
delta_r = (-delta_vl + delta_vr) / 2;

N = length(c_m);
equation_error_results_lon;

% Calculate c_m predicted with equation error and add rudder term
curr_regr = [ones(N,1) aoa_alpha q_hat delta_e delta_e.^2];
%th = [c_m_0 c_m_alpha c_m_q c_m_delta_e c_m_delta_e_sq]';
th = [0.0076 -1.2589 -21.3597 -0.8543 -0.4868]'; %From output error
c_m_hat = curr_regr * th;

RSS = calc_RSS(c_m_hat, c_m)
R_sq = calc_R_sq(c_m_hat, c_m);

% plot(c_m); hold on
% plot(c_m_hat);

% Predict residual with delta_r as regressor
z = c_m_hat - c_m;
c_m_delta_r = LSE(delta_r, z);


curr_regr = [ones(N,1) aoa_alpha q_hat delta_e delta_e.^2 delta_r];
th = [c_m_0 c_m_alpha c_m_q c_m_delta_e c_m_delta_e_sq c_m_delta_r]';
c_m_hat = curr_regr * th;
RSS = calc_RSS(c_m_hat, c_m)
R_sq = calc_R_sq(c_m_hat, c_m);

plot(c_m); hold on
plot(c_m_hat);

