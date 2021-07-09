clear all;

%%%
% This script tests how well a "textbook model" for the lateral dynamics
% works on the validation data. The purpose of this is to compare with
% Stepwise-regression method, which works purely analytically.

%%%
% Load validation data
%%%

disp("## Standard equation-error for lon model")

maneuver_types = ["pitch_211"];
data_type = "val";
load_data;
c_D_val = c_D;
c_L_val = c_L;
c_m_val = c_m;
t_val_plot = 0:dt:length(t)*dt-dt;

N = length(c_Y);
X_val_D = [ones(N,1) aoa_alpha aoa_alpha.^2 q_hat V_hat delta_e]; % Needs to be changed if separate regressors are to be used for separate coeffs
X_val_L = [ones(N,1) aoa_alpha aoa_alpha.^2 q_hat V_hat delta_e]; % Needs to be changed if separate regressors are to be used for separate coeffs
X_val_m = [ones(N,1) aoa_alpha q_hat V_hat delta_e]; % Needs to be changed if separate regressors are to be used for separate coeffs

%%% Plot figures

%%%
% Load training data
%%%

data_type = "train";
load_data;

% Basis regressors
N = length(c_Y);
X_D = [ones(N,1) aoa_alpha aoa_alpha.^2 q_hat V_hat delta_e]; % Needs to be changed if separate regressors are to be used for separate coeffs
X_L = [ones(N,1) aoa_alpha aoa_alpha.^2 q_hat V_hat delta_e]; % Needs to be changed if separate regressors are to be used for separate coeffs
X_m = [ones(N,1) aoa_alpha q_hat V_hat delta_e]; % Needs to be changed if separate regressors are to be used for separate coeffs

th_names_D = ["1" "alpha" "alpha_sq" "q" "V" "delta_e"];
th_names_L = ["1" "alpha" "alpha_sq" "q" "V" "delta_e"];
th_names_m = ["1" "alpha" "q" "V" "delta_e"];

figure

% c_D
z = c_D;
z_val = c_D_val;
N = length(z);
name = "c_D";

th_hat = LSE(X_D, z);
y_hat = X_val_D * th_hat;
RSS = calc_RSS(y_hat, z_val);
R_sq = calc_R_sq(y_hat, z_val);

subplot(3,1,1)
plot(t_val_plot, z_val, t_val_plot, y_hat); hold on
legend(name, name + " hat", 'Interpreter','latex')
title(name + " " + "R^2 = " + R_sq + "%")
print_eq_error_params(name, th_hat, th_names_D);

% c_L
z = c_L;
z_val = c_L_val;
N = length(z);
name = "c_L";

th_hat = LSE(X_L, z);
y_hat = X_val_L * th_hat;
RSS = calc_RSS(y_hat, z_val);
R_sq = calc_R_sq(y_hat, z_val);

subplot(3,1,2)
plot(t_val_plot, z_val, t_val_plot, y_hat); hold on
legend(name, name + " hat", 'Interpreter','latex')
title(name + " " + "R^2 = " + R_sq + "%")
print_eq_error_params(name, th_hat, th_names_L);


% c_m
z = c_m;
z_val = c_m_val;
N = length(z);
name = "c_m";

th_hat = LSE(X_m, z);
y_hat = X_val_m * th_hat;
RSS = calc_RSS(y_hat, z_val);
R_sq = calc_R_sq(y_hat, z_val);

subplot(3,1,3)
plot(t_val_plot, z_val, t_val_plot, y_hat); hold on
legend(name, name + " hat", 'Interpreter','latex')
title(name + " " + "R^2 = " + R_sq + "%")
print_eq_error_params(name, th_hat, th_names_m);

sgtitle("Standard Equation-Error Longitudinal Model")