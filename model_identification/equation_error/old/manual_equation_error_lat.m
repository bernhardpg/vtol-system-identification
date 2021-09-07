clear all;

%%%
% This script tests how well a "textbook model" for the lateral dynamics
% works on the validation data. The purpose of this is to compare with
% Stepwise-regression method, which works purely analytically.

%%%
% Load validation data
%%%

disp("## Standard equation-error for lon model")

maneuver_types = ["roll_211" "yaw_211"];
data_type = "val";
load_data;
c_Y_val = c_Y;
c_l_val = c_l;
c_n_val = c_n;
t_val_plot = 0:dt:length(t)*dt-dt;

N = length(c_Y);
X_val = [ones(N,1) beta p_hat r_hat delta_a delta_r]; % Needs to be changed if separate regressors are to be used for separate coeffs

%%% Plot figures

%%%
% Load training data
%%%

data_type = "train";
load_data;

% Basis regressors
N = length(c_Y);
X = [ones(N,1) beta p_hat r_hat delta_a delta_r];
th_names = ["1" "beta" "p" "r" "delta_a" "delta_r"];


figure

% c_Y
z = c_Y;
z_val = c_Y_val;
N = length(z);
name = "c_Y";

th_hat = LSE(X, z);
y_hat = X_val * th_hat;
RSS = calc_RSS(y_hat, z_val);
R_sq = calc_R_sq(y_hat, z_val);

subplot(3,1,1)
plot(t_val_plot, z_val, t_val_plot, y_hat); hold on
legend(name, name + " hat", 'Interpreter','latex')
title(name + " " + "R^2 = " + R_sq + "%")
print_eq_error_params(name, th_hat, th_names);

% c_l
z = c_l;
z_val = c_l_val;
N = length(z);
name = "c_l";

th_hat = LSE(X, z);
y_hat = X_val * th_hat;
RSS = calc_RSS(y_hat, z_val);
R_sq = calc_R_sq(y_hat, z_val);

subplot(3,1,2)
plot(t_val_plot, z_val, t_val_plot, y_hat); hold on
legend(name, name + " hat", 'Interpreter','latex')
title(name + " " + "R^2 = " + R_sq + "%")
print_eq_error_params(name, th_hat, th_names);


% c_n
z = c_n;
z_val = c_n_val;
N = length(z);
name = "c_n";

th_hat = LSE(X, z);
y_hat = X_val * th_hat;
RSS = calc_RSS(y_hat, z_val);
R_sq = calc_R_sq(y_hat, z_val);

subplot(3,1,3)
plot(t_val_plot, z_val, t_val_plot, y_hat); hold on
legend(name, name + " hat", 'Interpreter','latex')
title(name + " " + "R^2 = " + R_sq + "%")
print_eq_error_params(name, th_hat, th_names);

sgtitle("Standard Equation-Error Lateral Model")