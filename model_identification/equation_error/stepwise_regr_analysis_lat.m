clc; close all; clear all;

%%%
% This script calculates the best model for lateral coeffs using
% stepwise-regression. This is only based on the data. If this method gives
% a simpler model, this model is preferred!

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lat.mat");

% Collect data from multiple experiments into one long dataset
% Note: Ordering does not matter for equation error

maneuver_types = [
    "roll_211",...
    "yaw_211",...
    ];

dt = fpr_data_lat.training.roll_211(1).Dt;

dependent_variable_names = ["C_Y" "C_l" "C_n"];
independent_variable_names = ["AngPHat" "AngRHat" "VelVHat" "DeltaA" "DeltaR"];
regr_names = ["p_hat" "r_hat" "v_hat" "delta_a" "delta_r"];
nonlin_regr_names = ["v_sq"];

%%%
% Load training data
%%%

zs = {};
for data_type = dependent_variable_names
    zs.(data_type) = collect_data_from_multiple_maneuvers(fpr_data_lat.training, maneuver_types, data_type);
end

regr = [];
for data_type = independent_variable_names
    regr = [regr collect_data_from_multiple_maneuvers(fpr_data_lat.training, maneuver_types, data_type)];
end

nonlin_regr = [regr(:,3) .^ 2];

%%%
% Load validation data
%%%
zs_val = {};
for data_type = dependent_variable_names
    zs_val.(data_type) = collect_data_from_multiple_maneuvers(fpr_data_lat.validation, maneuver_types, data_type);
end

regr_val = [];
for data_type = independent_variable_names
    regr_val = [regr_val collect_data_from_multiple_maneuvers(fpr_data_lat.validation, maneuver_types, data_type)];
end

nonlin_regr_val = [regr_val(:,3) .^ 2];
[N_val, ~] = size(regr_val);
t_plot_val = 0:dt:N_val * dt - dt;

% For coefficient storage
std_regr_order_lat = ["bias" "v_hat" "p_hat" "r_hat" "delta_a" "delta_r"];

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

min_r_sq_change = 2; % Demand at least X% improvement to add a regressor

z = zs.C_Y;
z_val = zs_val.C_Y;
[th_hat, th_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_Y", th_hat, th_names);

% Store coeff values
c_Y = create_coeff_vector(std_regr_order_lat, th_hat, th_names);

fig = figure;
subplot(3,1,1)
plot(t_plot_val, z_val, t_plot_val, y_hat_val); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_Y: " + "R^2 = " + R_sq_val + "%" + ", vars: " + join(th_names))

z = zs.C_l;
z_val = zs_val.C_l;
[th_hat, th_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_l", th_hat, th_names);

% Store coeff values
c_l = create_coeff_vector(std_regr_order_lat, th_hat, th_names);

subplot(3,1,2)
plot(t_plot_val, z_val, t_plot_val, y_hat_val); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_l: " + "R^2 = " + R_sq_val + "%" + ", vars: "  + join(th_names))

z = zs.C_n;
z_val = zs_val.C_n;
[th_hat, th_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change);
print_eq_error_params("c_n", th_hat, th_names);

% Store coeff values
c_n = create_coeff_vector(std_regr_order_lat, th_hat, th_names);

subplot(3,1,3)
plot(t_plot_val, z_val, t_plot_val, y_hat_val); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')
title("c_n: " + "R^2 = " + R_sq_val + "%" + ", vars: "  + join(th_names))

plot_location = 'data/flight_data/selected_data/lateral_directional_data/equation_error_fit/';
sgtitle("Stepwise-Regression Equation-Error Lateral model")
filename = "equation_error_fit";
saveas(fig, plot_location + filename, 'epsc')

% Save coeffs to file
equation_error_coeffs_lat = [c_Y c_l c_n];

save("model_identification/equation_error/results/equation_error_coeffs_lat.mat", "equation_error_coeffs_lat");

function coeff_vector = create_coeff_vector(std_regr_order, th_hat, th_names)
    n_total_regr = length(std_regr_order);
    coeff_vector = zeros(n_total_regr,1);
    % Iterate through found regressors
    for regr_i = 1:length(th_hat)
        curr_regr_name = th_names(regr_i);
        % Find the right place in the coeff vector
        for coeff_i = 1:n_total_regr
            if strcmp(curr_regr_name, std_regr_order(coeff_i)) % found right place in coeff matrix
                coeff_vector(coeff_i) = th_hat(regr_i);
                break
            end
        end
    end
end