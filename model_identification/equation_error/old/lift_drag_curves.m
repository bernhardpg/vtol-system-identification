clc; clear all; close all;
maneuver_types = ["pitch_211"];
data_type = "val";
load_data;

% Plot first validation data
figure
subplot(1,3,1)
scatter(rad2deg(aoa_alpha), c_L, 10, 'filled'); alpha(0.5); hold on;
subplot(1,3,2)
scatter(rad2deg(aoa_alpha), c_D, 10, 'filled'); alpha(0.5); hold on;
subplot(1,3,3)
scatter(rad2deg(aoa_alpha), c_m, 10, 'filled'); alpha(0.5); hold on;

% Calculate X and Z aerodynamic coefficients for varying w speeds
% X and Z body forces are not dependent on u, so changing u will not change
% the outcome.

equation_error_results_lon;

aoa_alpha = deg2rad(-10:0.01:20);
c_L = c_L_0 +c_L_alpha * aoa_alpha + c_L_alpha_sq * aoa_alpha.^2;
c_D = c_D_0 +c_D_alpha * aoa_alpha + c_D_alpha_sq * aoa_alpha.^2;
c_m = c_m_0 + c_m_alpha * aoa_alpha;

subplot(1,3,1)
plot(rad2deg(aoa_alpha), c_L, 'LineWidth', 2)
title("Lift  coefficient")
xlabel("\alpha [deg]")

subplot(1,3,2)
plot(rad2deg(aoa_alpha), c_D, 'LineWidth', 2)
title("Drag coefficient")
xlabel("\alpha [deg]")

subplot(1,3,3)
plot(rad2deg(aoa_alpha), c_m, 'LineWidth', 2)
title("Pitch moment coefficient")
xlabel("\alpha [deg]")

function [aoa_alpha, c_L, c_D] = calc_lift_drag(u, w, c_X, c_Z)
    aoa_alpha = atan(w./u);
    c = zeros(2, length(c_X));
    for i = 1:length(aoa_alpha)
        R = [cos(-aoa_alpha(i)) -sin(-aoa_alpha(i));
             sin(-aoa_alpha(i)) cos(-aoa_alpha(i))];
        c(:,i) = R * [c_X(i); c_Z(i)];
    end

    c_D = -c(1,:);
    c_L = -c(2,:);
end