clc; clear all; close all;
maneuver_types = ["pitch_211"];
data_type = "val";
load_data;

% Plot first validation data
[aoa, c_L, c_D] = calc_lift_drag(u, w, c_X, c_Z);
figure
subplot(1,3,1)
scatter(rad2deg(aoa), c_L, 10, delta_e, 'filled'); alpha(0.5); hold on;
subplot(1,3,2)
scatter(rad2deg(aoa), c_D, 10, delta_e, 'filled'); alpha(0.5); hold on;
subplot(1,3,3)
scatter(rad2deg(aoa), c_m, 10, delta_e, 'filled'); alpha(0.5); hold on;
hcb=colorbar;
title(hcb,'\delta_e')


% Calculate X and Z aerodynamic coefficients for varying w speeds
% X and Z body forces are not dependent on u, so changing u will not change
% the outcome.

equation_error_results_lon;


V_nom = 21;
w = (-2:0.01:5) / V_nom;
u = 21 / V_nom;

c_X = c_X_0 + c_X_w * w + c_X_w_sq * w.^2;
c_Z = c_Z_0 + c_Z_w * w + c_Z_w_sq * w.^2;
c_m = c_m_0 + c_m_w * w;

[aoa, c_L, c_D] = calc_lift_drag(u, w, c_X, c_Z);

subplot(1,3,1)
plot(rad2deg(aoa), c_L, 'LineWidth', 2)
title("Lift  coefficient")
xlabel("\alpha [deg]")

subplot(1,3,2)
plot(rad2deg(aoa), c_D, 'LineWidth', 2)
title("Drag coefficient")
xlabel("\alpha [deg]")

subplot(1,3,3)
plot(rad2deg(aoa), c_m, 'LineWidth', 2)
title("Pitch moment coefficient")
xlabel("\alpha [deg]")

function [aoa, c_L, c_D] = calc_lift_drag(u, w, c_X, c_Z)
    aoa = atan(w./u);
    c = zeros(2, length(c_X));
    for i = 1:length(aoa)
        R = [cos(-aoa(i)) -sin(-aoa(i));
             sin(-aoa(i)) cos(-aoa(i))];
        c(:,i) = R * [c_X(i); c_Z(i)];
    end

    c_D = -c(1,:);
    c_L = -c(2,:);
end