clc; close all; clear all;

load("model_identification/output_error/results/output_error_coeffs_lon.mat");

lon_coeffs = equation_error_coeffs_lon;

airframe_static_properties; % get alpha_nom
% Remember that model is around trim. Therefore, we must subtract alpha_nom
% when calculating lift and drag coeffs
alpha_nom = alpha_nom * 180 / pi;
delta_e_nom = delta_e_nom * 180 / pi;

c_D_0 = lon_coeffs(1);
c_D_alpha = lon_coeffs(2) * pi / 180;
c_D_alpha_sq = lon_coeffs(3) * (pi / 180)^2;
c_D_delta_e = lon_coeffs(5) * pi / 180;
c_D_alpha_delta_e = lon_coeffs(6) * (pi / 180)^2;

c_L_0 = lon_coeffs(7);
c_L_alpha = lon_coeffs(8) * pi / 180;
c_L_delta_e = lon_coeffs(11) * pi / 180;

c_m_0 = lon_coeffs(13);
c_m_alpha = lon_coeffs(14) * pi / 180;
c_m_delta_e = lon_coeffs(17) * pi / 180;




%%


%%%%%%%%%%%%%%%%%
% SURFACE PLOTS %
%%%%%%%%%%%%%%%%%

resolution = 1.5;

alpha_min = -7;
alpha_max = 15;
alpha = (alpha_min:resolution:alpha_max);

delta_e_min = -20;
delta_e_max = 20;
delta_e = delta_e_min:resolution:delta_e_max;

[alpha,delta_es] = meshgrid(alpha, delta_e);

c_L = calc_lift_coeff(c_L_0, c_L_alpha, c_L_delta_e, alpha, delta_es, alpha_nom, delta_e_nom);
c_D = calc_drag_coeff(c_D_0, c_D_alpha, c_D_alpha_sq, c_D_delta_e, c_D_alpha_delta_e, alpha, delta_es, alpha_nom, delta_e_nom);
c_m = calc_moment_coeff(c_m_0, c_m_alpha, c_m_delta_e, alpha, delta_es, alpha_nom, delta_e_nom);

% Plot lift coefficient
figure
plot_settings; % import plot settings
surf(alpha,delta_es,c_L); hold on
[Y,Z] = meshgrid(delta_e, min(min(c_L))-0.5:0.1:max(max(c_L)));
X = ones(size(Y)) * alpha_nom;
surf(X,Y,Z,'FaceColor','b','FaceAlpha',0.3,'EdgeColor','None'); hold on
text(alpha_nom+0.2, -20, -0.7, "$\alpha^*$",'interpreter','latex', 'FontSize',font_size_large)
xlabel("$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel("$\delta_e [^\circ]$",'interpreter','latex','FontSize',font_size)
zlabel("$c_L$",'interpreter','latex','FontSize',font_size_large)
title("Lift Coefficient",'FontSize',font_size_large, 'interpreter','latex')

% Plot drag coefficient
figure
surf(alpha,delta_es,c_D); hold on
[Y,Z] = meshgrid(delta_e, min(min(c_D))-0.5:0.1:max(max(c_D)));
X = ones(size(Y)) * alpha_nom;
surf(X,Y,Z,'FaceColor','b','FaceAlpha',0.3,'EdgeColor','None'); hold on
text(alpha_nom+0.2, -20, -0.7, "$\alpha^*$",'interpreter','latex', 'FontSize',font_size_large)
xlabel("$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel("$\delta_e [^\circ]$",'interpreter','latex','FontSize',font_size)
zlabel("$c_D$",'interpreter','latex','FontSize',font_size_large)
title("Drag Coefficient",'FontSize',font_size_large, 'interpreter','latex')

% Plot moment coefficient
figure
surf(alpha,delta_es,c_m); hold on
% Plot alpha_nom
[Y,Z] = meshgrid(delta_e, min(min(c_m))-0.5:0.1:max(max(c_m)));
X = ones(size(Y)) * alpha_nom;
surf(X,Y,Z,'FaceColor','b','FaceAlpha',0.3,'EdgeColor','None'); hold on
text(alpha_nom+0.2, -20, -0.7, "$\alpha^*$",'interpreter','latex', 'FontSize',font_size_large)

xlabel("$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel("$\delta_e [^\circ]$",'interpreter','latex','FontSize',font_size)
zlabel("$c_m$",'interpreter','latex','FontSize',font_size_large)
title("Pitch Moment Coefficient",'FontSize',font_size_large, 'interpreter','latex')

%%
%%%%%%%%%%%%%%%%%%%%
% 2D CONTOUR PLOTS %
%%%%%%%%%%%%%%%%%%%%

plot_settings; % import plot settings

load("data/flight_data/selected_data/fpr_data_lon.mat");
alpha_recorded = collect_data_from_multiple_maneuvers(fpr_data_lon.training, "pitch_211", "AlphaAbsolute") * 180/pi;
delta_e_recorded = collect_data_from_multiple_maneuvers(fpr_data_lon.training, "pitch_211", "DeltaE") * 180/pi;
c_L_recorded = collect_data_from_multiple_maneuvers(fpr_data_lon.training, "pitch_211", "C_L");
c_D_recorded = collect_data_from_multiple_maneuvers(fpr_data_lon.training, "pitch_211", "C_D");
c_m_recorded = collect_data_from_multiple_maneuvers(fpr_data_lon.training, "pitch_211", "C_m");




resolution = 0.5;
alpha_min = -10;
alpha_max = 15;
alpha = (alpha_min:resolution:alpha_max);

delta_es = [-15 delta_e_nom, 0, 5, 15];
delta_e_texts = "$\delta_e =" + ["-15^\circ$" "\delta_e^*$" "0^\circ$" "5^\circ$" "15^\circ$"];
colors = [0 0.4470 0.7410;
          0.8500 0.3250 0.0980;
          0.9290 0.6940 0.1250;
          0.4940 0.1840 0.5560;
          0.4660 0.6740 0.1880;
          0.3010 0.7450 0.9330;
          0.6350 0.0780 0.1840];

% Drag curve
fig = figure;
fig.Position = [100 100 1000 400];
t = tiledlayout(1,2);
nexttile
for delta_e = delta_es
    c_D = calc_drag_coeff(c_D_0, c_D_alpha, c_D_alpha_sq, c_D_delta_e, c_D_alpha_delta_e, alpha, delta_e, alpha_nom, delta_e_nom);
    plot(alpha, c_D,'LineWidth',line_width); hold on;
end
xline(alpha_nom,":"); hold on
legend(delta_e_texts,'interpreter','latex','Fontsize',font_size,'location','best')
text(alpha_nom+0.1,-0.3,"$\alpha^*$",'interpreter','latex','Fontsize',font_size)
title("Predicted",'FontSize',font_size, 'interpreter','latex')

nexttile
scatter(alpha_recorded, c_D_recorded,[],delta_e_recorded,'filled','MarkerFaceAlpha',0.3);
legend("Flight Data")
c = colorbar; hold on
c.Label.String = "$\delta_e$"; c.Label.Interpreter = 'latex'; c.Label.FontSize = font_size_large;
title("Measured",'FontSize',font_size, 'interpreter','latex')

xlabel(t,"$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel(t,"$c_D$",'interpreter','latex','FontSize',font_size_large)
title(t,"Drag Coefficient",'FontSize',font_size_large, 'interpreter','latex')

% Lift curve
fig = figure;
fig.Position = [100 100 1000 400];
t = tiledlayout(1,2);
nexttile
for delta_e = delta_es
    c_L = calc_lift_coeff(c_L_0, c_L_alpha, c_L_delta_e, alpha, delta_e, alpha_nom, delta_e_nom);
    plot(alpha, c_L); hold on;
end
xline(alpha_nom,":"); hold on
legend(delta_e_texts,'interpreter','latex','Fontsize',font_size,'location','best')
text(alpha_nom+0.1,-0.3,"$\alpha^*$",'interpreter','latex','Fontsize',font_size)
title("Predicted",'FontSize',font_size, 'interpreter','latex')

nexttile
scatter(alpha_recorded, c_L_recorded,[],delta_e_recorded,'filled','MarkerFaceAlpha',0.3);
legend("Flight Data")
c = colorbar; hold on
c.Label.String = "$\delta_e$"; c.Label.Interpreter = 'latex'; c.Label.FontSize = font_size_large;
title("Measured",'FontSize',font_size, 'interpreter','latex')

xlabel(t,"$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel(t,"$c_L$",'interpreter','latex','FontSize',font_size_large)
title(t,"Lift Coefficient",'FontSize',font_size_large, 'interpreter','latex')



% Pitch Moment curve
fig = figure;
fig.Position = [100 100 1000 400];
t = tiledlayout(1,2);
nexttile
for i = 1:length(delta_es)
    delta_e = delta_es(i);
    c_m = calc_moment_coeff(c_m_0, c_m_alpha, c_m_delta_e, alpha, delta_e, alpha_nom, delta_e_nom);
    plot(alpha, c_m+0.07); hold on;
end
xline(alpha_nom,":"); hold on
legend(delta_e_texts,'interpreter','latex','Fontsize',font_size,'location','best')
text(alpha_nom+0.1,-0.3,"$\alpha^*$",'interpreter','latex','Fontsize',font_size)
title("Predicted",'FontSize',font_size, 'interpreter','latex')

nexttile
scatter(alpha_recorded, c_m_recorded,[],delta_e_recorded,'filled','MarkerFaceAlpha',0.3);
legend("Flight Data")
c = colorbar; hold on
c.Label.String = "$\delta_e$"; c.Label.Interpreter = 'latex'; c.Label.FontSize = font_size_large;
title("Measured",'FontSize',font_size, 'interpreter','latex')
ylim([-0.5 0.5])

xlabel(t,"$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel(t,"$c_m$",'interpreter','latex','FontSize',font_size_large)
title(t,"Pitch Moment Coefficient",'FontSize',font_size_large, 'interpreter','latex')


%% Theoretical validation
% Convert coefficients to absolute values
airframe_static_properties;
AR = aspect_ratio;

c_L_alpha_approx = pi * AR / (1 + sqrt(1 + (AR / 2)^2))
c_L_alpha * 180 / pi

c_L_0 = c_L_0 - c_L_alpha * alpha_nom; % Find absolute c_L_0
c_L_alpha = c_L_alpha;

c_D_0 = c_D_0 - c_D_alpha * alpha_nom + c_D_alpha_sq * alpha_nom^2;
c_D_alpha = c_D_alpha - 2 * c_D_alpha_sq * alpha_nom;
c_D_alpha_sq = c_D_alpha_sq;

c_m_0 = c_m_0 - c_m_alpha * alpha_nom; % Find absolute c_m_0
c_m_alpha = c_m_alpha;



% %Crude line search on e
% e = 0.8:0.01:1.5;
% c_D_0_theoretical = c_D_0 + c_L_0 ^ 2 ./ (pi * e * AR);
% c_D_alpha_theoretical = 2 * c_L_0 *c_L_alpha ./ (pi * e * AR);
% c_D_alpha_sq_theoretical = c_L_alpha^2 ./ (pi * e * AR);
% 
% cost = abs(c_D_0 - c_D_0_theoretical) ./ c_D_0 ...
%     + abs(c_D_alpha - c_D_alpha_theoretical) ./ c_D_alpha ...
%     + abs(c_D_alpha_sq - c_D_alpha_sq_theoretical) ./ c_D_alpha_sq;
% plot(e,cost)

% Choose e = 1 as this gives lowest cost
e = 0.8;
c_D_0_theoretical = c_D_0 + c_L_0 ^ 2 ./ (pi * e * AR);
c_D_alpha_theoretical = 2 * c_L_0 * c_L_alpha ./ (pi * e * AR);
c_D_alpha_sq_theoretical = c_L_alpha^2 ./ (pi * e * AR);

% compare curves
alpha = -15:20;
c_D = c_D_0 + c_D_alpha .* alpha + c_D_alpha_sq .* alpha.^2;
c_D_theoretical = c_D_0_theoretical + c_D_alpha_theoretical .* alpha + c_D_alpha_sq_theoretical .* alpha.^2;

figure
plot_settings
plot(alpha, c_D, alpha, c_D_theoretical);
legend(["Model drag" "Theoretical drag"], 'interpreter', 'latex', 'FontSize', font_size_small)
set(gca,'FontSize', font_size_small)
xlabel("$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel("$c_D$",'interpreter','latex','FontSize',font_size_large)
title("Comparing Model Drag with Theoretial Drag",'FontSize',font_size_large, 'interpreter','latex')
ylim([0 0.7])
% figure
% t = tiledlayout(1,3);
% nexttile
% bar(1,[c_D_0; c_D_0_theoretical]);
% 
% nexttile
% bar(1,[c_D_alpha; c_D_alpha_theoretical]);
% 
% nexttile
% bar(1,[c_D_alpha_sq; c_D_alpha_sq_theoretical]);


%%


function c_D = calc_drag_coeff(c_D_0, c_D_alpha, c_D_alpha_sq, c_D_delta_e, c_D_alpha_delta_e, alpha, delta_e, alpha_nom, delta_e_nom)
    c_D = c_D_0 + c_D_alpha .* (alpha - alpha_nom) + c_D_alpha_sq .* (alpha - alpha_nom).^2 ...
    + c_D_delta_e .* (delta_e - delta_e_nom) + c_D_alpha_delta_e .* (alpha - alpha_nom)  .* (delta_e - delta_e_nom);
end

function c_L = calc_lift_coeff(c_L_0, c_L_alpha, c_L_delta_e, alpha, delta_e, alpha_nom, delta_e_nom)
    c_L = c_L_0 + c_L_alpha .* (alpha - alpha_nom) + c_L_delta_e .* (delta_e - delta_e_nom);
end

function c_m = calc_moment_coeff(c_m_0, c_m_alpha, c_m_delta_e, alpha, delta_e, alpha_nom, delta_e_nom)
    c_m = c_m_0 + c_m_alpha .* (alpha - alpha_nom) + c_m_delta_e .* (delta_e - delta_e_nom);
end