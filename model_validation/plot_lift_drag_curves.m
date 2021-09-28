clc; close all; clear all;

load("model_identification/equation_error/results/equation_error_coeffs_lon.mat");

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

[alphas,delta_es] = meshgrid(alpha, delta_e);

c_L = calc_lift_coeff(c_L_0, c_L_alpha, c_L_delta_e, alphas, delta_es, alpha_nom, delta_e_nom);
c_D = calc_drag_coeff(c_D_0, c_D_alpha, c_D_alpha_sq, c_D_delta_e, c_D_alpha_delta_e, alphas, delta_es, alpha_nom, delta_e_nom);
c_m = calc_moment_coeff(c_m_0, c_m_alpha, c_m_delta_e, alphas, delta_es, alpha_nom, delta_e_nom);

% Plot lift coefficient
figure
plot_settings; % import plot settings
surf(alphas,delta_es,c_L); hold on
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
surf(alphas,delta_es,c_D); hold on
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
surf(alphas,delta_es,c_m); hold on
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

resolution = 0.5;
alpha_min = -10;
alpha_max = 15;
alphas = (alpha_min:resolution:alpha_max);

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
figure
for delta_e = delta_es
    c_D = calc_drag_coeff(c_D_0, c_D_alpha, c_D_alpha_sq, c_D_delta_e, c_D_alpha_delta_e, alphas, delta_e, alpha_nom, delta_e_nom);
    plot(alphas, c_D); hold on;
end
xlabel("$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel("$c_D$",'interpreter','latex','FontSize',font_size_large)
xline(alpha_nom,":")
legend(delta_e_texts,'interpreter','latex','Fontsize',font_size,'location','best')
text(alpha_nom+0.1,-0.3,"$\alpha^*$",'interpreter','latex','Fontsize',font_size)
title("Drag contour lines",'FontSize',font_size_large, 'interpreter','latex')

% Lift curve
figure
for delta_e = delta_es
    c_L = calc_lift_coeff(c_L_0, c_L_alpha, c_L_delta_e, alphas, delta_e, alpha_nom, delta_e_nom);
    plot(alphas, c_L); hold on;
end
xlabel("$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel("$c_L$",'interpreter','latex','FontSize',font_size_large)
xline(alpha_nom,":")
legend(delta_e_texts,'interpreter','latex','Fontsize',font_size,'location','best')
text(alpha_nom+0.1,-0.3,"$\alpha^*$",'interpreter','latex','Fontsize',font_size)
title("Lift contour lines",'FontSize',font_size_large, 'interpreter','latex')

% Pitch moment curve
figure
for i = 1:length(delta_es)
    delta_e = delta_es(i);
    c_m = calc_moment_coeff(c_m_0, c_m_alpha, c_m_delta_e, alphas, delta_e, alpha_nom, delta_e_nom);
    plot(alphas, c_m+0.07); hold on;
end
xlabel("$\alpha [^\circ]$",'interpreter','latex','FontSize',font_size)
ylabel("$c_m$",'interpreter','latex','FontSize',font_size_large)
xline(alpha_nom,":")
legend(delta_e_texts,'interpreter','latex','Fontsize',font_size,'location','best')
text(alpha_nom+0.1,-0.3,"$\alpha^*$",'interpreter','latex','Fontsize',font_size)
title("Pitch moment contour lines",'FontSize',font_size_large, 'interpreter','latex')

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