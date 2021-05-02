clc; clear all; close all;

% Load data
datapaths = ["data/static_stability/experiment_1/" "data/static_stability/experiment_2/"];
num_experiments = length(datapaths);
state = [];
input = [];
c_L = [];
c_D = [];
AoA_rad = [];
dt = [];

for i = 1:num_experiments
    datapath = datapaths(i);
    state_exp = readmatrix(datapath + "state.csv");
    input_exp = readmatrix(datapath + "input.csv");
    c_L_exp = readmatrix(datapath + "cl.csv");
    c_D_exp = readmatrix(datapath + "cd.csv");
    AoA_rad_exp = readmatrix(datapath + "aoa_rad.csv");
    maneuver_start_indices_exp = readmatrix(datapath + "maneuver_start_indices.csv");
    AoA_deg_exp = AoA_rad .* (180 / pi);
    dt_exp = readmatrix(datapath + "dt.csv");
    
    state = [state;
             state_exp];
    input = [input;
             input_exp];
    c_L = [c_L;
           c_L_exp];
    c_D = [c_D;
           c_D_exp];
    AoA_rad = [AoA_rad;
               AoA_rad_exp];
    dt = [dt;
           dt_exp];
    
end

AoA_deg = rad2deg(AoA_rad);

q = state(:,6);
delta_e = input(:,6);

% Model parameters
aircraft_properties;

% Scatter plot of all maneuvers

scatter_lift_drag(AoA_rad, q, c_L, c_D);
plot_3d_lift_drag(AoA_rad, q, c_L, c_D);


%sgtitle(filename);
%filename = "Scatter plot with " + length(aggregated_maneuvers) + " maneuvers";
%saveas(fig, 'static_curves/data/maneuver_plots/' + filename, 'epsc')


%% Curve fitting
N = length(AoA_rad);

% c_L = c_L0 + c_Lalpha * alpha + c_Lq * q + c_Ldeltae + delta_e
Phi = [ones(1,N);
       AoA_rad';
       q'];
Y = c_L;

theta = least_squares_est(Phi, Y);
c_L0 = theta(1);
c_Lalpha = theta(2);
c_Lq = theta(3);
%c_Ldeltae = theta(4);
    
% Quadratic drag
% c_D = c_Dp + c_Dalpha * alpha^2 + c_Dq * q + c_Ddeltae + delta_e
Phi = [ones(1,N);
       AoA_rad'.^2;
       q'];
      % delta_e'];
Y = c_D;
theta = least_squares_est(Phi, Y);

c_Dp = theta(1);
c_Dalpha = theta(2);
c_Dq = theta(3);
%c_Ddeltae = theta(4);

AoA_test_deg = 0:0.5:11;
AoA_test_rad = AoA_test_deg .* (pi / 180);
q_test = 0:0.01:0.5;
%delta_e_test = 0:0.01:0.5;
[X,Y] = meshgrid(AoA_test_rad, q_test);
%surf(X(:,:,1),Y(:,:,1),c_L_estimated(:,:,1));

c_L_estimated = c_L0 + c_Lalpha .* X + ...
    c_Lq .* Y;
c_D_estimated = c_Dp + c_Dalpha * X .^2 + ...
    c_Dq .* Y;

fig = figure;
fig.Position = [100 100 1000 300];
subplot(1,2,1)
surf(X,Y,c_L_estimated); hold on; alpha(0.5);
scatter3(AoA_rad, q, c_L); alpha(0.5);
xlabel("AoA")
ylabel("q")
zlabel("c_L")
zlim([0 1.15])


subplot(1,2,2)
surf(X,Y,c_D_estimated); hold on; alpha(0.5);
scatter3(AoA_rad, q, c_D); alpha(0.5);
xlabel("AoA")
ylabel("q")
zlabel("c_D")
zlim([0 0.2])


% Calculate fit
c_L_estimated = c_L0 + c_Lalpha .* AoA_rad + ...
    c_Lq .* q;
c_D_estimated = c_Dp + c_Dalpha * AoA_rad .^2 + ...
    c_Dq .* q;

residuals_c_L = abs((c_L - c_L_estimated) ./ c_L);
fit_c_L = 1 - mean(residuals_c_L);
residuals_c_D = abs((c_D - c_D_estimated) ./ c_D);
fit_c_D = 1 - mean(residuals_c_D);



% disp("Linear lift model");
% disp("c_L = c_L0 + c_Lalpha * AoA");
% disp("c_L0 = " + c_L0);
% disp("c_Lalpha = " + c_Lalpha);
% disp("Quadratic drag model");
% disp("c_D = c_Dp + c_Dalpha * AoA.^2");
% disp("c_Dp = " + c_Dp);
% disp("c_Dalpha = " + c_Dalpha);
% disp(" ");

%sgtitle(filename);
%filename = "Scatter plot with " + length(aggregated_maneuvers) + " maneuvers (unfiltered)";
%saveas(fig, 'static_curves/data/maneuver_plots/' + filename, 'epsc')

%%

function [] = scatter_lift_drag(AoA_rad, q, c_L, c_D)
    fig = figure;
    fig.Position = [100 100 1000 300];
    subplot(1,2,1)
    scatter(AoA_rad * 180 / pi, c_L, [], q); alpha(0.3);
    c = colorbar;
    c.Label.String = 'q [rad/s]';
    xlabel("AoA")
    ylabel("c_L")
    ylim([0 1.15])

    subplot(1,2,2)
    scatter(AoA_rad * 180 / pi, c_D, [], q); alpha(0.3);
    c = colorbar;
    c.Label.String = 'q [rad/s]';
    xlabel("AoA")
    ylabel("c_D")
    ylim([0 0.2])
end

function [] = plot_3d_lift_drag(AoA_rad, q, c_L, c_D)
    fig = figure;
    fig.Position = [100 100 1000 300];
    subplot(1,2,1)
    scatter3(AoA_rad, q, c_L); alpha(0.3);
    xlabel("AoA")
    ylabel("q")
    zlabel("c_L")
    zlim([0 1.15])

    subplot(1,2,2)
    scatter3(AoA_rad, q, c_D); alpha(0.3);
    xlabel("AoA")
    ylabel("q")
    zlabel("c_D")
    zlim([0 0.2])
end

function [theta] = least_squares_est(Phi, Y)
    P = inv(Phi * Phi');
    B = Phi * Y;

    theta = P * B;
end

function [sigma] = blending_function(alpha_stall_rad, M, alpha_rad)
    num = 1 + exp(-M * (alpha_rad - alpha_stall_rad)) + exp(M * (alpha_rad + alpha_stall_rad));
    den = (1 + exp(-M * (alpha_rad - alpha_stall_rad))) .* (1 + exp(M * (alpha_rad + alpha_stall_rad)));
    sigma = num ./ den;
end

function [c_L_linear] = lift_linear(c_L0, c_Lalpha, alpha_rad)
    c_L_linear = c_L0 + c_Lalpha * alpha_rad;
end

function [c_L_flatplate] = lift_flat_plate(alpha_rad)
    c_L_flatplate = 2 .* sign(alpha_rad) .* sin(alpha_rad).^2 .* cos(alpha_rad);
end
