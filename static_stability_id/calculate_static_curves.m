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
% c_Lalpha guess
c_Lalpha_guess = pi * aspect_ratio / (1 + sqrt(1 + (aspect_ratio/2)^2));
disp("c_Lalpha_guess = " + c_Lalpha_guess);
disp(" ");

N = length(AoA_rad);

% Linear lift
Phi = [ones(1,N);
       AoA_rad'];
Y = c_L;

P = inv(Phi * Phi');
B = Phi * Y;

theta = P * B;
c_L0 = theta(1);
c_Lalpha = theta(2);
    
% Quadratic drag
Phi = [ones(1,N);
       AoA_rad'.^2];
P = inv(Phi * Phi');
Y = c_D;
B = Phi * Y;
theta = P * B;
c_Dp = theta(1);
c_Dalpha = theta(2);

% Using Oswald's efficiency factor
% Phi = [ones(1,N);
%        (c_L0 + c_Lalpha * AoA_deg)'.^2 / (AR * pi)];
% P = inv(Phi * Phi');
% Y = c_D;
% B = Phi * Y;
% theta = P * B;
% c_Dp = theta(1);
% e = 1/theta(2); % Oswalds efficiency factor

AoA_test_deg = 2:0.01:11;
AoA_test_rad = AoA_test_deg .* (pi / 180);
c_L_estimated = c_L0 + c_Lalpha * AoA_test_rad;
c_D_estimated = c_Dp + c_Dalpha * AoA_test_rad .^2;

fig = figure;
fig.Position = [100 100 1000 300];
subplot(1,2,1)
plot(AoA_test_deg, c_L_estimated); hold on
scatter(AoA_deg, c_L);
xlabel("AoA")
ylabel("c_L")

subplot(1,2,2)
plot(AoA_test_deg, c_D_estimated); hold on
scatter(AoA_deg, c_D);
xlabel("AoA")
ylabel("c_D")

disp("Linear lift model");
disp("c_L = c_L0 + c_Lalpha * AoA");
disp("c_L0 = " + c_L0);
disp("c_Lalpha = " + c_Lalpha);
disp("Quadratic drag model");
disp("c_D = c_Dp + c_Dalpha * AoA.^2");
disp("c_Dp = " + c_Dp);
disp("c_Dalpha = " + c_Dalpha);
disp(" ");

%sgtitle(filename);
%filename = "Scatter plot with " + length(aggregated_maneuvers) + " maneuvers (unfiltered)";
%saveas(fig, 'static_curves/data/maneuver_plots/' + filename, 'epsc')


%%% Flat plate model fit
% Guesstimate these
alpha_stall = 14 / 180 * pi;
M = 0.001;

sigma = blending_function(alpha_stall, M, AoA_rad);
Phi = [(1 - sigma)';
       (1 - sigma)' .* AoA_rad'];
Y = c_L - sigma .* lift_flat_plate(AoA_rad);
P = inv(Phi * Phi');
B = Phi * Y;

theta = P * B;
c_L0 = theta(1);
c_Lalpha = theta(2);
    

% Flat plate model

% Test blending function
%AoA_test = 0:0.01:12;
%plot(AoA_test, blending_function(AoA_test));
%plot(AoA_test, flat_plate(AoA_test / 180 * pi));
sigma = blending_function(alpha_stall, M, AoA_test_rad);

c_L_estimated = (1 - sigma) .* lift_linear(c_L0, c_Lalpha, AoA_test_rad) + sigma .* lift_flat_plate(AoA_test_rad);

fig = figure;
fig.Position = [100 100 1000 300];
subplot(1,2,1)
plot(AoA_test_deg, c_L_estimated); hold on
scatter(AoA_deg, c_L);
xlabel("AoA")
ylabel("c_L")

subplot(1,2,2)
plot(AoA_test_deg, c_D_estimated); hold on
scatter(AoA_deg, c_D);
xlabel("AoA")
ylabel("c_D")

disp("")
disp("Flat plate model")
disp("Chosen values: alpha_stall = " + alpha_stall + ", M = " + M);
disp("c_L = (1 - sigma) * (c_L0 + c_Lalpha * AoA) + (sigma) * c_L_flat_plate(AoA)");
disp("c_L0 = " + c_L0);
disp("c_Lalpha = " + c_Lalpha);
disp("c_D = c_Dp + c_Dalpha * AoA.^2");
disp("c_Dp = " + c_Dp);
disp("c_Dalpha = " + c_Dalpha);

% filename = "Scatter plot with " + length(aggregated_maneuvers) + " maneuvers";
% sgtitle(filename);
% filename = "Scatter plot with " + length(aggregated_maneuvers) + " maneuvers (unfiltered) flat plate";
% saveas(fig, 'static_curves/data/maneuver_plots/' + filename, 'epsc')


%%

function [] = scatter_lift_drag(AoA_rad, q, c_L, c_D)
    fig = figure;
    fig.Position = [100 100 1000 300];
    subplot(1,2,1)
    scatter(AoA_rad * 180 / pi, c_L, [], q); alpha(0.5);
    c = colorbar;
    c.Label.String = 'q [rad/s]';
    xlabel("AoA")
    ylabel("c_L")
    ylim([0 1.15])

    subplot(1,2,2)
    scatter(AoA_rad * 180 / pi, c_D, [], q); alpha(0.5);
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
    scatter3(AoA_rad, q, c_L);
    xlabel("AoA")
    ylabel("q")
    zlabel("c_L")
    zlim([0 1.15])

    subplot(1,2,2)
    scatter3(AoA_rad, q, c_D);
    xlabel("AoA")
    ylabel("q")
    zlabel("c_D")
    zlim([0 0.2])
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
