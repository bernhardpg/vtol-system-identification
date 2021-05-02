clc; clear all; close all;

% Load data
input_output_data = readtable('static_curves/data/output.csv');
c_L = readmatrix('static_curves/data/c_L.csv');
c_D = readmatrix('static_curves/data/c_D.csv');
AoA_rad = readmatrix('static_curves/data/AoA_rad.csv');
AoA_deg = AoA_rad .* (180 / pi);
maneuver_length = readmatrix('static_curves/data/maneuver_length.csv');
dt = readmatrix('static_curves/data/dt.csv');
aggregated_maneuvers = readmatrix('static_curves/data/aggregated_maneuvers.csv');
num_maneuvers = length(AoA_rad) / maneuver_length;

q = input_output_data.state_output_6;
delta_e = input_output_data.input_output_6;
    

% Model parameters
aircraft_properties;

% Scatter plot of all maneuvers
fig = figure;
fig.Position = [100 100 1000 300];
subplot(1,2,1)
scatter(AoA_rad * 180 / pi, c_L);
xlabel("AoA")
ylabel("c_L")
ylim([0 1.15])

subplot(1,2,2)
scatter(AoA_rad * 180 / pi, c_D);
xlabel("AoA")
ylabel("c_D")
ylim([0 0.2])

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


%% 3D plots
if 0
    for i = 1:num_maneuvers
        start_index = 1 + maneuver_length * (i - 1);
        end_index = start_index + maneuver_length - 1;
        fig = figure;
        %fig.Visible = 'off';
        fig.Position = [100 100 1000 300];
        subplot(1,2,1)
        scatter3(AoA_rad(start_index:end_index), u_fw(start_index:end_index,2), c_L(start_index:end_index));
        xlabel("AoA")
        ylabel("delta e")
        zlabel("c_L")

        subplot(1,2,2)
        scatter3(AoA_rad(start_index:end_index), u_fw(start_index:end_index,2), c_D(start_index:end_index));
        xlabel("AoA")
        ylabel("delta e")
        zlabel("c_D")

        sgtitle("maneuver no " + aggregated_maneuvers(i));
    end
end

if 1
    fig = figure;
    fig.Position = [100 100 1000 300];
    subplot(1,2,1)
    scatter3(AoA_rad, delta_e, c_L);
    xlabel("AoA")
    ylabel("q")
    zlabel("c_L")

    subplot(1,2,2)
    scatter3(AoA_rad, delta_e, c_D);
    xlabel("AoA")
    ylabel("q")
    zlabel("c_D")
end



%%
%%%% OLD
%% Plot static curves
indices_before_maneuver = 3.5 / dt;
indices_after_maneuver_start = 0 / dt;

% 5: good 6, 2
% 5: good -12, 16. Some SD card error. However, the thrust is here.
% 6: useless
% 7: useless, data dropout
% 8: useless, strange data. Lift goes down with increasing AoA
% 9 is useless
% 10: good 4, 1
% 10: good, -2 5

for i = 2:2
    start_index = static_sysid_indices(i) - indices_before_maneuver;
    maneuver_end_index = static_sysid_indices(i) + indices_after_maneuver_start;
    maneuver_length_in_indices = maneuver_end_index - start_index + 1;
    t_maneuver = t(start_index:maneuver_end_index);
    
    % Lift coefficient
    % Construct regressors
    Phi = [ones(maneuver_length_in_indices,1) AoA(start_index:maneuver_end_index)]';
    % Least Squares Estimation
    P = (Phi * Phi')^-1;
    theta = P * Phi * c_L(start_index:maneuver_end_index);
    % Extract coefficients
    c_L_0 = theta(1);
    c_L_alpha = theta(2);
    
    c_L_hat = c_L_0 + c_L_alpha * AoA(start_index:maneuver_end_index);
    
    % Drag coefficient
    % LSE
    Phi = [ones(maneuver_length_in_indices,1) c_L_hat.^2 / (AR * pi)]';
    P = (Phi * Phi')^-1;
    theta = P * Phi * c_D(start_index:maneuver_end_index);
    c_D_p = theta(1);
    e = 1/theta(2); % Oswalds efficiency factor
    
    c_D_hat = c_D_p + c_L_hat.^2 / (pi * e * AR);
   
    % Plotting
    figure
    subplot(7,1,1);
    plot(t_maneuver, rad2deg(eul(start_index:maneuver_end_index,2:3)));
    legend('pitch','roll');
    title("attitude")

    subplot(7,1,2);
    plot(t_maneuver, w_B(start_index:maneuver_end_index,:));
    legend('w_x','w_y','w_z');
    title("ang vel body")

    subplot(7,1,3);
    plot(t_maneuver, v_B(start_index:maneuver_end_index,:));
    legend('v_x', 'v_y', 'v_z');
    title("vel body")

    subplot(7,1,4);
    plot(t_maneuver, u_fw(start_index:maneuver_end_index,:));
    legend('delta_a','delta_e','delta_r', 'T_fw');
    title("inputs")

    subplot(7,1,5);
    plot(t_maneuver, AoA(start_index:maneuver_end_index));
    title("Angle of Attack")

    subplot(7,1,6);
    plot(t_maneuver, L(start_index:maneuver_end_index));
    legend('L');
    title("Lift force")
    
    subplot(7,1,7);
    plot(t_maneuver, D(start_index:maneuver_end_index));
    legend('D');
    title("Drag force")
    
    figure
    subplot(2,1,1);
    plot(AoA(start_index:maneuver_end_index), c_L_hat); hold on;
    scatter(AoA(start_index:maneuver_end_index), c_L(start_index:maneuver_end_index)); hold on;
    xlabel("AoA")
    ylabel("c_L")
    ylim([0 max(c_L(start_index:maneuver_end_index))*1.2])
    
    subplot(2,1,2);
    plot(AoA(start_index:maneuver_end_index), c_D_hat); hold on;
    scatter(AoA(start_index:maneuver_end_index), c_D(start_index:maneuver_end_index)); hold on;
    xlabel("AoA")
    ylabel("c_D")
    ylim([min([0 c_D(start_index:maneuver_end_index)']) max(c_D(start_index:maneuver_end_index))*1.2])
end
%%

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
