clc; clear all; close all;

% File locations
data_location = "static_curves/data/output.csv";

kLENGTH_MANEUVER = 391; % Chose this when importing the data

% Load data
data = readtable(data_location);

state = [data.state_1 data.state_2 data.state_3 data.state_4 ...
         data.state_5 data.state_6 data.state_7 ...
         data.state_8 data.state_9 data.state_10];
     
v_B = state(:,8:10);

u_mr = [data.input_5 data.input_6 data.input_7 data.input_8];
u_fw = [data.input_5 data.input_6 data.input_7 data.input_8];

a_x = data.accelerations_1;
a_y = data.accelerations_2;
a_z = data.accelerations_3;

%% STATIC CURVES
%%%%%%%%

%% Model parameters

% TODO fix these
m = 5.6+3*1.27+2*0.951; % kg 

g = 9.81; % m/s^2
rho = 1.225; % kg / m^3
b = 2.5;
S = 0.75; % TODO: not exact
AR = b^2 / S;

%% TODO
% Calculate total airspeed

V = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);

% Calculate Angle of Attack

AoA = rad2deg(atan2(v_B(:,3),v_B(:,1)));

%% Extract aerodynamic forces

F_x = a_x / m;
F_z = a_y / m;

N = length(state);

L = zeros(N,1);
D = zeros(N,1);

for i = 1:N
   alpha = deg2rad(AoA(i));
   R_BS = [cos(alpha) -sin(alpha);
           sin(alpha) cos(alpha)];
   R_SB = inv(R_BS);
   F_B = [F_x(i); F_z(i)]; % Body frame forces
   F_S = R_SB * F_B; % Rotated to stability frame
   D(i) = -F_S(1);
   L(i) = -F_S(2);
end

dynamic_pressure = 0.5 * rho * V.^2;
c_L = L ./ dynamic_pressure;
c_D = D ./ dynamic_pressure;

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
    maneuver_start_index = static_sysid_indices(i) - indices_before_maneuver;
    maneuver_end_index = static_sysid_indices(i) + indices_after_maneuver_start;
    maneuver_length_in_indices = maneuver_end_index - maneuver_start_index + 1;
    t_maneuver = t(maneuver_start_index:maneuver_end_index);
    
    % Lift coefficient
    % Construct regressors
    phi = [ones(maneuver_length_in_indices,1) AoA(maneuver_start_index:maneuver_end_index)]';
    % Least Squares Estimation
    P = (phi * phi')^-1;
    theta = P * phi * c_L(maneuver_start_index:maneuver_end_index);
    % Extract coefficients
    c_L_0 = theta(1);
    c_L_alpha = theta(2);
    
    c_L_hat = c_L_0 + c_L_alpha * AoA(maneuver_start_index:maneuver_end_index);
    
    % Drag coefficient
    % LSE
    phi = [ones(maneuver_length_in_indices,1) c_L_hat.^2 / (AR * pi)]';
    P = (phi * phi')^-1;
    theta = P * phi * c_D(maneuver_start_index:maneuver_end_index);
    c_D_p = theta(1);
    e = 1/theta(2); % Oswalds efficiency factor
    
    c_D_hat = c_D_p + c_L_hat.^2 / (pi * e * AR);
   
    % Plotting
    figure
    subplot(7,1,1);
    plot(t_maneuver, rad2deg(eul(maneuver_start_index:maneuver_end_index,2:3)));
    legend('pitch','roll');
    title("attitude")

    subplot(7,1,2);
    plot(t_maneuver, w_B(maneuver_start_index:maneuver_end_index,:));
    legend('w_x','w_y','w_z');
    title("ang vel body")

    subplot(7,1,3);
    plot(t_maneuver, v_B(maneuver_start_index:maneuver_end_index,:));
    legend('v_x', 'v_y', 'v_z');
    title("vel body")

    subplot(7,1,4);
    plot(t_maneuver, u_fw(maneuver_start_index:maneuver_end_index,:));
    legend('delta_a','delta_e','delta_r', 'T_fw');
    title("inputs")

    subplot(7,1,5);
    plot(t_maneuver, AoA(maneuver_start_index:maneuver_end_index));
    title("Angle of Attack")

    subplot(7,1,6);
    plot(t_maneuver, L(maneuver_start_index:maneuver_end_index));
    legend('L');
    title("Lift force")
    
    subplot(7,1,7);
    plot(t_maneuver, D(maneuver_start_index:maneuver_end_index));
    legend('D');
    title("Drag force")
    
    figure
    subplot(2,1,1);
    plot(AoA(maneuver_start_index:maneuver_end_index), c_L_hat); hold on;
    scatter(AoA(maneuver_start_index:maneuver_end_index), c_L(maneuver_start_index:maneuver_end_index)); hold on;
    xlabel("AoA")
    ylabel("c_L")
    ylim([0 max(c_L(maneuver_start_index:maneuver_end_index))*1.2])
    
    subplot(2,1,2);
    plot(AoA(maneuver_start_index:maneuver_end_index), c_D_hat); hold on;
    scatter(AoA(maneuver_start_index:maneuver_end_index), c_D(maneuver_start_index:maneuver_end_index)); hold on;
    xlabel("AoA")
    ylabel("c_D")
    ylim([min([0 c_D(maneuver_start_index:maneuver_end_index)']) max(c_D(maneuver_start_index:maneuver_end_index))*1.2])
end

