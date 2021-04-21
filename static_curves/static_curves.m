clc; clear all; close all;

% File locations
data_location = "static_curves/data/output.csv";

maneuver_length = readmatrix('static_curves/data/maneuver_length.csv');
dt = readmatrix('static_curves/data/dt.csv');
aggregated_maneuvers = readmatrix('static_curves/data/aggregated_maneuvers.csv');

% Load data
data = readtable(data_location);

%% Read estimated data
state = [data.state_1 data.state_2 data.state_3 data.state_4 ...
         data.state_5 data.state_6 data.state_7 ...
         data.state_8 data.state_9 data.state_10];
     
num_maneuvers = length(state) / maneuver_length;
     
input = [data.input_1 data.input_2 data.input_3 data.input_4 ...
         data.input_5 data.input_6 data.input_7 data.input_8];

u_mr = input(:,1:4);
u_fw = input(:,5:8);
     
N = length(state);

q_NB = state(:,1:4);
v_B = state(:,8:10);

u_mr = [data.input_5 data.input_6 data.input_7 data.input_8];
u_fw = [data.input_5 data.input_6 data.input_7 data.input_8];

%% Read raw acceleration data
 
acc_B = [data.accelerations_1 data.accelerations_2 data.accelerations_3];


%% Model parameters

% TODO
m = 5.6+3*1.27+2*0.951; % kg 
g = 9.81; % m/s^2

% b = 2.5;
% S = 0.75; % TODO: not exact
% AR = b^2 / S;

%% TODO
% Calculate total airspeed

V = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);

% Calculate Angle of Attack

AoA_rad = atan2(v_B(:,3),v_B(:,1));
AoA_deg = rad2deg(AoA_rad);

%% Rotate data
% Create rotation matrices
q_BN = quatinv(q_NB);
R_BN = quat2rotm(q_BN);

eul_rad = [zeros(size(AoA_rad)) AoA_rad zeros(size(AoA_rad))];
R_SB = eul2rotm(eul_rad); % For some reason this gives a left-handed rotation

% Calculate gravitational force in Stability frame
Fg_N = [0; 0; m*g];
Fg_S = zeros(N,3);

for i = 1:N
   R_SN_at_i = R_SB(:,:,i) .* R_BN(:,:,i);
   Fg_S_at_i = R_SN_at_i * Fg_N;
   Fg_S(i,:) = Fg_S_at_i;
end

% Transform acceleration from body frame to stability frame
acc_S = zeros(size(acc_B));
for i = 1:N
    acc_S_at_i = R_SB(:,:,i) * acc_B(i,:)';
    acc_S(i,:) = acc_S_at_i;
end

%% Extract aerodynamic forces
% figure
% plot(Fg_S(:,[1 3]))
% legend('x','z')
% 
% figure
% plot(m*acc_S(:,[1 3]))
% legend('x','z')

Fa_S = m * acc_S - Fg_S;
D = -Fa_S(:,1);
L = -Fa_S(:,3);

%% Calculate coefficients
rho = 1.225; % kg / m^3
dynamic_pressure = 0.5 * rho * V.^2;
c_L = L ./ dynamic_pressure;
c_D = D ./ dynamic_pressure;

%% Plot
figure
subplot(5,1,1)
plot(L);
legend('Lift')

subplot(5,1,2)
plot(D)
legend('Drag')

subplot(5,1,3)
plot(V)
legend('airspeed')

subplot(5,1,4)
plot(AoA_deg)
legend('AoA')

subplot(5,1,5)
plot(u_fw)
legend('delta_a','delta_e', 'delta_r','delta_T')


%% 
figure
subplot(5,1,1)
plot(c_L);
legend('c_L');

subplot(5,1,2)
plot(c_D)
legend('c_D');

subplot(5,1,3)
plot(V)
legend('airspeed');

subplot(5,1,4)
plot(AoA_deg)
legend('AoA')

subplot(5,1,5)
plot(u_fw(:,1))
legend('delta_a')%,'delta_e', 'delta_r','delta_T')

%%
figure
scatter3(AoA_deg, u_fw(:,2), c_L)
xlabel('AoA')
ylabel('delta e')
zlabel('c_L')

figure
scatter3(AoA_deg, u_fw(:,2), c_D)
xlabel('AoA')
ylabel('delta e')
zlabel('c_D')

%%

% 2D plots
for i = 1:num_maneuvers
    start_index = 1 + maneuver_length * (i - 1);
    end_index = start_index + maneuver_length - 1;
    fig = figure;
    %fig.Visible = 'off';
    fig.Position = [100 100 1000 300];
    subplot(1,2,1)
    scatter(AoA_deg(start_index:end_index), c_L(start_index:end_index));
    xlabel("AoA")
    ylabel("c_L")

    subplot(1,2,2)
    scatter(AoA_deg(start_index:end_index), c_D(start_index:end_index));
    xlabel("AoA")
    ylabel("c_D")
    
    sgtitle("maneuver no " + aggregated_maneuvers(i));
end

% 3D plots
for i = 1:num_maneuvers
    start_index = 1 + maneuver_length * (i - 1);
    end_index = start_index + maneuver_length - 1;
    fig = figure;
    %fig.Visible = 'off';
    fig.Position = [100 100 1000 300];
    subplot(1,2,1)
    scatter3(AoA_deg(start_index:end_index), u_fw(start_index:end_index,2), c_L(start_index:end_index));
    xlabel("AoA")
    ylabel("delta e")
    zlabel("c_L")

    subplot(1,2,2)
    scatter3(AoA_deg(start_index:end_index), u_fw(start_index:end_index,2), c_D(start_index:end_index));
    xlabel("AoA")
    ylabel("delta e")
    zlabel("c_D")
    
    sgtitle("maneuver no " + aggregated_maneuvers(i));
end



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
    phi = [ones(maneuver_length_in_indices,1) AoA(start_index:maneuver_end_index)]';
    % Least Squares Estimation
    P = (phi * phi')^-1;
    theta = P * phi * c_L(start_index:maneuver_end_index);
    % Extract coefficients
    c_L_0 = theta(1);
    c_L_alpha = theta(2);
    
    c_L_hat = c_L_0 + c_L_alpha * AoA(start_index:maneuver_end_index);
    
    % Drag coefficient
    % LSE
    phi = [ones(maneuver_length_in_indices,1) c_L_hat.^2 / (AR * pi)]';
    P = (phi * phi')^-1;
    theta = P * phi * c_D(start_index:maneuver_end_index);
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


