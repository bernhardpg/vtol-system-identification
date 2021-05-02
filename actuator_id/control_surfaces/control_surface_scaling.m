clc; clear all; close all;

dt = 1/200; % Frame rate

kDEG_OFFSET = 90;

%%
% Ailerons
data_folder = "data/sweep/";
experiment_names = ["aileron_down.csv" "aileron_up.csv"];
log_names = ["aileron_down_actuator_controls_1_0.csv" "aileron_up_actuator_controls_1_0.csv"];

total_fused_input = [];
total_angle_deg = [];
for i = 1:2
    % Read data
    filename = experiment_names(i);
    log_name = log_names(i);
    
    data = readtable(data_folder + filename);
    all_inputs = readtable(data_folder + log_name);
    
    % Read angle data
    frame_number = data.Frame_number;
    angle_deg_raw = data.Angle_deg_;
       
    % Offset angles
    angle_deg = angle_deg_raw - 90;

    % Process data into usable formats
    t_angle_data = frame_number * dt;
  
    % Read input
    input_time_raw = all_inputs.timestamp;
    input_raw = all_inputs.control_0_;
    
    % Synchronize input 
    for j = 1:length(input_raw)
       eps = 1e-4;
       if (abs(input_raw(j) - input_raw(1)) > eps)
          input_start_index = j;
          break;
       end
    end
    input = input_raw(input_start_index:end);
    input_time_w_offset_ms = input_time_raw(input_start_index:end);
    t_input = (input_time_w_offset_ms - input_time_w_offset_ms(1)) / 1e6;

    % Fuse input to match angle data
    N = length(t_angle_data);
    input_fused = interp1q(t_input, input, t_angle_data);
    
    total_fused_input = [total_fused_input;
                         input_fused];
    total_angle_deg = [total_angle_deg;
                         angle_deg];
end

total_fused_input = sort(total_fused_input);
total_angle_deg = sort(total_angle_deg);

% Plot angle as a function of input
figure
plot(total_fused_input, total_angle_deg); hold on;
ylabel("angle [deg]");
xlabel("input [-1,1]");

% LSE
phi = [total_fused_input';
       ones(1,length(total_fused_input))];

y = total_angle_deg;
P = (phi * phi')^-1;
B = phi * y;
theta = P*B;

% RMSE
y_hat = phi' * theta;
RMSE = sqrt(mean((y_hat - y).^2));

% Test LSE
input_test = min(total_fused_input):0.01:max(total_fused_input);
angle_deg_est = theta(1) * input_test + theta(2); % Linear approximation
plot(input_test, angle_deg_est);
legend("Data","Estimated: y = " + theta + "x")
title(filename + ", RMSE: " + RMSE);

disp("Aileron:")
disp("y = " + theta(1) + "x + " + theta(2));
aileron_input_to_aileron_deg = theta(1);


%%
% Elevator
data_folder = "data/sweep/";
experiment_names = ["elevator_down.csv" "elevator_up.csv"];
log_names = ["elevator_down_actuator_controls_1_0.csv" "elevator_up_actuator_controls_1_0.csv"];

total_fused_input = [];
total_angle_deg = [];
for i = 1:2
    % Read data
    filename = experiment_names(i);
    log_name = log_names(i);
    
    data = readtable(data_folder + filename);
    all_inputs = readtable(data_folder + log_name);
    
    % Read angle data
    frame_number = data.Frame_number;
    angle_deg_raw = data.Angle_deg_;
       
    % Offset angles
    angle_deg = angle_deg_raw - 90;

    % Process data into usable formats
    t_angle_data = frame_number * dt;
    

  
    % Read input
    input_time_raw = all_inputs.timestamp;
    input_raw = all_inputs.control_1_;
    
    % Synchronize input 
    for j = 1:length(input_raw)
       eps = 1e-4;
       if (abs(input_raw(j) - input_raw(1)) > eps)
          input_start_index = j;
          break;
       end
    end
    input = input_raw(input_start_index:end);
    input_time_w_offset_ms = input_time_raw(input_start_index:end);
    t_input = (input_time_w_offset_ms - input_time_w_offset_ms(1)) / 1e6;
    
    
    
    % Fuse input to match angle data
    N = length(t_angle_data);
    input_fused = interp1q(t_input, input, t_angle_data);
    
    total_fused_input = [total_fused_input;
                         input_fused];
    total_angle_deg = [total_angle_deg;
                         angle_deg];
end

total_fused_input = sort(total_fused_input);
total_angle_deg = sort(total_angle_deg);

% Plot angle as a function of input
figure
plot(total_fused_input, total_angle_deg); hold on;
ylabel("angle [deg]");
xlabel("input [-1,1]");

% LSE
phi = [total_fused_input';
       ones(1,length(total_fused_input))];

y = total_angle_deg;
P = (phi * phi')^-1;
B = phi * y;
theta = P*B;

% RMSE
y_hat = phi' * theta;
RMSE = sqrt(mean((y_hat - y).^2));

% Test LSE
input_test = min(total_fused_input):0.01:max(total_fused_input);
angle_deg_est = theta(1) * input_test + theta(2); % Linear approximation
plot(input_test, angle_deg_est);
legend("Data","Estimated: y = " + theta + "x")
title(filename + ", RMSE: " + RMSE);

disp("Tail deflection from elevator input:")
disp("y = " + theta(1) + "x + " + theta(2));
elevator_input_to_tail_deg = theta(1);


%% Extract actual values to model
mixer_elevator_to_tail = 0.8; % From PX4 mixer file
mixer_rudder_to_tail = 0.7; % From PX4 mixer file
rudder_input_to_tail_deg = elevator_input_to_tail_deg / mixer_elevator_to_tail * mixer_rudder_to_tail;

aileron_input_to_aileron_deg
elevator_input_to_tail_deg
rudder_input_to_tail_deg