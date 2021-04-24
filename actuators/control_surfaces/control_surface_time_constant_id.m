clc; clear all; close all;

dt = 1/200; % Frame rate

kDEG_OFFSET = 90;

data_folder = "data/step/";
experiment_names = [...
    "aileron_down_test.csv" "aileron_down_test_half_amplitude.csv"...
    "aileron_up_test.csv" "aileron_up_test_half_amplitude.csv"...
    "a_tail_up_test.csv" "a_tail_down_test.csv"];

for i = 1:length(experiment_names)
    % Read data
    filename = experiment_names(i)
    data_down = readtable(data_folder + filename);
    frame_number = data_down.Frame_number;
    angle_deg_raw = data_down.Angle_deg_;

    % Offset angles
    angle_deg = abs(angle_deg_raw - 90);
    amplitude = max(angle_deg);
    altitude_at_time_constant = amplitude * 0.632;

    % Process data into usable formats
    t = frame_number * dt;
    % Find time constant
    time_constant = interp1q(angle_deg, t, altitude_at_time_constant);
    max_deflection = max(angle_deg);
    disp(filename)
    disp("time constant: " + time_constant);
    disp("max deflection: " + max_deflection);
    
    figure
    plot(t, angle_deg); hold on;
    plot(t, ones(size(t)) * altitude_at_time_constant); hold on;
    xline(time_constant);
    ylabel("angle [deg]");
    xlabel("time [s]");
    title(filename + " Tc = " + time_constant);
end