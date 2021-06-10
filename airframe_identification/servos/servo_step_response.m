clc; clear all; close all;

kDEG_OFFSET = 90;

time_scaling = 8; % recorded at 240 fps, played back at 30 fps.

data_folder = "/data/servo_tests/";
experiment_names = ["aileron_step_up.csv", "aileron_half_step_up.csv","elevator_step_up.csv"];
titles = [...
    "Aileron (Amplitude = 1.0)", ...
    "Aileron (Amplitude = 0.5)", ...
    "Elevator (Amplitude = 1.0)", ...
    ];

fig = figure;
fig.Position = [100 100 1200 300];

for i = 1:length(experiment_names)
    % Read data
    filename = experiment_names(i);
    data_down = readtable(data_folder + filename);
    time_ms = data_down.time_ms;
    angle_deg = data_down.angle_deg;

    % Offset angles
    angle_deg_abs = angle_deg - 90;

    % Process data into usable formats
    t = time_ms / 1e3 / time_scaling;
    
    subplot(1,3,i)
    plot(t, angle_deg_abs, '-o'); hold on;
    ylabel("angle [deg]");
    xlabel("time [s]");
    title(titles(i));
end

% Find time constant and rate limit heuristically

% Simulate system
u_steps = [25 14 24.5];

for i = 1:length(u_steps)
    u_step = u_steps(i);

    delta_a_0 = 0;
    tspan = [0 0.5];
    T = 0.028; % Found by comparing with experiments
    rate_lim_deg_s = 200; % From manufactorer: 429 deg/s without load.
    [t,y] = ode45(@(t,y) f(t,y,u_step,rate_lim_deg_s,T), tspan, delta_a_0);
    subplot(1,3,i)
    plot(t,y); hold on;
end

legend("Experimental data", "Simulated model",'Location','southeast');
sgtitle("Control surface step responses")

function delta_e_dot = f(t, delta_e, delta_e_sp, rate_lim, T)
    delta_e_dot = bound(-1/T * delta_e + 1/T * delta_e_sp, -rate_lim, rate_lim);
end

function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y=min(max(x,bl),bu);
end