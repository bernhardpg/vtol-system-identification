clc; clear all; close all;

% Continue following this guide: https://se.mathworks.com/help/ident/ug/industrial-three-degrees-of-freedom-robot-c-mex-file-modeling-of-mimo-system-using-vector-matrix-parameters.html

metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);
[full_state, full_input, t, maneuver_start_indices] = read_experiment_data(metadata);

% Load airframe properties
aircraft_properties;

% Create ID data from each experiment
[data] = create_iddata(full_state, full_input, maneuver_start_indices, t);

state_size = 8;
num_experiments = length(data.Experiment);

%% Load previous model
model_number_to_load = 2;
model_load_path = "nlgr_models/" + "model_" + model_number_to_load + "/";
load(model_load_path + "model.mat");
old_parameters = nlgr_model.Parameters;

%% Create new nlgr object
parameters = create_lon_parameter_struct();

% Create model path
model_number = 3;
model_path = "nlgr_models/" + "model_" + model_number + "/";

experiments_to_use = [1:5];
initial_states = create_initial_states_struct(data, state_size, experiments_to_use);

[nlgr_model] = create_nlgr_object(parameters, initial_states);
%[nlgr_model] = load_parameters(nlgr_model, old_parameters);

nlgr_model = fix_parameters([9 10 13 14 15], nlgr_model);


%%
opt = nlgreyestOptions('Display', 'on');
opt.SearchOptions.MaxIterations = 100;

weight = diag(ones(7,1));
weight(1,1) = 0;
weight(2,2) = 0;
weight(3,3) = 0;
weight(4,4) = 0;
weight(5,5) = 10;
opt.OutputWeight = weight;

%% Estimate NLGR model
%nlgr_model = unfix_parameters([9 10 13 14 15], nlgr_model);
nlgr_model = nlgreyest(data(:,:,:,experiments_to_use), nlgr_model, opt);
parameters = nlgr_model.Parameters;
print_parameters(nlgr_model.Parameters);


%%
print_parameters(nlgr_model.Parameters);
compare(nlgr_model, data(:,:,:,3));
%sim_response(experiments_to_use, nlgr_model, data(:,:,:,experiments_to_use), model_path, true);
%compare(data(:,:,:,experiments_to_use), nlgr_model)
%% Save model

mkdir(model_path)
save(model_path + "model.mat", 'nlgr_model');

%%
print_parameters(parameters);

%%
%nlgr.SimulationOptions.RelTol = 1e-5;
%compare(data, nlgr);



%%


%opt = nlgreyestOptions('Display', 'on', 'SearchMethod', 'fmincon');
%opt = nlgreyestOptions('SearchMethod', 'lm', 'Display', 'on');


%%

function [nlgr_model] = load_parameters(nlgr_model, params)
    for i = 1:length(nlgr_model.Parameters)
        for j = 1:length(params)
            if strcmp(nlgr_model.Parameters(i).Name, params(j).Name)
                nlgr_model.Parameters(i).Value = params(j).Value;
            end
        end
    end
end

function [] = print_parameters(parameters)
    num_parameters = length(parameters);
    disp(" ")
    disp("=== Parameter values ===")
    for i = 1:num_parameters
       param_fixed = parameters(i).Fixed;
       if i > 8
           param_name = parameters(i).Name;
           param_value = parameters(i).Value;
           disp(param_name + " = " + param_value);
       end
    end
end


function [nlgr_model] = fix_parameters(parameters_to_fix, nlgr_model)
    for i = parameters_to_fix
        nlgr_model.Parameters(i).Fixed = 1;
    end
end

function [nlgr_model] = unfix_parameters(parameters_to_fix, nlgr_model)
    for i = parameters_to_fix
        nlgr_model.Parameters(i).Fixed = 0;
    end
end

function [state, input, t, maneuver_start_indices] = read_experiment_data(metadata)
    num_experiments = length(metadata.Experiments);
    experiment_data_path = "data/experiments/";

    state = [];
    input = [];
    maneuver_start_indices = [];
    t = [];

    for i = 1:num_experiments
        datapath = experiment_data_path + "experiment_" + metadata.Experiments(i).Number ...
            + "/pitch_211_no_throttle/output/";
        if ~exist(datapath, 'dir')
            continue
        end
            
        state_exp = readmatrix(datapath + "state.csv");
        input_exp = readmatrix(datapath + "input.csv");
        maneuver_start_indices_exp = readmatrix(datapath + "maneuver_start_indices.csv");
        t_exp = readmatrix(datapath + "t.csv");

        state = [state;
                 state_exp];
        input = [input;
                 input_exp];
        maneuver_start_indices = [maneuver_start_indices...
            maneuver_start_indices_exp];
        t = [t;
             t_exp];
    end
    
    disp("Loaded " + length(maneuver_start_indices) + " maneuvers.")
end

function [data] = create_iddata(full_state, full_input, maneuver_start_indices, t)
    num_maneuvers = length(maneuver_start_indices);
    dt = t(2) - t(1);

    data = iddata('Name', 'Longitudinal data');

    % Describe input
    InputName = {'delta_e_sp','delta_t_fw'};
    InputUnit =  {'rad', 'rpm'};

    % Describe state (which is equal to output)
    OutputName = {'q0', 'q1', 'q2', 'q3', 'q', 'u', 'w'};
    OutputUnit = {'', '', '', '', 'rad/s', 'm/s', 'm/s'};

    for i = 1:num_maneuvers
        if i == 1
            maneuver_start_index = 1;
        else
            maneuver_start_index = maneuver_start_indices(i - 1);
        end
        
        maneuver_end_index = maneuver_start_indices(i) - 1;

        % Extract only relevant maneuver data
        t_maneuver = t(maneuver_start_index:maneuver_end_index,:);
        full_state_maneuver = full_state(maneuver_start_index:maneuver_end_index,:);
        full_input_maneuver = full_input(maneuver_start_index:maneuver_end_index,:);

        quat = full_state_maneuver(:,1:4);
        quat_only_pitch = create_quat_w_only_pitch_movement(quat);
        
        q = full_state_maneuver(:,6);
        u = full_state_maneuver(:,8);
        w = full_state_maneuver(:,10);
        delta_e = full_input_maneuver(:,6);
        delta_t_fw = full_input_maneuver(:,8);

        output = [quat_only_pitch q u w];
        input = [delta_e delta_t_fw];

        % Create sysid data object
        z = iddata(output, input, dt, 'Name', 'Pitch 211 maneuvers');
        z.TimeUnit = 's';
        z.Tstart = 0;
        z.InputName = InputName;
        z.InputUnit = InputUnit;
        z.OutputName = OutputName;
        z.OutputUnit = OutputUnit;

        if i == 1
            data = z;
        else
            data = merge(data,z);
        end
    end
end


function [quat_only_pitch] = create_quat_w_only_pitch_movement(quat)
    eul = quat2eul(quat);
    roll = eul(:,3);
    pitch = eul(:,2);
    yaw = eul(:,1);

    eul_only_pitch = [zeros(size(pitch)) pitch zeros(size(pitch))];
    quat_only_pitch = eul2quat(eul_only_pitch);
end


function [parameters] = create_lon_parameter_struct()
    aircraft_properties;
    initial_guess_lon;
    approx_zero = eps;
    
    ParName = {
        'g',                ...
        'half_rho_planform', ...
        'mass',					...
        'mean_chord_length',              ...
        'wingspan',					...
        'nondim_constant_lon', ...
        'lam',				...
        'J_yy' ,            ...
        'servo_time_constant',...
        'servo_rate_lim_rad_s',...
        'c_L_0',				...
        'c_L_alpha',      	...
        'c_L_q',          	...
        'c_L_delta_e',    	...
        'c_D_p',				...
        'c_D_alpha',				...
        'c_D_alpha_sq',				...
        'c_D_q',          	...
        'c_D_delta_e',    	...
        'c_m_0',				...
        'c_m_alpha',          ...
        'c_m_q',				...
        'c_m_delta_e',		...
    };

    ParFixed = {
        true,... % g,                  ...
        true,... % half_rho_planform, ...
        true,... % mass_kg,					...
        true,... % mean_chord_length,              ...
        true,... % wingspan,					...
        true,... %nondim_constant_lon
        true,... % lam,				...
        true,... % Jyy, ...
        true,... % servo_time_const,...
        true,... % servo_rate_lim_rad_s,...
        false,... % c_L_0,				...
        false,... % c_L_alpha,      	...
        false,... % c_L_q,          	...
        false,... % c_L_delta_e,    	...
        false,... % c_D_p,				...
        false,... % c_D_alpha,          ...
        false,... % c_D_alpha_sq,          ...
        false,... % c_D_q,          	...
        false,... % c_D_delta_e,    	...
        false,... % c_m_0,				...
        false,... % c_m_alpha,          ...
        false,... % c_m_q,				...
        false,... % c_m_delta_e,		...
    };

    ParMin = {
        -Inf,...
        -Inf,...
        -Inf,...
        -Inf,...
        -Inf,...
        -Inf,...
        -Inf,...
        -Inf,...
        approx_zero,... % servo_time_const
        servo_rate_lim_rad_s,... % servo_time_const
        approx_zero,... % c_L_0,				...
        approx_zero,... % c_L_alpha,      	...
        approx_zero,... % c_L_q,          	...
        approx_zero,... % c_L_delta_e,    	...
        approx_zero, ... % c_D_p,				...
        approx_zero, ...% c_D_alpha,          ...
        approx_zero, ...% c_D_alpha_sq,          ...
        approx_zero,... % c_D_q,          	...
        approx_zero,... % c_D_delta_e,    	...
        approx_zero,... % c_m_0,				...
        -Inf,... % c_m_alpha,          ...
        -Inf,... % c_m_q,				...
        -Inf,... % c_m_delta_e,		...
    };

    ParMax = {
        Inf,...
        Inf,...
        Inf,...
        Inf,...
        Inf,...
        Inf,...
        Inf,...
        Inf,...
        Inf,... % servo_time_const
        Inf,... % servo_rate_lim_rad_s
        Inf,... % c_L_0,				...
        Inf,... % c_L_alpha,      	...
        Inf,... % c_L_q,          	...
        Inf,... % c_L_delta_e,    	...
        Inf, ... % c_D_p,				...
        Inf, ...% c_D_alpha,          ...
        Inf, ...% c_D_alpha_sq,          ...
        Inf,... % c_D_q,          	...
        Inf,... % c_D_delta_e,    	...
        Inf,... % c_m_0,				...
        -approx_zero,... % c_m_alpha,          ...
        -approx_zero,... % c_m_q,				...
        -approx_zero,... % c_m_delta_e,		...
    };

    ParValue = {
        g,                  ...
        half_rho_planform, ...
        mass_kg,					...
        mean_aerodynamic_chord_m,              ...
        wingspan_m,					...
        nondim_constant_lon, ...
        lam,				...
        Jyy, ...
        servo_time_const, ...
        servo_rate_lim_rad_s,...
        c_L_0,				...
        c_L_alpha,      	...
        c_L_q,          	...
        c_L_delta_e,    	...
        c_D_p,				...
        c_D_alpha,          ...
        c_D_alpha_sq,          ...
        c_D_q,          	...
        c_D_delta_e,    	...
        c_m_0,				...
        c_m_alpha,          ...
        c_m_q,				...
        c_m_delta_e,		...
    };


    parameters = struct('Name', ParName, ...
        'Unit', '',...
        'Value', ParValue, ...
        'Minimum', ParMin, ...
        'Maximum', ParMax, ...
        'Fixed', ParFixed);
end

function [initial_states] = create_initial_states_struct(data, state_size, experiments_to_use)
    initial_states_values = {};
    if length(experiments_to_use) == 1
       for i = 1:state_size - 1
           initial_states_values(i) = {data(1,i,:,experiments_to_use).y};
       end
       initial_states_values(8) = {data(1,:,1,experiments_to_use).u};
    else
        for i = 1:state_size - 1
           initial_states_values(i) = {cell2mat(data(1,i,:,experiments_to_use).y)'};
        end
        % Load initial conditions for elevator
        initial_states_values(8) = {cell2mat(data(1,:,1,experiments_to_use).u)'};
    end
    


    initial_states = struct(...
        'Name', {'q0', 'q1', 'q2', 'q3', 'q','u', 'w','delta_e'},...
        'Unit', {'', '', '', '', 'rad/s', 'm/s', 'm/s','rad'}, ...
        'Value', initial_states_values, ...
        'Minimum', -Inf, 'Maximum', Inf, ...
        'Fixed', true);
end

function [nlgr] = create_nlgr_object(parameters, initial_states)
    % Create model
    FileName = 'longitudinal_model_w_actuator_dynamics_c';
    Nx = 8; % number of states
    Ny = 7; % number of outputs
    Nu = 2; % number of inputs
    Order = [Ny Nu Nx];

    % Describe input
    InputName = {'delta_e_sp','delta_t_fw'};
    InputUnit =  {'rad', 'rpm'};

    % Describe state (which is equal to output)
    OutputName = {'q0', 'q1', 'q2', 'q3', 'q', 'u', 'w'};
    OutputUnit = {'', '', '', '', 'rad/s', 'm/s', 'm/s'};

    % Construct nlgr object
    Ts = 0; % Continuous system
    nlgr = idnlgrey(FileName, Order, parameters, initial_states, Ts, ...
        'Name', 'Longitudinal Model', ...
        'InputName', InputName, 'InputUnit', InputUnit, ...
        'OutputName', OutputName, 'OutputUnit', OutputUnit, ...
        'TimeUnit', 's');
end

function [] = sim_response(experiments_to_use, nlgr_model, data, model_path, save_plots)
    num_experiments = length(experiments_to_use);
    if num_experiments == 1
        exp_i = experiments_to_use;
        y = sim(nlgr_model, data);
        predicted_state = y.y;
        state = data.y;
        input = data.u;
        dt = data.Ts;
        plot_response(exp_i, state, predicted_state, input, dt, true, model_path, save_plots);
    else
        for i = 1:num_experiments
            exp_i = experiments_to_use(i);
            y = sim(nlgr_model, data);
            predicted_state = cell2mat(y.y(i));
            state = cell2mat(data.y(i));
            input = cell2mat(data.u(i));
            dt = cell2mat(data.Ts(i));
            plot_response(exp_i, state, predicted_state, input, dt, true, model_path, save_plots);
        end
    end
end

function [] = plot_response(exp_i, state, predicted_state, input, dt, plot_actual_trajectory, model_path, save_plots)
    tf = length(state) * dt - dt;
    t = 0:dt:tf;
        
    quat = state(:,1:4);
    eul = quat2eul(quat);
    quat_pred = predicted_state(:,1:4);
    eul_pred = quat2eul(quat_pred);

    theta_pred = eul_pred(:,2);
    q_pred = predicted_state(:,5);
    u_pred = predicted_state(:,6);
    w_pred = predicted_state(:,7);

    theta = eul(:,2);
    q = state(:,5);
    u = state(:,6);
    w = state(:,7);

    fig = figure;
    fig.Position = [100 100 600 600];
    subplot(6,1,1)
    plot(t, theta_pred); 
    if plot_actual_trajectory
        hold on
        plot(t, theta);
    end
    legend("\theta (estimated)", "\theta")

    subplot(6,1,2)
    plot(t, q_pred);
    if plot_actual_trajectory
        hold on
        plot(t, q);
    end
    legend("q (estimated)", "q")

    subplot(6,1,3)
    plot(t, u_pred);
    if plot_actual_trajectory
        hold on
        plot(t, u);
    end
    legend("u (estimated)", "u")

    subplot(6,1,4)
    plot(t, w_pred);
    if plot_actual_trajectory
        hold on
        plot(t, w);
    end
    legend("w (estimated)", "w")

    subplot(6,1,5)
    plot(t, input(:,1));
    legend("\delta_e")

    subplot(6,1,6)
    plot(t, input(:,2));
    legend("\delta_t")
    
    sgtitle("experiment index: " + exp_i)
    
    if save_plots
        filename = exp_i + "_long";
        plot_location = model_path + "plots/";
        mkdir(plot_location);
        saveas(fig, plot_location + filename, 'epsc')
        savefig(plot_location + filename + '.fig')
    end
end
