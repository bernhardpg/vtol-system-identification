clc; clear all; close all;

% Continue following this guide: https://se.mathworks.com/help/ident/ug/industrial-three-degrees-of-freedom-robot-c-mex-file-modeling-of-mimo-system-using-vector-matrix-parameters.html

% Import data
y_train = readmatrix('training_state.csv');
u_train = readmatrix('training_input.csv');
y_test = readmatrix('test_state.csv');
u_test = readmatrix('test_input.csv');

dt = 1 / 100; % See data_handler.m

%% Create sysid data object

sysid_data = iddata(y_train, u_train, dt, 'Name', 'VTOL-roll');

sysid_data.InputName = {'nt1', 'nt2', 'nt3', 'nt4',...
    'delta_a', 'delta_e', 'delta_r', 'np'};
sysid_data.InputUnit =  {'rpm', 'rpm', 'rpm', 'rpm', ...
    'rpm', 'rad', 'rad', 'rad'};
sysid_data.OutputName = {'q0', 'q1', 'q2', 'q3', ...
    'p', 'q', 'r', 'u', 'v', 'w'};
sysid_data.OutputUnit = {'', '', '', '', 'rad/s', 'rad/s', 'rad/s', ...
    'm/s', 'm/s', 'm/s'};
sysid_data.Tstart = 0;
sysid_data.TimeUnit = 's';

figure('Name', [sysid_data.Name ': Aileron input -> Attitude output']);
plot(sysid_data(:, 1:4, 5));   % Plot first input-output pair (Voltage -> Angular position).

%% Create nonlinear grey box model

%%%%%
% Model parameters
%%%%%

FileName = 'vtol_c';
Nx = 10; % number of states
Ny = 10; % number of outputs
Nu = 8; % number of inputs
Order = [Ny Nu Nx];

%%%%%
% Constants
%%%%%

% Physical constants
rho = 1.225; % kg / m^3 (air density at sea level)
g = 9.81; % m / s^2

% Propellers and motors
kINCH_TO_METER = 0.0254;
prop_diam_top_in_inches = 16;
prop_diam_top = prop_diam_top_in_inches * kINCH_TO_METER;
prop_diam_pusher_in_inches = 15;
prop_diam_pusher = prop_diam_pusher_in_inches * kINCH_TO_METER;
c_F_top = 0;
c_F_pusher = 0;
c_Q_top = 0;
c_Q_pusher = 0;

% Airframe
m = 13; % TODO update
S = 0.75; % TODO update
chord = 0.5; % TODO update
b = 2.5; % wingspan
lam = zeros(3); % TODO base this on moment of inertia
r_t1 = [0; 0; 0];
r_t2 = [0; 0; 0];
r_t3 = [0; 0; 0];
r_t4 = [0; 0; 0];

%%%%%
% Parameters
%%%%%

% Lift parameters
c_L_0 = 0;
c_L_alpha = 0;
c_L_q = 0;
c_L_delta_e = 0;

% Blending function
M = 0;
alpha_stall = 0;

% Drag parameters
c_D_p = 0;

% Y-aerodynamic force
c_Y_p = 0;
c_Y_r = 0;
c_Y_delta_a = 0;
c_Y_delta_r = 0;

% Aerodynamic moment around x axis
c_l_p = 0;
c_l_r = 0;
c_l_delta_a = 0;
c_l_delta_r = 0;

% Aerodynamic moment around y axis
c_m_0 = 0;
c_m_alpha = 0;
c_m_q = 0;
c_m_delta_e = 0;

% Aerodynamic moment around z axis
c_n_p = 0;
c_n_r = 0;
c_n_delta_a = 0;
c_n_delta_r = 0;

% Build model
ParName = {
    'rho',				...
    'g',                ...
    'prop_diam_top',  	...
    'prop_diam_pusher',   ...
    'c_F_top',			...
    'c_F_pusher',         ...
    'c_Q_top',			...
    'c_Q_pusher',         ...
    'm',					...
    'S',					...
    'chord',              ...
    'b',					...
    'lam',				...
    'r_t1',               ...
    'r_t2',				...
    'r_t3',				...
    'r_t4'				...
    'c_L_0',				...
    'c_L_alpha',      	...
    'c_L_q',          	...
    'c_L_delta_e',    	...
    'M',              	...
    'alpha_stall',    	...
    'c_D_p',				...
    'c_Y_p',				...
    'c_Y_r',				...
    'c_Y_delta_a',		...
    'c_Y_delta_r',		...
    'c_l_p',              ...
    'c_l_r',				...
    'c_l_delta_a',		...
    'c_l_delta_r',        ...
    'c_m_0',				...
    'c_m_alpha',          ...
    'c_m_q',				...
    'c_m_delta_e',		...
    'c_n_p',				...
    'c_n_r',				...
    'c_n_delta_a',		...
    'c_n_delta_r',        ...
};

ParValue = {
    rho,				...
    g,                  ...
    prop_diam_top,  	...
    prop_diam_pusher,   ...
    c_F_top,			...
    c_F_pusher,         ...
    c_Q_top,			...
    c_Q_pusher,         ...
    m,					...
    S,					...
    chord,              ...
    b,					...
    lam,				...
    r_t1,               ...
    r_t2,				...
    r_t3,				...
    r_t4,				...
    c_L_0,				...
    c_L_alpha,      	...
    c_L_q,          	...
    c_L_delta_e,    	...
    M,              	...
    alpha_stall,    	...
    c_D_p,				...
    c_Y_p,				...
    c_Y_r,				...
    c_Y_delta_a,		...
    c_Y_delta_r,		...
    c_l_p,              ...
    c_l_r,				...
    c_l_delta_a,		...
    c_l_delta_r,        ...
    c_m_0,				...
    c_m_alpha,          ...
    c_m_q,				...
    c_m_delta_e,		...
    c_n_p,				...
    c_n_r,				...
    c_n_delta_a,		...
    c_n_delta_r        ...
};

ParUnit = {
    'kg/m^3',   ...
    'm/s^2',    ...
    'm',   ...
    'm',   ...
    '',			...
    '',			...
    '',			...
    '',			...
    'kg',			...
    'm^2',		...
    'm',		...
    'm',		...
    '',			...
    'm',		...
    'm',		...
    'm',		...
    'm',		...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    '',			...
    ''
};
ParFixed = {
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    true, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
    false, ...
};

% TODO set parameter max and min
ParMin = -Inf;
ParMax   = Inf;


Parameters    = struct('Name', ParName, 'Unit', ParUnit, 'Value', ParValue, ...
                       'Minimum', ParMin, 'Maximum', ParMax, 'Fixed', ParFixed);
InitialStates = struct(...
    'Name', {'q0', 'q1', 'q2', 'q3', 'p', 'q', 'r', 'u', 'v', 'w'},...
    'Unit', {'', '', '', '', 'rad/s', 'rad/s', 'rad/s', 'm/s', 'm/s', 'm/s'}, ...
    'Value', {1, 0, 0, 0, 0, 0, 0, 23, 0, 0}, ...
    'Minimum', -Inf, 'Maximum', Inf, 'Fixed', false);
         
Ts = 0;

nlgr = idnlgrey(FileName,Order,Parameters, InitialStates, Ts, ...
    'Name', 'VTOL_aircraft');
