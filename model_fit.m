clc; clear all; close all;

% Import data
y_train = readmatrix('training_state.csv');
u_train = readmatrix('training_input.csv');
y_test = readmatrix('test_state.csv');
u_test = readmatrix('test_input.csv');

dt = 1 / 100; % See data_handler.m

data_train = iddata(y_train, u_train, dt);


% Create nonlinear grey box model
FileName = 'vtol_c';
Nx = 10; % number of states
Ny = 10; % number of outputs
Nu = 8; % number of inputs
Order = [Ny Nu Nx];

Parameters = {0.5;0.0035;0.019; ...
    9.81;0.25;0.016};

InitialStates = [0;0.1];

Ts = 0;

nlgr = idnlgrey(FileName,Order,Parameters)