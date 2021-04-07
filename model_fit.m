clc; clear all; close all;
y_train = readmatrix('training_state.csv');
u_train = readmatrix('training_input.csv');
y_test = readmatrix('test_state.csv');
u_test = readmatrix('test_input.csv');

dt = 1 / 100; % See data_handler.m

data_train = iddata(y_train, u_train, dt);
