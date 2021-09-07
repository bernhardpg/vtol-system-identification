clear all; close all; clc;
%%
disp("##################")
disp("Longitudinal model")
disp("##################")
stepwise_regr_analysis_lon;
manual_equation_error_lon;

%%
disp("##################")
disp("Lateral model")
disp("##################")
stepwise_regr_analysis_lat;
manual_equation_error_lat;