% Calculating deflection angles in degrees from raw PX4 input:
% 1. Saturate PX4 input at [-1,1]
% 2. Subtract trim values
% 3. Use the following function to calculate deflection angles in deg:
%    delta_deg = offset + input * linear_term

% These data are obtained in the script:
% actuator_id/control_surfaces/control_surface_scaling.m

% Trims to get zero deflection angles
% These are the ones programmed in PX4
trim_for_zero_deg_roll = -0.11;
trim_for_zero_deg_pitch = -0.03;
trim_for_zero_deg_yaw = -0.02;

% Scaling from PX4 input (with trims subtracted) to degrees
offset_rudder = -0.7467;
linear_term_rudder = 22.3333;
offset_elevator = -1.0700;
linear_term_elevator = 25.6667;
offset_aileron = -2.7433;
linear_term_aileron = 27.6667;

% Max angles (not really used for anything)
aileron_max_deg = 25;
elevator_max_deg = 25;
rudder_max_deg = 22;

% Dynamics
servo_rate_lim_rad_s = 3.4907;
servo_time_const_s = 0.028;



