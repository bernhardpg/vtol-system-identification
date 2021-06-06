% Calculating deflection angles in degrees from raw PX4 input:
% 1. PX4 inputs are WITH trims added. This is fine.
% 2. Use raw PX4 input to calculate deflection angle as
%    delta_deg = offset + input * linear_term
% 3. Saturate deflection angle at known limits

% Trims that are required for zero deflection angles
% due to manifacturing inaccuriacies
% (These are the TRIM_* parameters in PX4)
trim_for_zero_deg_roll = -0.11;
trim_for_zero_deg_pitch = -0.03;
trim_for_zero_deg_yaw = -0.02;

% These data are obtained in the script:
% actuator_id/control_surfaces/control_surface_scaling.m

% Scaling from PX4 inputs to degrees
offset_rudder = 0.1467;
linear_term_rudder = 22.3333;
offset_elevator = 0.4700;
linear_term_elevator = 25.6667;
offset_aileron = 3.3433;
linear_term_aileron = 27.6667;

% Max angles in actual deflections
aileron_max_deg = 25;
elevator_max_deg = 25;
rudder_max_deg = 22;

% Servo dynamics
servo_rate_lim_rad_s = 3.4907;
%servo_rate_lim_rad_s = 99;
servo_time_const_s = 0.028;



