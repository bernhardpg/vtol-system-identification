% Raw PX4 inputs required for steady flight
roll_input_trim_raw = -0.03; % 06_31_21.ulg 12:55 to 13:08
pitch_input_trim_raw = 0.12; % 06_31_21.ulg 17:20-17:26
yaw_input_trim_raw = -0.12; % 06_31_21.ulg 12:55 to 13:05

% load control surface properties
control_surface_properties;

% 1. Subtract PX4 trims to obtain trim values in absolute inputs
% (where zero input => zero deflection angle)
roll_input_trim_abs = roll_input_trim_raw - trim_for_zero_deg_roll;
pitch_input_trim_abs = pitch_input_trim_raw - trim_for_zero_deg_pitch;
yaw_input_trim_abs = yaw_input_trim_raw - trim_for_zero_deg_yaw;

% 2. Convert absolute inputs to degrees
delta_a_trim_deg = linear_term_aileron * roll_input_trim_abs + offset_aileron;
delta_e_trim_deg = linear_term_elevator * pitch_input_trim_abs + offset_elevator;
delta_r_trim_deg = linear_term_rudder * yaw_input_trim_abs + offset_rudder;

% 3. Convert degrees to radians
delta_a_trim_rad = deg2rad(delta_a_trim_deg);
delta_e_trim_rad = deg2rad(delta_e_trim_deg);
delta_r_trim_rad = deg2rad(delta_r_trim_deg);