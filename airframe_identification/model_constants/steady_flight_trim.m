% Raw PX4 inputs required for steady flight
% Note: these values are only used to move the zero as seen by the model.
% These values DO NOT affect actual deflection angles
roll_input_trim = -0.03; % 06_31_21.ulg 12:55 to 13:08
pitch_input_trim = 0.12; % 06_31_21.ulg 17:20-17:26
yaw_input_trim = -0.12; % 06_31_21.ulg 12:55 to 13:05

pitch_input_trim = 0.12; % 06_31_21.ulg 17:20-17:26
% TODO: Not sure about roll and yaw

% Convert to right signs following standard aerospace notation
roll_input_trim = roll_input_trim;
pitch_input_trim = -pitch_input_trim;
yaw_input_trim = -yaw_input_trim;

% See 07_12_32.ulg from 2:48 to 2:54.

% load control surface properties
control_surface_properties;

% 2. Convert absolute inputs to degrees
delta_a_trim_deg = linear_term_aileron * roll_input_trim + offset_aileron;
delta_e_trim_deg = linear_term_elevator * pitch_input_trim + offset_elevator;
delta_r_trim_deg = linear_term_rudder * yaw_input_trim + offset_rudder;

% 3. Convert degrees to radians
delta_a_trim_rad = deg2rad(delta_a_trim_deg);
delta_e_trim_rad = deg2rad(delta_e_trim_deg);
delta_r_trim_rad = deg2rad(delta_r_trim_deg);

delta_e_trim_rad = -0.03;

