% Trim values for steady flight
% These are absolute, i.e. in raw PX4 inputs. Trims needs to be
% subtracted from these before converting to degrees
aileron_trim_px4_input = -0.03; % 06_31_21.ulg 12:55 to 13:08
elevator_trim_px4_input = 0.12; % 06_31_21.ulg 17:20-17:26
rudder_trim_px4_input = -0.12; % 06_31_21.ulg 12:55 to 13:05

% load control surface properties
control_surface_properties;

% 1. Subtract trims hardcoded in PX4
aileron_trim_px4_input_wo_trim = aileron_trim_px4_input - trim_for_zero_deg_roll;
elevator_trim_px4_input_wo_trim = elevator_trim_px4_input - trim_for_zero_deg_pitch;
rudder_trim_px4_input_wo_trim = rudder_trim_px4_input - trim_for_zero_deg_yaw;

% 2. Convert inputs to degrees
delta_a_trim_deg = linear_term_aileron * aileron_trim_px4_input_wo_trim + offset_aileron;
delta_e_trim_deg = linear_term_elevator * elevator_trim_px4_input_wo_trim + offset_elevator;
delta_r_trim_deg = linear_term_rudder * rudder_trim_px4_input_wo_trim + offset_rudder;

% 3. Convert degrees to radians
delta_a_trim_rad = deg2rad(delta_a_trim_deg);
delta_e_trim_rad = deg2rad(delta_e_trim_deg);
delta_r_trim_rad = deg2rad(delta_r_trim_deg);

% TODO: Continue here