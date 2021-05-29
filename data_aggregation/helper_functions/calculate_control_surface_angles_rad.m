function [delta_a_rad, delta_e_rad, delta_r_rad] = calculate_control_surface_angles_rad(aileron_input, elevator_input, rudder_input)
    control_surface_properties; % See this file for a more detailed description
    
    % 1. Saturate inputs at [-1,1]
    aileron_input_saturated = bound(aileron_input, -1, 1);
    elevator_input_saturated = bound(elevator_input, -1, 1);
    rudder_input_saturated = bound(rudder_input, -1, 1);
    
    % 2. Subtract trim values
    aileron_input_saturated_wo_trim = aileron_input_saturated - trim_for_zero_deg_roll;
    elevator_input_saturated_wo_trim = elevator_input_saturated - trim_for_zero_deg_pitch;
    rudder_input_saturated_wo_trim = rudder_input_saturated - trim_for_zero_deg_yaw;
    
    % 3. Convert inputs to degrees
    delta_a_deg = linear_term_aileron * aileron_input_saturated_wo_trim + offset_aileron;
    delta_e_deg = linear_term_elevator * elevator_input_saturated_wo_trim + offset_elevator;
    delta_r_deg = linear_term_rudder * rudder_input_saturated_wo_trim + offset_rudder;
    
    % Convert degrees to radians
    delta_a_rad = deg2rad(delta_a_deg);
    delta_e_rad = deg2rad(delta_e_deg);
    delta_r_rad = deg2rad(delta_r_deg);
end

function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y = min(max(x,bl),bu);
end