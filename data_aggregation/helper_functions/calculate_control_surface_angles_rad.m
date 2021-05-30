function [delta_a_rad, delta_e_rad, delta_r_rad] = calculate_control_surface_angles_rad(roll_input_raw, pitch_input_raw, yaw_input_raw)
    control_surface_properties; % See this file for a more detailed description
    
    % 1. Saturate raw px4 inputs at [-1,1], as these will be
    %    saturated by PX4.
    roll_input_raw_sat = sat(roll_input_raw);
    pitch_input_raw_sat = sat(pitch_input_raw);
    yaw_input_raw_saw = sat(yaw_input_raw);
    
    % 2. Subtract PX4 trim params to obtain absolute inputs
    % (zero input => zero deflection angle)
    roll_input_abs = roll_input_raw_sat - trim_for_zero_deg_roll;
    pitch_input_abs = pitch_input_raw_sat - trim_for_zero_deg_pitch;
    yaw_input_abs = yaw_input_raw_saw - trim_for_zero_deg_yaw;
    
    % 3. Convert inputs to degrees
    delta_a_deg = linear_term_aileron * roll_input_abs + offset_aileron;
    % Flip signs on rudder and elevator to match standard aerospace
    % conventions (for some reason PX4 does not do this)
    delta_e_deg = -(linear_term_elevator * pitch_input_abs + offset_elevator);
    delta_r_deg = -(linear_term_rudder * yaw_input_abs + offset_rudder);
    
    % Convert degrees to radians
    delta_a_rad = deg2rad(delta_a_deg);
    delta_e_rad = deg2rad(delta_e_deg);
    delta_r_rad = deg2rad(delta_r_deg);
end

function y = sat(x)
    y = bound(x,-1,1);
end

function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y = min(max(x,bl),bu);
end