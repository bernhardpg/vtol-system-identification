function [aileron_angle_rad, elevator_angle_rad, rudder_angle_rad] = calculate_control_surface_angles_rad(aileron_input, elevator_input, rudder_input)
    % Control surfaces
    aileron_input_to_aileron_deg = 28.1713; % See control_surface_scaling.m
    aileron_offset_deg = 2.1973;
    elevator_input_to_tail_deg = 59.6735; % See control_surface_scaling.m
    rudder_input_to_tail_deg = 52.2143; % See control_surface_scaling.m
    
    % Found from manual measurements
    max_elevator_angle_deg = 25;
    % TODO: It is still unclear what the real max rudder angle is, as this has not been measured experimentally. This may not matter, however.
    max_rudder_angle_deg = 25; 
    max_aileron_angle_deg = 25;
    
    % Convert input signals to degrees
    % Makes the assumption that the elevator and rudder can be controlled individually.
    aileron_angle_deg = min(...
        aileron_input .* aileron_input_to_aileron_deg + aileron_offset_deg,...
        max_aileron_angle_deg);
    elevator_angle_deg = min(...
        elevator_input .* elevator_input_to_tail_deg,...
        max_elevator_angle_deg);
    rudder_angle_deg = min(...
        rudder_input .* rudder_input_to_tail_deg,...
        max_rudder_angle_deg);
    
    % Convert degrees to radians
    aileron_angle_rad = deg2rad(aileron_angle_deg);
    elevator_angle_rad = deg2rad(elevator_angle_deg);
    rudder_angle_rad = deg2rad(rudder_angle_deg);
end