function [aileron_angle_rad, elevator_angle_rad, rudder_angle_rad] = calculate_control_surface_angles(aileron_input, elevator_input, rudder_input)
    % Control surfaces
    aileron_input_to_aileron_deg = 28.1713; % See control_surface_scaling.m
    elevator_input_to_tail_deg = 59.6735; % See control_surface_scaling.m
    rudder_input_to_tail_deg = 52.2143; % See control_surface_scaling.m
    
    % Convert input signals to degrees
    % Assumes that the elevator and rudder can be controlled individually.
    % TODO implement max and min here?
    aileron_angle_deg = aileron_input .* aileron_input_to_aileron_deg;
    elevator_angle_deg = elevator_input .* elevator_input_to_tail_deg;
    rudder_angle_deg = rudder_input .* rudder_input_to_tail_deg;
    
    % Convert degrees to radians
    aileron_angle_rad = deg2rad(aileron_angle_deg);
    elevator_angle_rad = deg2rad(elevator_angle_deg);
    rudder_angle_rad = deg2rad(rudder_angle_deg);
end