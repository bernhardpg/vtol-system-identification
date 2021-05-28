function [aileron_angle_rad, elevator_angle_rad, rudder_angle_rad] = calculate_control_surface_angles_rad(aileron_input, elevator_input, rudder_input)
    % Trim values
    % Calculate this in model instead
%     % Found from 2:48-2:52 in '07_12_32.ulg'
%     elevator_trim_px4_input = 0.15;
%     aileron_trim_px4_input = -0.05;
%     rudder_trim_px4_input = 0.1;
%     
%     % Subtract trim values from input signals
%     elevator_input_wo_trim = elevator_input - elevator_trim_px4_input;
%     aileron_input_wo_trim = aileron_input - aileron_trim_px4_input;
%     rudder_input_wo_trim = rudder_input - rudder_trim_px4_input;

    % Control surfaces
    aileron_input_to_aileron_deg = 22.4411; % See control_surface_scaling.m
    % NOTE: Signs are flipped to comply with standard aviation notation, as
    % in Beard & McClain
    elevator_input_to_tail_deg = -26.3083; % See control_surface_scaling.m
    rudder_input_to_tail_deg = -23.0197; % See control_surface_scaling.m
    
    % Found from manual measurements
    max_elevator_angle_deg = 26;
    % NOTE: It is still unclear what the real max rudder angle is,
    % as this has not been measured experimentally. This may not
    % matter that much though, as we are not using the max angle.
    max_rudder_angle_deg = 26;
    max_aileron_angle_deg = 25;
    
    % Convert input signals to degrees
    % Makes the assumption that the elevator and rudder can be controlled individually.
    aileron_angle_deg = bound(...
        aileron_input .* aileron_input_to_aileron_deg,...
        -max_aileron_angle_deg, max_aileron_angle_deg);
    elevator_angle_deg = bound(...
        elevator_input .* elevator_input_to_tail_deg,...
        -max_elevator_angle_deg, max_elevator_angle_deg);
    rudder_angle_deg = bound(...
        rudder_input .* rudder_input_to_tail_deg,...
        -max_rudder_angle_deg, max_rudder_angle_deg);
    
    % Convert degrees to radians
    aileron_angle_rad = deg2rad(aileron_angle_deg);
    elevator_angle_rad = deg2rad(elevator_angle_deg);
    rudder_angle_rad = deg2rad(rudder_angle_deg);
end

function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y = min(max(x,bl),bu);
end