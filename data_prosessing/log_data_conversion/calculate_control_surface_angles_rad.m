function [delta_a_rad, delta_e_rad, delta_r_rad] = calculate_control_surface_angles_rad(roll_px4_input, pitch_px4_input, yaw_px4_input)
    control_surface_constants; % See this file for a more detailed description
    
    % Calculating deflection angles in degrees from raw PX4 input:
    % 1. Logged PX4 inputs are WITH trims added. This is fine.
    % 2. Use raw PX4 input to calculate deflection angle as
    %    delta_deg = offset + input * linear_term
    % 3. Saturate deflection angle at known limits

    % 2. Convert inputs to degrees
    delta_a_deg = linear_term_aileron * roll_px4_input + offset_aileron;
    % Flip signs on rudder and elevator to match standard aerospace
    % conventions (for some reason PX4 does not do this)
    delta_e_deg = -(linear_term_elevator * pitch_px4_input + offset_elevator);
    delta_r_deg = -(linear_term_rudder * yaw_px4_input + offset_rudder);
    
    % Calculate actual deflections of each individual a-tail surface
%     delta_vl_deg = delta_e_deg - delta_r_deg;
%     delta_vr_deg = delta_e_deg + delta_r_deg;
    
    % 3. Saturate deflections at known deflection angles
%     delta_vl_deg = bound(delta_vl_deg, -elevator_max_deg, elevator_max_deg);
%     delta_vr_deg = bound(delta_vr_deg, -elevator_max_deg, elevator_max_deg);
    delta_a_deg = bound(delta_a_deg, -aileron_max_deg, aileron_max_deg);
    delta_e_deg = bound(delta_e_deg, -elevator_max_deg, elevator_max_deg);
    delta_r_deg = bound(delta_r_deg, -rudder_max_deg, rudder_max_deg);
    
    % Convert degrees to radians
    delta_a_rad = deg2rad(delta_a_deg);
    delta_e_rad = deg2rad(delta_e_deg);
    delta_r_rad = deg2rad(delta_r_deg);
    
%     delta_vl_rad = deg2rad(delta_vl_deg);
%     delta_vr_rad = deg2rad(delta_vr_deg);
end

function y = sat(x)
    y = bound(x,-1,1);
end

function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y = min(max(x,bl),bu);
end