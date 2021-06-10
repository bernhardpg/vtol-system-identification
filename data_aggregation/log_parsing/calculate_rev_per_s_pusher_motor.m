function [rev_per_s] = calculate_rev_per_s_pusher_motor(px4_input)
    pwm_signal = convert_pusher_motor_input_to_pwm_scale(px4_input);
    rev_per_s = convert_pusher_pwm_to_rev_per_s(pwm_signal);
end

function [pwm_signal] = convert_pusher_motor_input_to_pwm_scale(px4_input)
    % Follows the following format
    %    pwm = scale * input + offset
    min_pwm = 950; % From PX4 parameters
    max_pwm = 2000; % From PX4 parameters
    pusher_motor_input_to_pwm_scale = max_pwm - min_pwm;
    pusher_motor_input_to_pwm_offset = min_pwm;
    pwm_signal = pusher_motor_input_to_pwm_offset + ...
        pusher_motor_input_to_pwm_scale .* px4_input;
end

function [rev_per_s] = convert_pusher_pwm_to_rev_per_s(pwm_signal)
    % Follows the following format:
    %    rpm = scale * pwm + offset
    pwm_to_rpm_scale = 0.18676; % See motor_id.m
    pwm_to_rpm_offset = -200.1543; % See motor_id.m
    
    % Never let rev_per_s be negative
    rev_per_s = max(pwm_to_rpm_scale .* pwm_signal + pwm_to_rpm_offset, 0);
end