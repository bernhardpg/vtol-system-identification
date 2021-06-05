function [rev_per_s] = calculate_rev_per_s_pusher_motor(px4_input)
    pwm_signal = convert_pusher_motor_input_to_pwm_scale(px4_input);
    rpm = convert_pusher_pwm_to_rpm(pwm_signal);
    rev_per_s = rpm ./ 60;
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

function [rpm] = convert_pusher_pwm_to_rpm(pwm_signal)
    % Follows the following format:
    %    rpm = scale * pwm + offset
    pwm_to_rpm_scale = 11.2054; % See motor_id.m
    pwm_to_rpm_offset = -12009.2554; % See motor_id.m
    
    rpm = max(pwm_to_rpm_scale .* pwm_signal + pwm_to_rpm_offset, 0);
end