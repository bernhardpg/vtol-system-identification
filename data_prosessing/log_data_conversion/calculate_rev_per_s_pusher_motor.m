function [rev_per_s] = calculate_rev_per_s_pusher_motor(px4_input)
    pwm_signal = convert_pusher_motor_input_to_pwm_scale(px4_input);
    rev_per_s = convert_pusher_pwm_to_rev_per_s(pwm_signal);
end

function [pwm_signal] = convert_pusher_motor_input_to_pwm_scale(px4_input)
    fixedwing_propeller_constants;
    
    pwm_signal = pusher_motor_input_to_pwm_offset + ...
        pusher_motor_input_to_pwm_scale .* px4_input;
end

function [rev_per_s] = convert_pusher_pwm_to_rev_per_s(pwm_signal)
    fixedwing_propeller_constants;
    
    % Never let rev_per_s be negative
    rev_per_s = pwm_to_rps_th0 + pwm_to_rps_th1 * pwm_signal + pwm_to_rps_th2 * pwm_signal.^2;
    rev_per_s = max(rev_per_s, 0);
end