kINCH_TO_METER = 0.0254;
prop_diam_pusher_in_inches = 15;
prop_diam_pusher = prop_diam_pusher_in_inches * kINCH_TO_METER;

c_T_pusher = 0.0861; % See motor_id.m for calculation off this
c_Q_pusher = 0;

pwm_to_rpm_scale = 0.18676; % See motor_id.m
pwm_to_rpm_offset = -200.1543; % See motor_id.m

% Follows the following format:
%    rpm = scale * pwm + offset
min_pwm = 950; % From PX4 parameters
max_pwm = 2000; % From PX4 parameters
pusher_motor_input_to_pwm_scale = max_pwm - min_pwm;
pusher_motor_input_to_pwm_offset = min_pwm;