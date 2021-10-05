kINCH_TO_METER = 0.0254;
prop_diam_pusher_in_inches = 15;
prop_diam_pusher = prop_diam_pusher_in_inches * kINCH_TO_METER;

 % See propeller_id.m for calculation off these
c_T_pusher = 0.083977697623922;
c_Q_pusher = -0.005072037909591; % This is actually not used.

% From PWM to RPS
%pwm_to_rpm = -348.3499 + 0.38884 * pwm-6.6615e-05 * pwm^2
pwm_to_rps_th0 = -3.483498914350639e+02;
pwm_to_rps_th1 = 0.388836114895980;
pwm_to_rps_th2 = -6.661488559428497e-05;

% From PX4 input to PWM
min_pwm = 950; % From PX4 parameters
max_pwm = 2000; % From PX4 parameters
pusher_motor_input_to_pwm_scale = max_pwm - min_pwm;
pusher_motor_input_to_pwm_offset = min_pwm;