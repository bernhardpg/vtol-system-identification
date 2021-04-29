%%% Initial guesses
% Import lift and drag initial guesses
lift_drag_properties;

% Lift parameters
small_number = 0.01;
approx_zero = eps(0);
c_L_q = rand_num_in_interval(-small_number, small_number);
c_L_delta_e = rand_num_in_interval(-small_number, small_number);

% Y-aerodynamic force
c_Y_p = 0;
c_Y_r = 0;
c_Y_delta_a = 0;
c_Y_delta_r = 0;

% Aerodynamic moment around x axis
c_l_p = rand_num_in_interval(-0.5, approx_zero); % Dynamic damping derivative, should be negative
c_l_r = 0; % Cross coupling
c_l_delta_a = rand_num_in_interval(0.6, 1); % Control derivative, should be positive to follow PX4 convention
c_l_delta_r = 0; % Cross-control derivative

% Aerodynamic moment around y axis
c_m_0 = rand_num_in_interval(-small_number, small_number);
c_m_alpha = rand_num_in_interval(-0.5, -approx_zero); % Spring coefficient. Should be negative for stable aircraft
c_m_q = rand_num_in_interval(-0.5, -approx_zero); % Dynamic damping derivative. Should be negative
c_m_delta_e = rand_num_in_interval(0.6, 1); % Control derivative. Should be positive to follow PX4 convention

% Aerodynamic moment around z axis
c_n_p = 0; % Cross coupling
c_n_r = rand_num_in_interval(-0.5, -approx_zero); % Dynamic damping derivative. Should be negative
c_n_delta_a = 0; % Cross-control derivative
c_n_delta_r = rand_num_in_interval(0.6, 1); % Control derivative. Should be positive to follow PX4 convention

% % Parameters from Aerosonde UAV
% % Lift parameters
% c_L_q = 0;
% c_L_delta_e = -0.36; % sign on this?
% 
% % Y-aerodynamic force
% c_Y_p = 0;
% c_Y_r = 0;
% c_Y_delta_a = 0;
% c_Y_delta_r = -0.17;
% 
% % Aerodynamic moment around x axis
% c_l_p = -0.26;
% c_l_r = 0.14;
% c_l_delta_a = 0.08;
% c_l_delta_r = 0.105;
% 
% % Aerodynamic moment around y axis
% c_m_0 = -0.02;
% c_m_alpha = -0.38;
% c_m_q = -0.5;
% c_m_delta_e = 1;
% 
% % Aerodynamic moment around z axis
% c_n_p = 0.022;
% c_n_r = -0.35;
% c_n_delta_a = 0.06;
% c_n_delta_r = 0.032;


%%

function [r] = rand_num_in_interval(min, max)
    r = min + (max - min) * rand();
end
