aircraft_properties;

% Directly from AVL
c_L_alpha = 4.732;
c_L_q = 7.375 ;
c_L_delta_e = 0.00632 ;
c_D_delta_e = 0.000257; % Trefftz drag
c_l_p = -0.472;
c_l_r = 0.141;
c_l_delta_a = -0.00494;
c_l_delta_r = 0.000145;
c_m_0 = 0.00970;
c_m_alpha = -0.974;
c_m_q = -11.882 ;
c_m_delta_e = -0.0208;
c_n_p = -0.0635;
c_n_r = -0.0869;
c_n_delta_a = -0.00018;
c_n_delta_r = 0.0019;

% Convert all control derivatives to rad instead of deg
c_L_delta_e = c_L_delta_e * 180 / pi;
c_D_delta_e = c_D_delta_e * 180 / pi;
c_l_delta_a = c_l_delta_a * 180 / pi;
c_l_delta_r = c_l_delta_r * 180 / pi;
c_m_delta_e = c_m_delta_e * 180 / pi;
c_n_delta_a = c_n_delta_a * 180 / pi;
c_n_delta_r = c_n_delta_r * 180 / pi;

% Non-dimensionalize ang rate derivatives
% TODO: I am not sure if AVL derivatives are nondimensionalized or not
c_L_q = c_L_q / nondim_constant_lon;
c_m_q = c_m_q / nondim_constant_lon;

c_l_p = c_l_p / nondim_constant_lat;
c_l_r = c_l_r / nondim_constant_lat;
c_n_p = c_n_p / nondim_constant_lat;
c_n_r = c_n_r / nondim_constant_lat;

% Flip axis to be correct
% All damping derivatives seem to have the correct sign
c_l_delta_a = abs(c_l_delta_a); % Pos deflection -> pos moment
c_m_delta_e = -abs(c_m_delta_e); % Pos deflection -> neg moment
c_n_delta_r = -abs(c_n_delta_r); % Pos deflection -> neg moment

