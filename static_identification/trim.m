% Cruise conditions
V_nom = 21; % Nominal airspeed, m/s
alpha_nom = 3 / 180 * pi;
u_nom = V_nom / (sqrt(1 + tan(alpha_nom)^2));
w_nom = sqrt(V_nom^2 - u_nom^2);
delta_e_nom = -0.0985;
delta_a_nom = 0.0529;
delta_r_nom = 0;

% % Find delta_t at trim
% q_bar_trim = (1/2) * rho * V_trim^2;
% D_trim = (c_D_0 + c_D_alpha * alpha_trim + c_D_alpha_sq * alpha_trim^2) * q_bar_trim;
% T_trim = D_trim;
% delta_t_trim = T_trim / (rho * D_FW^4 * c_T_FW);