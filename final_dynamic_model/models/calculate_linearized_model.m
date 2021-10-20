clc; clear all; close all

% Import parameters
aerodynamic_coeffs;
static_parameters;
trim_values;

% Declare symbolic variables
syms u v w p q r phi theta;
syms delta_a delta_e delta_r delta_t;

% Aerodynamics
V = sqrt(u^2 + v^2 + w^2);
q_bar = (1/2) * rho * V^2;

alpha = atan(w/u);
beta = asin(v/V);

% Model is around perturbation quantities
delta_e_pert = (delta_e - delta_e_trim);
delta_a_pert = (delta_a - delta_a_trim);

% Nondimensionalize rates
p_hat = b * p / (2 * V_trim);
q_hat = c_bar * q / (2 * V_trim);
r_hat = b * r / (2 * V_trim);

% Calculate aerodynamic coefficients
c_D = c_D_0 + c_D_alpha * alpha * c_D_alpha_sq * alpha^2 + c_D_q_hat * q_hat + c_D_delta_e * delta_e_pert + c_D_delta_e_alpha * alpha * delta_e_pert;
c_L = c_L_0 + c_L_alpha * alpha + c_L_alpha_sq * alpha^2 + c_L_delta_e * delta_e_pert;
c_m = c_m_0 + c_m_alpha * alpha + c_m_q_hat * q_hat + c_m_delta_e * delta_e_pert;

c_Y = c_Y_0 + c_Y_beta * beta + c_Y_p_hat * p_hat + c_Y_delta_a * delta_a_pert + c_Y_delta_r * delta_r;
c_l = c_l_0 + c_l_beta * beta + c_l_p_hat * p_hat + c_l_r_hat * r_hat + c_l_delta_a * delta_a_pert;
c_n = c_n_0 + c_n_beta * beta + c_n_p_hat * p_hat + c_n_r_hat * r_hat + c_n_delta_r * delta_r;

% Calculate forces and moments
D = q_bar * S * c_D;
L = q_bar * S * c_L;
m = q_bar * S * c_bar * c_m;
X = -cos(alpha) * D + sin(alpha) * L;
Z = -sin(alpha) * D - cos(alpha) * L;

Y = q_bar * S * c_Y;
l = q_bar * S * b * c_l;
n = q_bar * S * b * c_n;

% Propeller force
T = rho * D_FW^4 * c_T_FW * delta_t;

% Dynamics
phi_dot = p + tan(theta) * (q * sin(theta) + r * cos(phi));
theta_dot = q * cos(phi) - r * sin(phi);

f_x = X + T - mass*g * sin(theta);
f_y = Y + mass*g * sin(phi) * cos(theta);
f_z = Z + mass*g * cos(phi) * cos(theta);

u_dot = r*v - q*w + (1/mass) * (f_x);
v_dot = p*w - r*u + (1/mass) * (f_y);
w_dot = q*u - p*v + (1/mass) * (f_z);

p_dot = gam(1)*p*q - gam(2)*q*r + gam(3)*l + gam(4)*n;
q_dot = gam(5)*p*r - gam(6)*(p^2 - r^2) + (1/J_yy) * m;
r_dot = gam(7)*p*q - gam(1)*q*r + gam(4)*l + gam(8)*n;

% Collect dynamics
f_lon = [u_dot;
         w_dot;
         q_dot
         theta_dot];
     
f_lat = [v_dot;
         p_dot;
         r_dot;
         phi_dot];

% Linearize around trim
vars = [u v w p q r phi theta delta_a delta_e delta_r delta_t];
trim = [u_trim 0 w_trim 0 0 0 0 theta_trim delta_a_trim delta_e_trim 0 delta_t_trim];

A_lon = double(subs(jacobian(f_lon,[u w q theta]),vars,trim));
B_lon = double(subs(jacobian(f_lon,[delta_e delta_t]),vars,trim));
C = eye(4);
ss_lon = ss(A_lon, B_lon, C, [], ...
    'OutputName',{'u', 'w', 'q', 'theta'},...
    'InputName',{'delta_e', 'delta_t'});

A_lat = double(subs(jacobian(f_lat,[v p r phi]),vars,trim));
B_lat = double(subs(jacobian(f_lat,[delta_a delta_r]),vars,trim));
ss_lat = ss(A_lat, B_lat, C, [],...
    'OutputName',{'v', 'p', 'r', 'phi'},...
    'InputName',{'delta_a', 'delta_r'});