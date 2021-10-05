%%% Physical constants
rho = 1.225; % kg / m^3 (air density at sea level)
g = 9.81; % m / s^2

%%%%%%
%%% Aircraft properties
%%%%%%
inertia_properties;

%%% Wings
% Planform
% From CAD 3D model
half_tail_planform_sqmm = 67225.12;
one_wing_planform_sqmm = 263626.10;
planform_sqmm = 2 * half_tail_planform_sqmm + 2 * one_wing_planform_sqmm;
planform_sqm = planform_sqmm * 1e-3^2;

% Mean chord length
% From CAD 3D model
chord_length_body_side_mm = 300;
chord_length_wingtip_side_mm = 195.4;
mean_aerodynamic_chord_m = 0.242; % Found through wing approximation in python script

% Wingspan
wingspan_mm = 2500;
wingspan_m = wingspan_mm / 1e3;

% Aspect ratio
aspect_ratio = wingspan_m^2 / planform_sqm;

%%% Motor placement
% From CAD 3D model
% Motor placements in body frame (z-axis is pointing down)
r_t1_B_mm = [353.2; 400.0; -52.0];
r_t2_B_mm = [-446.8; -400.0; -52.0];
r_t3_B_mm = [353.2; -400.0; -52.0];
r_t4_B_mm = [-446.8; 400.0; -52.0];
r_t1_B = r_t1_B_mm / 1e3;
r_t2_B = r_t1_B_mm / 1e3;
r_t3_B = r_t1_B_mm / 1e3;
r_t4_B = r_t1_B_mm / 1e3;

% Cruise conditions
V_nom = 21; % Nominal airspeed, m/s
alpha_nom = 3 / 180 * pi;
u_nom = V_nom / (sqrt(1 + tan(alpha_nom)^2));
w_nom = sqrt(V_nom^2 - u_nom^2);
delta_e_nom = -0.0985;
delta_a_nom = 0.0529;
delta_r_nom = 0;

% Load control surface properties
control_surface_constants;
fixedwing_propeller_constants;
multirotor_propeller_constants;