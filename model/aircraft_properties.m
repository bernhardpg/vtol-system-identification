%%%%%%
%%% Aircraft properties
%%%%%%

%%% Mass properties
% Mass
mass_g = 12630.00;
mass_kg = mass_g * 1e-3;

% Moment of Inertia around NED body frame
% Calculated from 3D CAD file
% units: grams * square millimeters;
Jxx = 1016069849.02;	Jxy = 20625.57;	Jxz = 131897698.13;
Jyx = 20625.57;	Jyy = 1112626767.24;	Jyz = 33212.68;
Jzx = 131897698.13;	Jzy = 33212.68;	Jzz = 2019891007.45;

J_grams_sqmm = [Jxx Jxy Jxz;
                Jyx Jyy Jyz
                Jzx Jzy Jzz];

grams_sqmm_to_kg_sqm = 1e-3 * 1e-3^2;
J = J_grams_sqmm * grams_sqmm_to_kg_sqm;

% Redefine these with correct units (this is bad coding style)
Jxx = J(1,1);
Jxy = J(1,2);
Jxz = J(1,3);
Jyx = J(2,1);
Jyy = J(2,2);
Jyz = J(2,3);
Jzx = J(3,1);
Jzy = J(3,2);
Jzz = J(3,3);

lam_det = Jxx * Jzz - Jxz ^ 2;
lam_1 = Jxz * (Jxx - Jyy + Jzz) / lam_det;
lam_2 = (Jzz * (Jzz - Jyy) + Jxz ^ 2) / lam_det;
lam_3 = Jzz / lam_det;
lam_4 = Jxz / lam_det;
lam_5 = (Jzz - Jxx) / Jyy;
lam_6 = Jxz / Jyy;
lam_7 = (Jxx * (Jxx - Jyy) + Jxz ^ 2) / lam_det;
lam_8 = Jxx / lam_det;
lam = [lam_1 lam_2 lam_3 lam_4 lam_5 lam_6 lam_7 lam_8];

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

%%% Physical constants
rho = 1.225; % kg / m^3 (air density at sea level)
g = 9.81; % m / s^2

%%% Propellers and motors
kINCH_TO_METER = 0.0254;
prop_diam_top_in_inches = 16;
prop_diam_top = prop_diam_top_in_inches * kINCH_TO_METER;
prop_diam_pusher_in_inches = 15;
prop_diam_pusher = prop_diam_pusher_in_inches * kINCH_TO_METER;
c_T_top = 0;
c_T_pusher = 2.3906e-05; % See motor_id.m for calculation off this
c_Q_top = 0;
c_Q_pusher = 0;

% Convenience constants
rho_diam_top_pwr_four = rho * prop_diam_top ^ 4; % computed once for efficiency
rho_diam_pusher_pwr_four = rho * prop_diam_pusher ^ 4; % computed once for efficiency
rho_diam_top_pwr_five = rho * prop_diam_top ^ 5; % computed once for efficiency
rho_diam_pusher_pwr_five = rho * prop_diam_pusher ^ 5; % computed once for efficiency
half_rho_planform = 0.5 * rho * planform_sqm; % computed once for efficiency
