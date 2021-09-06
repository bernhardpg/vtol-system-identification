% Mass
% see calc_mass.m
mass_kg = 12.140;
cg_position_from_front_m = 0.494;

% Moment of inertias
% see calc_moment_of_inertias.m
J = [0.7316 0.0000 0.1277;
     0.0000 1.0664 0.0000;
     0.1277 0.0000 1.6917];
 
J_xx = J(1,1);
J_xy = J(1,2);
J_xz = J(1,3);
J_yx = J(2,1);
J_yy = J(2,2);
J_yz = J(2,3);
J_zx = J(3,1);
J_zy = J(3,2);
J_zz = J(3,3);

% Convenience constants
gam = [0.1419    0.8795    1.3851    0.1045    0.9003    0.1197   -0.1872    0.5990];