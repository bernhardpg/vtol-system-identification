% Mass
% see calc_mass.m
mass_kg = 12.140;
cg_position_from_front_mm = 494.31;

% Moment of inertias
% see calc_moment_of_inertias.m
J = [0.7316 0.0000 0.1277;
     0.0000 1.0664 0.0000;
     0.1277 0.0000 1.6917];

% Convenience constants
gam = [0.1419    0.8795    1.3851    0.1045    0.9003    0.1197   -0.1872    0.5990];