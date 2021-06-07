%%% Inertia properties
% Mass
mass_g = 12140.00;
mass_kg = mass_g * 1e-3;

cg_position_from_front_mm = 494.31; 

% Moment of Inertia around NED body frame
% Calculated from 3D CAD file
% units: grams * square millimeters;
Jxx = 924440800.82;	Jxy = 4810.21;	Jxz = 127880446.25;
Jyx = 4810.21;	Jyy = 1070196067.65;	Jyz = 31829.85;
Jzx = 127880446.25;	Jzy = 31829.85;	Jzz = 1888642369.68;
            
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