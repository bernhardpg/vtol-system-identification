%%% Lift and drag properties
% Found with calculate_static_curves.m

% Linear lift model
% c_L = c_L0 + c_Lalpha * AoA
% Quadratic drag model
% c_D = c_Dp + c_Dalpha * AoA.^2

c_L0 = 0.83958;
c_Lalpha = 1.9228;

c_Dp = 0.042593;
c_Dalpha = 0.91101;