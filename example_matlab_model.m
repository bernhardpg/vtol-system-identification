function [dx, y] = twotanks_m(t, x, u, A1, k, a1, g, A2, a2, varargin)
%TWOTANKS_M  A two tank system.

%   Copyright 2005-2006 The MathWorks, Inc.

% Output equation.
y = x(2);                                              % Water level, lower tank.

% State equations.
dx = [1/A1*(k*u(1)-a1*sqrt(2*g*x(1)));             ... % Water level, upper tank.
      1/A2*(a1*sqrt(2*g*x(1))-a2*sqrt(2*g*x(2)))   ... % Water level, lower tank.
     ];