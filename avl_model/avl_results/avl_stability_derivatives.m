% NOTE! These values are outdated.



%  Vortex Lattice Output -- Total Forces
% 
%  Configuration: VTOL                                                        
%      # Surfaces =   6
%      # Strips   =  66
%      # Vortices = 608
% 
%   Sref = 0.66170       Cref = 0.24200       Bref =  2.5000    
%   Xref = 0.49400       Yref =  0.0000       Zref =-0.43000E-01
% 
%  Standard axis orientation,  X fwd, Z down         
% 
%  Run case: cruise                                  
% 
%   Alpha =   0.00000     pb/2V =  -0.00000     p'b/2V =  -0.00000
%   Beta  =   0.00000     qc/2V =  -0.00000
%   Mach  =     0.000     rb/2V =   0.00000     r'b/2V =   0.00000
% 
%   CXtot =  -0.01014     Cltot =   0.00000     Cl'tot =   0.00000
%   CYtot =   0.00000     Cmtot =  -0.00000
%   CZtot =  -0.46517     Cntot =   0.00000     Cn'tot =   0.00000
% 
%   CLtot =   0.46517
%   CDtot =   0.01014
%   CDvis =   0.00000     CDind =   0.01014
%   CLff  =   0.46528     CDff  =   0.01103    | Trefftz
%   CYff  =   0.00000         e =    0.6613    | Plane  
%  
%    aileron         =   0.00000
%    elevator        = -11.04762
%    rudder          =   0.00000
%  
%  ---------------------------------------------------------------
% 
%  Stability-axis derivatives...
% 
%                              alpha                beta
%                   ----------------    ----------------
%  z' force CL |    CLa =   4.753189    CLb =  -0.000000
%  y  force CY |    CYa =   0.000000    CYb =  -0.354354
%  x' mom.  Cl'|    Cla =  -0.000000    Clb =  -0.019794
%  y  mom.  Cm |    Cma =  -1.455466    Cmb =  -0.000000
%  z' mom.  Cn'|    Cna =   0.000000    Cnb =   0.114003
% 
%                      roll rate  p'      pitch rate  q'        yaw rate  r'
%                   ----------------    ----------------    ----------------
%  z' force CL |    CLp =   0.000000    CLq =   8.470922    CLr =  -0.000000
%  y  force CY |    CYp =   0.088506    CYq =   0.000000    CYr =   0.276492
%  x' mom.  Cl'|    Clp =  -0.473939    Clq =  -0.000000    Clr =   0.119358
%  y  mom.  Cm |    Cmp =  -0.000000    Cmq = -13.036248    Cmr =   0.000000
%  z' mom.  Cn'|    Cnp =  -0.053470    Cnq =  -0.000000    Cnr =  -0.093340
% 
%                   aileron      d1     elevator     d2     rudder       d3 
%                   ----------------    ----------------    ----------------
%  z' force CL |   CLd1 =  -0.000000   CLd2 =   0.006305   CLd3 =   0.000000
%  y  force CY |   CYd1 =  -0.000577   CYd2 =   0.000000   CYd3 =   0.005591
%  x' mom.  Cl'|   Cld1 =   0.004960   Cld2 =  -0.000000   Cld3 =  -0.000141
%  y  mom.  Cm |   Cmd1 =   0.000000   Cmd2 =  -0.021331   Cmd3 =   0.000000
%  z' mom.  Cn'|   Cnd1 =   0.000147   Cnd2 =   0.000000   Cnd3 =  -0.001963
%  Trefftz drag| CDffd1 =  -0.000000 CDffd2 =  -0.000461 CDffd3 =   0.000000
%  span eff.   |    ed1 =  -0.000000    ed2 =   0.045472    ed3 =  -0.000000
%  
%  
% 
%  Neutral point  Xnp =   0.568102
% 
%  Clb Cnr / Clr Cnb  =   0.135776    (  > 1 if spirally stable )
% 


% Directly from AVL
avl_c_L_0 = 0.46517; % see zero_aoa_flight.txt

% See stability_derivatives_trim.txt
% Longitudinal coeffs
avl_c_L_alpha = 4.753189;
avl_c_L_q = 8.470922;
avl_c_L_delta_e_deg = 0.006305;
avl_c_L_delta_e = 0.3612498898;

avl_c_D_delta_e_deg = -0.000461; % Treftz drag
avl_c_D_delta_e = 0.02641335436;

avl_c_m_alpha = -1.455466;
avl_c_m_q = -13.036248;
avl_c_m_delta_e_deg = -0.021331;
avl_c_m_delta_e = -1.2221762728;

% Lateral coeffs
avl_c_Y_beta =  -0.354354;
avl_c_Y_p = 0.088506;
avl_c_Y_r = 0.276492;
avl_c_Y_delta_a_deg = -0.000577;
avl_c_Y_delta_a = -0.03305966478;
avl_c_Y_delta_r_deg = 0.005591;
avl_c_Y_delta_r = 0.3203407033;

avl_c_l_beta = -0.019794;
avl_c_l_p = -0.473939;
avl_c_l_r = 0.119358;
avl_c_l_delta_a_deg = 0.004960;
avl_c_l_delta_a = 0.2841870664;
avl_c_l_delta_r_deg = -0.000141;
avl_c_l_delta_r = -0.008078704911;

avl_c_n_beta = 0.114003;
avl_c_n_p = -0.053470;
avl_c_n_r = -0.093340;
avl_c_n_delta_a_deg = 0.000147;
avl_c_n_delta_a = 0.008422479588;
avl_c_n_delta_r_deg = -0.001963;
avl_c_n_delta_r = -0.1124716152;


% Non-dimensionalize ang rate derivatives is not needed
% NOTE: The values from AVL are already dimensionless. See this post:
% https://www.researchgate.net/post/Does-anyone-know-if-the-derivatives-output-from-AVL-Athena-Vortex-Lattice-are-dimensional-or-dimensionless

