%%% Lift and drag properties
% Found with calculate_static_curves.m

c_L_0 = 0.4511;
c_L_alpha = 3.4542;
c_D_p = 0.0729;
c_D_alpha = 0.3501;
c_D_alpha_sq = 0.2621;

% Using 7410 datapoints, filtering on high pitch rates, strong rudder
% movement, or too high NED-frame velocity downwards

% Performance of these params:
% c_L_rmse =
% 
%     0.1485
% 
% 
% c_L_r_sq =
% 
%     0.6859
% 
% 
% c_D_rmse =
% 
%     0.0186
% 
% 
% c_D_r_sq =
% 
%     0.6415
