A_lon = [...
   -0.1324    0.6662   -1.8897   -9.7965
   -0.8336   -3.3660   20.9296   -0.5138
    0.1748   -2.5769   -2.9689         0
         0         0    1.0000         0];

B_lon = [...
   -1.7847    0.0002
   -6.8939         0
  -26.0315         0
         0         0];

A_lat = [...
   -0.4514    2.1958  -20.9710    9.7965
   -0.8272  -10.2467    3.6705         0
    0.9525   -2.4637   -1.0659         0
         0    1.0000    0.0524         0];
     
B_lat = [...
   -5.8203    3.1047
   75.8761   -2.7198
    5.7245  -15.5898
         0         0];
     
C = eye(4);
ss_lon = ss(A_lon, B_lon, C, [], ...
    'OutputName',{'u', 'w', 'q', 'theta'},...
    'InputName',{'delta_e', 'delta_t'});

ss_lat = ss(A_lat, B_lat, C, [],...
    'OutputName',{'v', 'p', 'r', 'phi'},...
    'InputName',{'delta_a', 'delta_r'});
