clc; clear all; close all;

% Describe inputs
InputName = {'Voltage applied to motor moving arm 1'; ...
             'Voltage applied to motor moving arm 2'; ...
             'Voltage applied to motor moving arm 3'};
InputUnit = {'V'; 'V'; 'V'};

% Describe states
StateName  = {'\Theta_1'; ...   % Relative angle between fundament and arm 1
              '\Theta_2'; ...   % Relative angle between arm 1 and arm 2
              '\Theta_3'; ...   % Relative angle between arm 2 and arm 3
              'Vel_1'; ...     % Relative velocity between fundament and arm 1
              'Vel_2'; ...     % Relative velocity between arm 1 and arm 2
              'Vel_3'...       % Relative velocity between arm 2 and arm 3
              };
StateUnit  = {'rad'; 'rad'; 'rad'; 'rad/s'; 'rad/s'; 'rad/s'};
OutputName = StateName(1:3);
OutputUnit = StateUnit(1:3);

% Describe parameters
ParName  = {'Gravity constant';                          ... % g, scalar.
            'Voltage-force constant of motor';           ... % Fc, 3-by-1 vector, for motor 1, 2, 3.
            'Gear ratio of motor';                       ... % r, 3-by-1 vector, for motor 1, 2, 3.
            'Moment of inertia of motor';                ... % Im, 3-by-1 vector, for motor 1, 2, 3.
            'Mass of arm 2 and 3 (incl. tool)';          ... % m, 2-by-1 vector, for arm 2 and 3.
            'Point mass of payload';                     ... % pl, scalar.
            'Length of arm 2 and 3 (incl. tool)';        ... % L, 2-by-1 vector, for arm 2 and 3.
            'Center of mass coordinates of arm 2 and 3'; ... % com, 2-by-2 matrix, 1:st column for arm 2 (x-,z-coord), 2:nd for arm 3.
            'Moment of inertia arm 1, element (3,3)';    ... % Ia1, scalar.
            'Moment of inertia arm 2 and 3'              ... % Ia, 4-by-2 matrix. 1:st column for arm 2, 2:nd for arm 3;
                                                         ... % column elements: 1: (1,1); 2: (2,2); 3: (3,3); 4: (1,3) and (3,1).
           };
ParUnit  = {'m/s^2'; 'N*m/V'; ''; 'kg*m^2'; 'kg'; 'kg'; 'm'; 'm'; 'kg*m^2'; 'kg*m^2'};
ParValue = {9.81; [-126; 252; 72]; [-105; 210; 60]; [1.3e-3; 1.3e-3; 1.3e-3]; ...
            [56.5; 60.3]; 10; [0.5; 0.98]; [0.172 0.028; 0.205 0.202];        ...
            1.16; [2.58 11.0; 2.73 8.0; 0.64 0.80; -0.46 0.50]};
ParMin   = {eps(0); -Inf(3, 1); -Inf(3, 1); eps(0)*ones(3, 1); [40; 40]; ...
            eps(0); eps(0)*ones(2, 1); eps(0)*ones(2); -Inf; -Inf(4, 2)};
ParMax   = Inf;   % No maximum constraint.

% Create grey-box nonliner identification model
FileName      = 'robot_c_copy';               % File describing the model structure.
Order         = [3 3 6];                 % Model orders [ny nu nx].
Parameters    = struct('Name', ParName, 'Unit', ParUnit, 'Value', ParValue, ...
                       'Minimum', ParMin, 'Maximum', ParMax, 'Fixed', false);
InitialStates = struct('Name', StateName, 'Unit', StateUnit, 'Value', 0, ...
                       'Minimum', -Inf, 'Maximum', Inf, 'Fixed', true);
Ts            = 0;                       % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts,     ...
                'Name', 'Manutec r3 robot', 'InputName', InputName, ...
                'InputUnit', InputUnit, 'OutputName', OutputName,   ...
                'OutputUnit', OutputUnit,  'TimeUnit', 's');
            
% Load input output data            
load(fullfile(matlabroot, 'toolbox', 'ident', 'iddemos', 'data', 'robotdata'));

% Create id-data object containing all experiments
z = merge(iddata(y1, u1, 0.02), iddata(y2, u2, 0.02));
z.Name = 'Manutec r3 robot';
z.InputName = nlgr.InputName;
z.InputUnit = nlgr.InputUnit;
z.OutputName = nlgr.OutputName;
z.OutputUnit = nlgr.OutputUnit;
z.Tstart = 0;
z.TimeUnit = 's';

% Simulate using found parameter values
nlgr.SimulationOptions.RelTol = 1e-5;
compare(z, nlgr);

%%
% Only estimate the last four parameters first
for k = 1:size(nlgr, 'Npo')-1   % Fix all parameters of the first 9 parameter objects.
    nlgr.Parameters(k).Fixed = true;
end
nlgr.Parameters(end).Fixed(:, 1) = true;   % Fix the moment of inertia parameters for arm 2.

% Estimate parameters
opt = nlgreyestOptions('SearchMethod', 'lm', 'Display', 'on');
nlgr = nlgreyest(z, nlgr, opt);

% Performance
compare(z, nlgr);

%% Print results
disp('   True     Estimated parameter vector');
ptrue = [9.81; -126; 252; 72; -105; 210; 60; 1.3e-3; 1.3e-3; 1.3e-3; ...
         56.5; 60.3; 10; 0.5; 0.98; 0.172; 0.205; 0.028; 0.202; 1.16; ...
         2.58; 2.73; 0.64; -0.46; 5.41; 5.60; 0.39; 0.33];
fprintf('   %1.3f    %1.3f\n', ptrue(25), nlgr.Parameters(end).Value(1, 2));
fprintf('   %1.3f    %1.3f\n', ptrue(26), nlgr.Parameters(end).Value(2, 2));
fprintf('   %1.3f    %1.3f\n', ptrue(27), nlgr.Parameters(end).Value(3, 2));
fprintf('   %1.3f    %1.3f\n', ptrue(28), nlgr.Parameters(end).Value(4, 2));

%
present(nlgr);