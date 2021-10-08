clc; clear all; close all;

% Import an STL mesh, returning a PATCH-compatible face-vertex structure
model = stlread('3d_model/output/babyshark.stl');

V = model.vertices;
F = model.faces;

% Move the aircraft to the origin
temp_max = max(V);
temp_min = min(V);
ranges = abs(temp_max - temp_min);
translation = ranges / 2;
V = V - translation;

% Rotate the aircraft to initial position, with positive x-axis out of nose
phi = pi/2;
theta = 0;
psi = pi/2;

Rx = [1 0 0;
      0 cos(phi) -sin(phi);
      0 sin(phi) cos(phi)];
  
Ry = [cos(theta) 0 sin(theta);
      0 1 0;
      -sin(theta) 0 cos(theta)];
           
Rz = [cos(psi), -sin(psi), 0 ;
      sin(psi), cos(psi), 0 ;
             0,         0, 1 ];
          
V = V * Rx';
V = V * Ry';
V = V * Rz';

% Scale the aircraft to the correct size
wingspan_m = 2.5;
temp_max = max(V);
temp_min = min(V);
ranges = abs(temp_max - temp_min);
y_range = ranges(2);
scaling_factor = y_range / wingspan_m;
V = V / scaling_factor;

patch('Faces', F, 'Vertices', V, ...
         'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15); hold on
     
     
translation = [2.5 0 0];

V2 = V + translation;
patch('Faces', F, 'Vertices', V2, ...
         'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15); hold on

% Plot settings

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
view([30 30]);
xlabel('x')
ylabel('y')
zlabel('z')