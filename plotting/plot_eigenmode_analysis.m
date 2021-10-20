clc; clear all; close all;
linear_model;
plot_settings;

figure
pzmap(ss_lon);
h = findobj(gca, 'type', 'line');
set(h, 'markersize', 14, 'linewidth', 1.5)
xlim([-12 12])
axis equal
p = pole(ss_lon);
title("Longitudinal Model", 'FontSize', font_size_large, 'interpreter', 'latex')
text(real(p(1)) - 1, imag(p(1)) - 1, 'Short-Period')
text(real(p(3)) - 1, imag(p(3)) + 1, 'Phugoid')

figure
pzmap(ss_lat);
h = findobj(gca, 'type', 'line');
set(h, 'markersize', 14, 'linewidth', 1.5)
xlim([-12 12])
axis equal
p = pole(ss_lat);
title("Lateral-Directional Model", 'FontSize', font_size_large, 'interpreter', 'latex')
text(real(p(1)), imag(p(1)) - 1, 'Roll')
text(real(p(2)) - 1, imag(p(2)) - 1, 'Dutch Roll')
text(real(p(4)), imag(p(4)) - 1, 'Spiral')