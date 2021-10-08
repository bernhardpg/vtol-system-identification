clc; clear all; close all;

amplitude = 15;

plot_settings;
plot([0 1.99 2 3.99 4 4.99 5 5.99 6 8], [0 0 amplitude amplitude -amplitude -amplitude amplitude amplitude 0 0])
ylim([-amplitude*2 amplitude*2])

ylabel("Deflection $[^\circ]$", 'interpreter', 'latex', 'FontSize', font_size)
set(gca,'FontSize', font_size_small)
xlabel("Time $[s]$", 'interpreter', 'latex', 'FontSize', font_size)

title("2-1-1 Signal", 'FontSize', font_size_large, 'interpreter', 'latex')