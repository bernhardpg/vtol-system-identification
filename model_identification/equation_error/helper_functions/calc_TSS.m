function [TSS] = calc_TSS(z)
   % Calculate total Sum of Squares
    z_bar = mean(z);
    TSS = (z - z_bar)' * (z - z_bar);
end