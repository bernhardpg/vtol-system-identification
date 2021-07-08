function [R_sq] = calc_R_sq(y_hat, z)
    RSS = calc_RSS(y_hat, z);
    TSS = calc_TSS(z);
    
    % Coefficient of Determination
    R_sq = (1 - (RSS / TSS)) * 100;
end