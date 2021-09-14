function [x_smoothed] = smooth_signal(x)
    order = 5;
    framelen = 11;
    x_smoothed = sgolayfilt(x,order,framelen);
end