function [x_smoothed] = smooth_signal(x)
    order = 10;
    framelen = 51;
    x_smoothed = sgolayfilt(x,order,framelen);
end