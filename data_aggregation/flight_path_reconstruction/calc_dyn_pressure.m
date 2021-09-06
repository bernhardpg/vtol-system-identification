function [dyn_pressure] = calc_dyn_pressure(u, v, w)
    airframe_static_properties; % to get rho

    V = sqrt(u .^ 2 + v .^ 2 + w .^ 2);
    dyn_pressure = 0.5 * rho * V .^ 2;
end