function [c_l, c_Y, c_n] = calc_coeffs_lat(lat_params, beta, p_hat, r_hat, delta_a, delta_r)
    lat_params = num2cell(lat_params);
    [c_Y_0 c_Y_beta c_Y_p c_Y_delta_a c_Y_delta_r c_l_0 c_l_beta c_l_p c_l_r c_l_delta_a c_n_0 c_n_beta c_n_p c_n_r c_n_delta_r] = lat_params{:};
    c_l = c_l_0 + c_l_beta * beta + c_l_p * p_hat + c_l_r * r_hat + c_l_delta_a * delta_a;
    c_Y = c_Y_0 + c_Y_beta * beta + c_Y_p * p_hat + c_Y_delta_a * delta_a + c_Y_delta_r * delta_r;
    c_n = c_n_0 + c_n_beta * beta + c_n_p * p_hat + c_n_r * r_hat + c_n_delta_r * delta_r; 
end