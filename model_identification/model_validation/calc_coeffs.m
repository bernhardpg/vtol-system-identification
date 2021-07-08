function [c_D, c_L, c_m] = calc_coeffs(lon_params, aoa_alpha, q_hat, delta_e)
    lon_params = num2cell(lon_params);
    [c_D_0 c_D_alpha c_D_alpha_sq c_D_q c_D_delta_e c_L_0 c_L_alpha c_L_alpha_sq c_L_q c_L_delta_e c_m_0 c_m_alpha c_m_q c_m_delta_e c_m_delta_e_sq] = lon_params{:};
    c_D = c_D_0 + c_D_alpha * aoa_alpha + c_D_alpha_sq * aoa_alpha.^2 + c_D_q * q_hat + c_D_delta_e * delta_e;
    c_L = c_L_0 + c_L_alpha * aoa_alpha + c_L_delta_e * delta_e + c_L_alpha_sq * aoa_alpha.^2 + c_L_q * q_hat;
    c_m = c_m_0 + c_m_alpha * aoa_alpha + c_m_q * q_hat + c_m_delta_e * delta_e + c_m_delta_e_sq * delta_e.^2;
end