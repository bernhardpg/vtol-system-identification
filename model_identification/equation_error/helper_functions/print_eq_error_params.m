function [] = print_eq_error_params(coeff_name, th_hat, th_names)
    num_params = length(th_hat);
    for i = 1:num_params
        disp(coeff_name + "_" + th_names(i) + " = " + th_hat(i) + ";");
    end
    disp(" ");
end