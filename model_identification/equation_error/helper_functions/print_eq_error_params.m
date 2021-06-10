function [] = print_eq_error_params(coeff_name, th_hat, th_names)
    num_params = length(th_hat);
    for i = 1:num_params
        if i == 1
            disp(coeff_name + "_0 = " + th_hat(i) + ";");
        else
            disp(coeff_name + "_" + th_names(i - 1) + " = " + th_hat(i) + ";");
        end
    end
    disp(" ");
end