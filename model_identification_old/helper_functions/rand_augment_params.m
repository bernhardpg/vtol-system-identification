function [params] = rand_augment_params(params_to_augment, params, percent)
    for i = params_to_augment
        a = 1 - percent / 100;
        b = 1 + percent / 100;
        rand_gain = rand_num_in_interval(a,b);
        
        old_value = params(i).Value;
        if old_value == 0
            new_value = rand_num_in_interval(-1 * percent / 100, 1 * percent / 100);
        else
            new_value = old_value * rand_gain;
        end
        
        params(i).Value = new_value;
    end
end

function [r] = rand_num_in_interval(a,b)
    r = a + (b - a) * rand(1);
end


