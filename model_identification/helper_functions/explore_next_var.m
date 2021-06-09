function [] = explore_next_var(z, variables_to_test, variables_to_test_str)
    disp("Testing new terms:")
    r = calculate_partial_correlation(variables_to_test, z);
    disp("r: " + variables_to_test_str);
    fprintf(['   ' repmat('%5.3f ',1,length(r)) '\n'], r);
    disp(" ")
end