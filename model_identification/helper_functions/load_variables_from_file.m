function [t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a, delta_e, delta_r, n_p, c_X, c_Y, c_Z, c_l, c_m, c_n,...
    maneuver_start_indices]...
    = load_variables_from_file(data_path)

    all_data = readmatrix(data_path + "data.csv");
    maneuver_start_indices = readmatrix(data_path + "maneuver_start_indices.csv");
    t = all_data(:,1);
    phi = all_data(:,2);
    theta = all_data(:,3);
    psi = all_data(:,4);
    p = all_data(:,5);
    q = all_data(:,6);
    r = all_data(:,7);
    u = all_data(:,8);
    v = all_data(:,9);
    w = all_data(:,10);
    a_x = all_data(:,11);
    a_y = all_data(:,12);
    a_z = all_data(:,13);
    p_dot = all_data(:,14);
    q_dot = all_data(:,15);
    r_dot = all_data(:,16);
    delta_a = all_data(:,17);
    delta_e = all_data(:,18);
    delta_r = all_data(:,19);
    n_p = all_data(:,20);
    c_X = all_data(:,21);
    c_Y = all_data(:,22);
    c_Z = all_data(:,23);
    c_l = all_data(:,24);
    c_m = all_data(:,25);
    c_n = all_data(:,26);
end