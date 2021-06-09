function [t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a_sp, delta_e_sp, delta_r_sp, delta_a, delta_e, delta_r, n_p, c_X, c_Y, c_Z, c_l, c_m, c_n]...
    = extract_variables_from_data(data)
    t = data(:,1);
    phi = data(:,2);
    theta = data(:,3);
    psi = data(:,4);
    p = data(:,5);
    q = data(:,6);
    r = data(:,7);
    u = data(:,8);
    v = data(:,9);
    w = data(:,10);
    a_x = data(:,11);
    a_y = data(:,12);
    a_z = data(:,13);
    p_dot = data(:,14);
    q_dot = data(:,15);
    r_dot = data(:,16);
    delta_a_sp = data(:,17);
    delta_e_sp = data(:,18);
    delta_r_sp = data(:,19);
    delta_a = data(:,20);
    delta_e = data(:,21);
    delta_r = data(:,22);
    n_p = data(:,23);
    c_X = data(:,24);
    c_Y = data(:,25);
    c_Z = data(:,26);
    c_l = data(:,27);
    c_m = data(:,28);
    c_n = data(:,29);
end