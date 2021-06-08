clc; clear all; close all;

% Maneuver settings
maneuver_type = "pitch_211";
data_path = "data/aggregated_data/" + maneuver_type + "/";

% Load data
[t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a, delta_e, delta_r, n_p, c_X, c_Y, c_Z, c_l, c_m, c_n, maneuver_start_indices]...
    = load_variables_from_file(data_path);
dt = t(2) - t(1);

% Explanatory variables for equation-error
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);

% Create a time vector for plotting of all maneuvers
t_plot = 0:dt:length(t)*dt - dt;


%% From lon equation-error

c_X_0 = 0.1276;
c_X_u = -0.3511;
c_X_w = 0.1757;
c_X_w_sq = 2.7162;
c_X_q = -3.2355;
c_X_n_p = 0.0025;
c_Z_0 = -0.5322;
c_Z_w = -5.1945;
c_Z_w_sq = 5.7071;
c_Z_delta_e = -0.3440;
c_m_0 = 0.0266;
c_m_w = -1.0317;
c_m_q = 1.0616;
c_m_delta_e = -0.3329;

aircraft_properties;
const_params = [rho, mass_kg, g, wingspan_m, mean_aerodynamic_chord_m, planform_sqm, V_nom,...
     gam_1, gam_2, gam_3, gam_4, gam_5, gam_6, gam_7, gam_8, J_yy,...
     ]';
x0 = [
     c_X_0, c_X_u, c_X_w, c_X_w_sq, c_X_q, c_X_n_p,...
     c_Z_0, c_Z_w, c_Z_w_sq, c_Z_delta_e,...
     c_m_0, c_m_w, c_m_q, c_m_delta_e,...
    ];

LB = min([x0 * 0.8; x0 * 1.2]);
UB = max([x0 * 0.8; x0 * 1.2]);


for maneuver_i = 5
    [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_m, delta_e_m, delta_r_m, n_p_m]...
     = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a, delta_e, delta_r, n_p);
    
    % Get inputs
    input_seq = [delta_a_m, delta_e_m, delta_r_m, n_p_m];
    % Take lat states as they are
    lat_state_seq = [phi_m, psi_m, p_m, r_m, v_m];
    y = [theta_m q_m u_m w_m];
    
    rng default % For reproducibility
    FitnessFunction = @(x) cost_fn(x, t_m, y, input_seq, lat_state_seq, const_params);
    numberOfVariables = length(x0);
    options = optimoptions('ga','UseParallel', true, 'UseVectorized', false,...
        'PlotFcn',@gaplotbestf,'Display','iter');
    % Set initial guess
    options.InitialPopulationMatrix = x0;
    %options = optimoptions(@ga,'MutationFcn',@mutationadaptfeasible);

    [x,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],LB,UB,[],options);
        
    all_params = [const_params;
                  x'];
    
    % Integration interval
    tspan = t_m(1):dt:t_m(end);
    y0 = [theta_m(1) q_m(1) u_m(1) w_m(1)];
    
    y_pred = ode5(@(t,y) aircraft_dynamics_lon(t, y, t_m, input_seq, lat_state_seq, all_params), tspan, y0);
        
    plot_maneuver("maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_e_m, delta_r_m, n_p_m,...
        t_m, y_pred,...
        false, true, "");
end

