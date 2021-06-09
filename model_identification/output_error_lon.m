clc; clear all; close all;

% Maneuver settings
maneuver_type = "pitch_211";
data_path = "data/aggregated_data/" + maneuver_type + "/";

% Load training data
data = readmatrix(data_path + "data_train.csv");
maneuver_start_indices = readmatrix(data_path + "maneuver_start_indices_train.csv");

[t_seq, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, p_dot, q_dot, r_dot, delta_a, delta_e, delta_r, n_p, c_X, c_Y, c_Z, c_l, c_m, c_n]...
    = extract_variables_from_data(data);
dt = t_seq(2) - t_seq(1);

maneuver_indices = [maneuver_start_indices; length(t_seq)]; % Add end index to this

% Explanatory variables for equation-error
[p_hat, q_hat, r_hat, u_hat, v_hat, w_hat] = calc_explanatory_vars(p, q, r, u, v, w);

% Create a common time vector for plotting of all maneuvers
t_plot = 0:dt:length(t_seq)*dt-dt;


%% From lon equation-error

% Load constants
aircraft_properties;
const_params = [rho, mass_kg, g, wingspan_m, mean_aerodynamic_chord_m, planform_sqm, V_nom,...
     gam_1, gam_2, gam_3, gam_4, gam_5, gam_6, gam_7, gam_8, J_yy,...
     ]';

% Initial guesses from equation-error
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

x0 = [
     c_X_0, c_X_u, c_X_w, c_X_w_sq, c_X_q, c_X_n_p,...
     c_Z_0, c_Z_w, c_Z_w_sq, c_Z_delta_e,...
     c_m_0, c_m_w, c_m_q, c_m_delta_e,...
     ];

% Variable bounds
allowed_param_change = 0.2;
LB = min([x0 * (1 - allowed_param_change); x0 * (1 + allowed_param_change)]);
UB = max([x0 * (1 - allowed_param_change); x0 * (1 + allowed_param_change)]);

% Collect data for opt problem
y_lon = [theta q u w];
y_lat = [phi, psi, p, r, v];
input = [delta_a, delta_e, delta_r, n_p];

if 0
    for maneuver_i = 1
        [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_m, delta_e_m, delta_r_m, n_p_m]...
         = get_maneuver_data(maneuver_i, maneuver_start_indices, t_seq, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a, delta_e, delta_r, n_p);

        all_params = [const_params;
                      x0'];

        % Integration interval
        tspan = t_m(1):dt:t_m(end);
        i = 50;
        y0 = [theta_m(i) q_m(i) u_m(i) w_m(i)];

        input_seq_m = [delta_a_m delta_e_m delta_r_m n_p_m];
        lat_state_seq_m = [phi_m, psi_m, p_m, r_m, v_m];
        test_matrix = [t_m input_seq_m lat_state_seq_m];

        tic
        [t_pred, y_pred] = ode45(@(t,y) lon_dynamics_c(t, y, test_matrix, all_params), tspan, y0);
        toc
        
        plot_maneuver("maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_e_m, delta_r_m, n_p_m,...
            t_m, y_pred,...
            false, true, "");
    end
end

tic
cost = cost_fn_lon(x0, dt, t_seq, y_lon, y_lat, input, const_params, maneuver_indices);
toc

% Optimization settings
rng default % For reproducibility
FitnessFunction = @(x) cost_fn_lon(x, dt, t_seq, y_lon, y_lat, input, const_params, maneuver_indices);
numberOfVariables = length(x0);
options = optimoptions('ga',...%'UseParallel', true, 'UseVectorized', false,...
    'PlotFcn',@gaplotbestf,'Display','iter');
% Set initial guess
options.InitialPopulationMatrix = x0;

% Solve optimization problem
[x,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],LB,UB,[],options);
writematrix(x, "lon_params.txt")



