clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

maneuver_types = ["roll_211"];
data_type = "train";
load_data;
load_const_params;

% Parameters to fit from roll data:
% c_Y_delta_a
% c_l_0, c_l_p, c_l_r, c_l_delta_a
% c_n_p, c_n_delta_a
% 
% Params to fit from yaw data:
% c_Y_0, c_Y_beta, c_Y_p, c_Y_r, c_Y_delta_r
% c_l_beta, c_l_delta_r,
% c_n_0, c_n_beta, c_n_r, c_n_delta_r

% All parameters:
% x0_lat = [c_Y_0 c_Y_beta c_Y_p c_Y_r c_Y_delta_a c_Y_delta_r...
%           c_l_0 c_l_beta c_l_p c_l_r c_l_delta_a c_l_delta_r...
%           c_n_0 c_n_beta c_n_p c_n_r c_n_delta_a c_n_delta_r];

%step_type = "1_roll";
%step_type = "2_yaw";
step_type = "3_roll";

% First fit rolling parameters from roll data
if step_type == "1_roll"
    
    % Load initial values from equation-error
    equation_error_results_lat;
    
    x0 = [c_Y_delta_a...
          c_l_0 c_l_p c_l_r c_l_delta_a...
          c_n_p c_n_delta_a];
    collect_all_params_func = @(x) collect_all_params_roll_initial(x);
   
% Loads params from previous step and fixes these, while letting the
% rest change
elseif step_type == "2_yaw"

    params_location = "model_identification/output_error/results/lat/";
    params_type = "lat_params_step_roll_medians";
    x_from_roll = readmatrix(params_location + params_type + ".txt");
    % Load initial values from equation-error
    equation_error_results_lat;
    x0 = [c_Y_0 c_Y_beta c_Y_p c_Y_r c_Y_delta_r...
          c_l_beta c_l_delta_r...
          c_n_0 c_n_beta c_n_r c_n_delta_r];
    
    collect_all_params_func = @(x) collect_all_params_yaw(x, x_from_roll);

% Loads params from previous step and fixes these, while letting the
% rest change
elseif step_type == "3_roll"

    params_location = "model_identification/output_error/results/lat/";
    params_type = "lat_params_step_roll_medians";
    x_from_roll = readmatrix(params_location + params_type + ".txt");
    
    params_location = "model_identification/output_error/results/lat/";
    params_type = "lat_params_step_yaw_medians";
    x_from_yaw = readmatrix(params_location + params_type + ".txt");
    % Combine results so far
    equation_error_results_lat;
    x0 = x_from_roll;
    
    collect_all_params_func = @(x) collect_all_params_roll(x, x_from_yaw);
end

      
% Collect recorded data
t_seq = t;
y_lon_seq = [theta q u w];
y_lat_seq = [phi psi p r v a_y p_dot r_dot];
input_seq = [delta_a delta_vl delta_vr n_p]; % Actuator dynamics simulated beforehand

%%
% Optimization

% Variable bounds
% Constructed such that the parameters can vary more away from 0, but not
% change sign
allowed_param_change = 0.5;
LB = min([x0 * (1 - allowed_param_change); x0 * (1 + 8 * allowed_param_change)]);
UB = max([x0 * (1 - allowed_param_change); x0 * (1 + 8 * allowed_param_change)]);

weight = diag([1 0 1 1 1 1 1 1]); % Do not weight yaw as it does not affect aircraft motion

% Opt settings
rng default % For reproducibility
numberOfVariables = length(x0);
%options = optimoptions('ga','UseParallel', true, 'UseVectorized', false,...
%    'PlotFcn',@gaplotbestf,'Display','iter');
options = optimoptions('ga','UseVectorized', false,'Display','iter','UseParallel', true);
options.InitialPopulationMatrix = x0;
options.FunctionTolerance = 1e-02;

% Run optimization problem on each maneuver separately
xs = zeros(num_maneuvers, length(x0));
for maneuver_i = 1:num_maneuvers
    disp("== Solving for maneuver " + maneuver_i + " ==");
    
    % Organize data for maneuver
    [t_seq_m, y_lon_seq_m, y_lat_seq_m, input_seq_m] = extract_man_data(maneuver_i, maneuver_indices, t_seq, y_lon_seq, y_lat_seq, input_seq);
    y0 = y_lat_seq_m(1,1:5); % do not include accelerations in initial states
    data_seq_maneuver = [t_seq_m input_seq_m y_lon_seq_m];
    N = length(data_seq_maneuver);
    residual_weight = diag(linspace(1,1,N)); % Currently we weight all time steps equally
    tspan = [t_seq_m(1) t_seq_m(end)];

    % Initial calculations for maneuver
    residuals_0 = calc_residuals_lat(y_lat_seq_m, data_seq_maneuver, const_params, collect_all_params_func(x0), dt, tspan, y0);
    R_hat_0 = diag(diag(residuals_0' * residuals_0) / N); % Assume cross-covariances to be zero
    fval_0 = cost_fn_lat(collect_all_params_func(x0), weight, residual_weight, R_hat_0, dt, data_seq_maneuver, y0, tspan, y_lat_seq_m, const_params);
    
    % Set initial values
    x = x0;
    fval = fval_0;
    R_hat = R_hat_0;
    
    reached_convergence = false;
    i = 0;
    tic
    while ~reached_convergence
        disp("Iteration " + i);
        fprintf(['R_hat: ' repmat('%2.5f  ',1,length(diag(R_hat))) '\n'], diag(R_hat))
        % Save values from last iteration
        x_prev = x;
        fval_prev = fval;
        R_hat_prev = R_hat;

        % Solve optimization problem with previous covariance matrix
        options.InitialPopulationMatrix = x;
        FitnessFunction = @(x) cost_fn_lat(collect_all_params_func(x), weight, residual_weight, R_hat_0, dt, data_seq_maneuver, y0, tspan, y_lat_seq_m, const_params);
        [x,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],LB,UB,[],options);
        
        % Calc new covariance matrix
        residuals = calc_residuals_lat(y_lat_seq_m, data_seq_maneuver, const_params, collect_all_params_func(x), dt, tspan, y0);
        N = length(data_seq_maneuver);
        R_hat = diag(diag(residuals' * residuals) / N); % Assume cross-covariances to be zero
        
        % Check for convergence
        abs_param_change = abs(x_prev - x);
        abs_fval_change = abs((fval - fval_prev) / fval_prev);
        abs_covar_change = abs(diag(R_hat_prev - R_hat)./(diag(R_hat_prev)));
        
        if all(abs_param_change < 1e-5)
            reached_convergence = true;
        end
        if abs_fval_change < 0.001
            reached_convergence = true;
        end
        if all(abs_covar_change < 0.05)
            reached_convergence = true;
        end
        i = i + 1;
    end
    toc
    
    fprintf(['abs_param_change: ' repmat('%2.4f  ',1,length(abs_param_change)) '\n'], abs_param_change)
    disp("abs_fval_change: " + abs_fval_change);
    fprintf(['abs_covar_change: ' repmat('%2.4f  ',1,length(abs_covar_change)) '\n'], abs_covar_change)
    
    xs(maneuver_i,:) = x;
    
    writematrix(xs, "lat_params_step_2_roll.txt")
end

display("Finished running output-error");

%% Check result
maneuver_types = "roll_211";
data_type = "train";
load_data;
x_lat = x;

all_params = [const_params;
              x'];
show_plot = true;
save_plot = false;
plot_output_location = "";

% Collect recorded data
t_seq = t;
y_lon_seq = [theta q u w];
input_seq = [delta_a, delta_vr, delta_vl, n_p]; % Actuator dynamics simulated beforehand

num_variables_to_pred = 5;
R_sq = zeros(num_maneuvers, num_variables_to_pred);
for maneuver_i = 1
    % Get data for desired maneuver
    [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, delta_a_m, delta_vl_m, delta_vr_m, n_p_m, p_dot_m, q_dot_m, r_dot_m]...
    = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_vl_sp, delta_vr_sp, delta_a, delta_vl, delta_vr, n_p, p_dot, q_dot, r_dot);
    input_seq_m = [delta_a_m delta_vl_m delta_vr_m n_p_m];
    lon_state_seq_m = [theta_m q_m u_m w_m];
    maneuver_seq_m = [t_m input_seq_m lon_state_seq_m];

    % Integrate dynamics
    y0 = [phi_m(1) psi_m(1) p_m(1) r_m(1) v_m(1)];
    tspan = [t_m(1) t_m(end)];

    % Integrate dynamics
    disp("Simulating dynamics for maneuver " + maneuver_i);
    tic
    [t_pred, y_pred] = ode45(@(t,y) lat_dynamics_liftdrag_c(t, y, maneuver_seq_m, all_params), tspan, y0);
    y_pred = interp1(t_pred, y_pred, t_m);
    acc = calc_acc_lat(y_pred, maneuver_seq_m, all_params);
    toc

    R_sq_phi = calc_R_sq(phi_m, y_pred(:,1));
    R_sq_psi = calc_R_sq(psi_m, y_pred(:,2));
    R_sq_p = calc_R_sq(p_m, y_pred(:,3));
    R_sq_r = calc_R_sq(r_m, y_pred(:,4));
    R_sq_v = calc_R_sq(v_m, y_pred(:,5));
    R_sq_man = [R_sq_phi R_sq_psi R_sq_p R_sq_r R_sq_v];
    R_sq(maneuver_i,:) = R_sq_man;
    
    if save_plot || show_plot
        plot_maneuver_lat("traj_val_maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_vl_m, delta_vr_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, n_p_m, a_x_m, a_y_m, a_z_m, p_dot_m, q_dot_m, r_dot_m,...
            t_m, [y_pred acc],...
            save_plot, show_plot, plot_output_location, R_sq_man);
        plot_coeffs_lat("coeffs_val_maneuver" + maneuver_i, x_lat, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_vl_m, delta_vr_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, n_p_m, a_x_m, a_y_m, a_z_m, p_dot_m, q_dot_m, r_dot_m,...
            t_m, [y_pred acc],...
            save_plot, show_plot, plot_output_location, R_sq_man);
    end
end


function [all_params] = collect_all_params_roll_initial(x)
    equation_error_results_lat;
    all_params = [c_Y_0 c_Y_beta c_Y_p c_Y_r x(1) c_Y_delta_r...
                  x(2) c_l_beta x(3) x(4) x(5) c_l_delta_r...
                  c_n_0 c_n_beta x(6) c_n_r x(7) c_n_delta_r];
end

function [all_params] = collect_all_params_roll(x, x_from_yaw)
    all_params = [x_from_yaw(1) x_from_yaw(2) x_from_yaw(3) x_from_yaw(4) x(1) x_from_yaw(5)...
                  x(2) x_from_yaw(6) x(3) x(4) x(5) x_from_yaw(7)...
                  x_from_yaw(8) x_from_yaw(9) x(6) x_from_yaw(10) x(7) x_from_yaw(11)];
end

function [all_params] = collect_all_params_yaw(x, x_from_roll)
    all_params = [x(1) x(2) x(3) x(4) x_from_roll(1) x(5)...
                  x_from_roll(2) x(6) x_from_roll(3) x_from_roll(4) x_from_roll(5) x(7)...
                  x(8) x(9) x_from_roll(6) x(10) x_from_roll(7) x(11)];
end