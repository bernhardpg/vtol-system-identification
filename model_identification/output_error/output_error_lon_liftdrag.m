clc; clear all; close all;
maneuver_types = ["pitch_211"];
data_type = "train";
load_data;

load_const_params;

% Initial guesses from equation-error
equation_error_results_lon;
x0_lon = [c_D_0 c_D_alpha c_D_alpha_sq c_D_q c_D_delta_e c_L_0 c_L_alpha c_L_alpha_sq c_L_q c_L_delta_e c_m_0 c_m_alpha c_m_q c_m_delta_e c_m_delta_e_sq];
% Collect recorded data
t_seq = t;
y_lon_seq = [theta q u w a_x a_z q_dot];
y_lat_seq = [phi, psi, p, r, v];
input_seq = [delta_a, delta_vl, delta_vr, n_p]; % Actuator dynamics simulated beforehand


%%
% Optimization

% Variable bounds
allowed_param_change = 0.5;
LB = min([x0_lon * (1 - allowed_param_change); x0_lon * (1 + allowed_param_change)]);
UB = max([x0_lon * (1 - allowed_param_change); x0_lon * (1 + allowed_param_change)]);

% % Only constrain sign of coefficients
% LB = min([sign(x0_lon) * eps ; sign(x0_lon) * inf]);
% UB = max([sign(x0_lon) * eps ; sign(x0_lon) * inf]);

weight = diag([1 1 1 1 1 1 1]); % Weight all outputs equally

% Opt settings
rng default % For reproducibility
numberOfVariables = length(x0_lon);
options = optimoptions('ga','UseVectorized', false,'Display','iter','UseParallel', true);
    %'PlotFcn',@gaplotbestf);
%options = optimoptions('fmincon','Algorithm','sqp');%,'UseParallel', true);

options.InitialPopulationMatrix = x0_lon;
options.FunctionTolerance = 1e-02;

% Run optimization problem on each maneuver separately
xs = zeros(num_maneuvers, length(x0_lon));
for maneuver_i = 1
    disp("== Solving for maneuver " + maneuver_i + " ==");
    
    % Organize data for maneuver
    [t_seq_m, y_lon_seq_m, y_lat_seq_m, input_seq_m] = extract_man_data(maneuver_i, maneuver_indices, t_seq, y_lon_seq, y_lat_seq, input_seq);
    y0 = y_lon_seq_m(1,1:4); % do not include accelerations in initial states
    data_seq_maneuver = [t_seq_m input_seq_m y_lat_seq_m];
    N = length(data_seq_maneuver);
    residual_weight = diag(linspace(1,1,N)); % Weight states in the beginning more
    tspan = [t_seq_m(1) t_seq_m(end)];

    % Initial calculations for maneuver
    residuals_0 = calc_residuals_lon([y_lon_seq_m], data_seq_maneuver, const_params, x0_lon, tspan, y0);
    R_hat_0 = diag(diag(residuals_0' * residuals_0) / N); % Assume cross-covariances to be zero
    fval_0 = cost_fn_lon(x0_lon, weight, residual_weight, R_hat_0, data_seq_maneuver, y0, tspan, y_lon_seq_m, const_params);
    
    % Set initial values
    x = x0_lon;
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
        FitnessFunction = @(x) cost_fn_lon(x, weight, residual_weight, R_hat, data_seq_maneuver, y0, tspan, y_lon_seq_m, const_params);
        tic
        [x,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],LB,UB,[],options);
        %[x, fval] = fmincon(FitnessFunction,x,[],[],[],[],LB,UB,[],options);
        toc
        
        % Calc new covariance matrix
        residuals = calc_residuals_lon(y_lon_seq_m, data_seq_maneuver, const_params, x, tspan, y0);
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
    
    writematrix(xs, "lon_params_ga.txt")
end

display("Finished running output-error");

%% Check result
maneuver_types = "pitch_211";
data_type = "val";
load_data;

all_params = [const_params;
              x'];
show_plot = true;
save_plot = false;
plot_output_location = "";

R_sq = zeros(num_maneuvers, 4);
for maneuver_i = 1:num_maneuvers
    % Get data for desired maneuver
    [t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, a_x_m, a_y_m, a_z_m, delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, delta_a_m, delta_vl_m, delta_vr_m, n_p_m]...
        = get_maneuver_data(maneuver_i, maneuver_start_indices, t, phi, theta, psi, p, q, r, u, v, w, a_x, a_y, a_z, delta_a_sp, delta_vl_sp, delta_vr_sp, delta_a, delta_vl, delta_vr, n_p);
    input_seq_m = [delta_a_m delta_vl_m delta_vr_m n_p_m];
    lat_state_seq_m = [phi_m, psi_m, p_m, r_m, v_m];
    maneuver_seq_m = [t_m input_seq_m lat_state_seq_m];

    % Integrate dynamics
    y0 = [theta_m(1) q_m(1) u_m(1) w_m(1)];
    tspan = [t_m(1) t_m(end)];

    % Integrate dynamics
    [t_pred, y_pred] = ode45(@(t,y) lon_dynamics_liftdrag_c(t, y, maneuver_seq_m, all_params), tspan, y0);
    y_pred = interp1(t_pred, y_pred, t_m);

    R_sq_theta = calc_R_sq(theta_m, y_pred(:,1));
    R_sq_q = calc_R_sq(q_m, y_pred(:,2));
    R_sq_u = calc_R_sq(u_m, y_pred(:,3));
    R_sq_w = calc_R_sq(w_m, y_pred(:,4));
    R_sq_m = [R_sq_theta R_sq_q R_sq_u R_sq_w];
    R_sq(maneuver_i,:) = R_sq_m;
    
    if save_plot || show_plot
        plot_maneuver("val_maneuver" + maneuver_i, t_m, phi_m, theta_m, psi_m, p_m, q_m, r_m, u_m, v_m, w_m, delta_a_m, delta_vl_m, delta_vr_m,  delta_a_sp_m, delta_vl_sp_m, delta_vr_sp_m, n_p_m,...
            t_m, y_pred,...
            save_plot, show_plot, plot_output_location, R_sq_m);
    end
end

function [R_sq] = calc_R_sq(z, y_hat)
    % Calculate total Sum of Squares
    z_bar = mean(z);
    SS_T = (z - z_bar)' * (z - z_bar);
    SS_E = (z - y_hat)' * (z - y_hat); % Residual Sum of Squares
    
    % Coefficient of Determination
    R_sq = (1 - SS_E/SS_T) * 100;
end
