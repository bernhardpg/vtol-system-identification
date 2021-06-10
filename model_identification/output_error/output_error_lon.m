clc; clear all; close all;
maneuver_types = "pitch_211";
data_type = "train";
load_data;

load_const_params;

% Initial guesses from equation-error
equation_error_results_lon;
x0 = [
     c_X_0, c_X_w, c_X_w_sq, c_X_q, c_X_q_sq, c_X_delta_e,...
     c_Z_0, c_Z_w, c_Z_w_sq, c_Z_delta_e,...
     c_m_0, c_m_w, c_m_q, c_m_delta_e, c_m_delta_e_sq,...
     ];
 
param_names = [
    "c_X_0", "c_X_w", "c_X_w_sq", "c_X_q", "c_X_q_sq", "c_X_delta_e",...
    "c_Z_0", "c_Z_w", "c_Z_w_sq", "c_Z_delta_e",...
    "c_m_0", "c_m_w", "c_m_q", "c_m_delta_e", "c_m_delta_e_sq",...
    ];

% Collect recorded data
t_seq = t;
y_lon_seq = [theta q u w];
y_lat_seq = [phi, psi, p, r, v];
input_seq = [delta_a, delta_e, delta_r, n_p]; % Actuator dynamics simulated beforehand


%%
% Optimization

% Variable bounds
allowed_param_change = 0.6;
LB = min([x0 * (1 - allowed_param_change); x0 * (1 + allowed_param_change)]);
UB = max([x0 * (1 - allowed_param_change); x0 * (1 + allowed_param_change)]);

weight = diag([2 2 1 1]);

% Opt settings
rng default % For reproducibility
numberOfVariables = length(x0);
options = optimoptions('ga','UseParallel', true, 'UseVectorized', false,...
    'PlotFcn',@gaplotbestf,'Display','iter');
options.InitialPopulationMatrix = x0;
options.FunctionTolerance = 1e-02;

% Run optimization problem on each maneuver separately
xs = zeros(num_maneuvers, length(x0));
for maneuver_i = num_maneuvers
    disp("== Solving for maneuver " + maneuver_i + " ==");
    
    % Organize data for maneuver
    [t_seq_m, y_lon_seq_m, y_lat_seq_m, input_seq_m] = extract_man_data_lon(maneuver_i, maneuver_indices, t_seq, y_lon_seq, y_lat_seq, input_seq);
    delta_e_m = input_seq(:,1);
    y0 = y_lon_seq_m(1,:); % Add actuator dynamics
    data_seq_maneuver = [t_seq_m input_seq_m y_lat_seq_m];
    N = length(data_seq_maneuver);
    residual_weight = diag(linspace(1,0,N)); % Weight states in the beginning more
    tspan = [t_seq_m(1) t_seq_m(end)];

    % Initial calculations for maneuver
    residuals_0 = calc_residuals(y_lon_seq_m, data_seq_maneuver, const_params, x0, dt, tspan, y0);
    R_hat_0 = diag(diag(residuals_0' * residuals_0) / N); % Assume cross-covariances to be zero
    fval_0 = cost_fn_lon(x0, weight, residual_weight, R_hat_0, dt, data_seq_maneuver, y0, tspan, y_lon_seq_m, const_params);
    
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
        FitnessFunction = @(x) cost_fn_lon(x, weight, residual_weight, R_hat, dt, data_seq_maneuver, y0, tspan, y_lon_seq_m, const_params);
        [x,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],LB,UB,[],options);
        
        % Calc new covariance matrix
        residuals = calc_residuals(y_lon_seq_m, data_seq_maneuver, const_params, x, dt, tspan, y0);
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
    
    writematrix(xs, "lon_params.txt")
end

display("Finished running output-error");