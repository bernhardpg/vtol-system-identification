clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lat.mat");

% Load equation_error parameters which will be used as initial guesses
load("model_identification/equation_error/results/equation_error_coeffs_lat.mat");

% Lateral system
% State = [v p r phi]
% Input = [delta_a delta_r]

maneuver_types = [
    "roll_211",...
    "yaw_211",...
    ];

% Start with one maneuver
% 1. Simulate dynamics with current coeffs
% 2. Calculate residuals
% 3. Estimate noise covariance R_hat
% 4. Calculate cost function value with residuals and noise covariance
% matrix


curr_coeffs_lat = equation_error_coeffs_lat;
lat_model = NonlinearModel(zeros(5,3), equation_error_coeffs_lat);
opt_problem = OutputErrorProblem(fpr_data_lat, lat_model, maneuver_types);

tic
opt_problem.solve();
toc



%%

maneuver_types = ["roll_211" "yaw_211"];
data_type = "train";
load_data;
load_const_params;

% Load initial guesses
equation_error_results_lat;
lon_params = [-0.1074    0.1720    2.5516  -10.0212  535.1918   -0.0493   -0.4894   -5.4394    3.3249   -0.5300    0.0115   -1.3611  -22.6064   -0.8852   -0.5934];

x0_lat = [
     c_Y_0, c_Y_p, c_Y_v, c_Y_delta_a, c_Y_delta_r,...
     c_l_0, c_l_p, c_l_r, c_l_v, c_l_delta_a,...
     c_n_0, c_n_p, c_n_r, c_n_v, c_n_delta_r,...
     ];
 
all_params = [const_params;
              lon_params';
              x0_lat'];

% Collect recorded data
t_seq = t;
input_seq = [delta_a delta_e delta_r n_p]; % Actuator dynamics simulated beforehand
y_seq = [phi theta psi p q r u v w];

%%
% Optimization

% Variable bounds
allowed_param_change = 0.6;
LB = min([x0_lat * (1 - allowed_param_change); x0_lat * (1 + allowed_param_change)]);
UB = max([x0_lat * (1 - allowed_param_change); x0_lat * (1 + allowed_param_change)]);

weight = diag([1 0 1 1 0 1 0 1 0]); % do not weight lon states

% Opt settings
rng default % For reproducibility
numberOfVariables = length(x0_lat);
options = optimoptions('ga','UseParallel', true, 'UseVectorized', false,...
    'PlotFcn',@gaplotbestf,'Display','iter');
options.InitialPopulationMatrix = x0_lat;
options.FunctionTolerance = 1e-02;

% Run optimization problem on each maneuver separately
xs = zeros(num_maneuvers, length(x0_lat));
for maneuver_i = 1
    disp("== Solving for maneuver " + maneuver_i + " ==");
    
    % Organize data for maneuver
    [t_seq_m, y_seq_m, input_seq_m] = extract_man_data_full(maneuver_i, maneuver_indices, t_seq, y_seq, input_seq);
    y0 = y_seq_m(1,:);
    data_seq_maneuver = [t_seq_m input_seq_m];
    N = length(data_seq_maneuver);
    residual_weight = diag(linspace(1,0,N)); % Weight states in the beginning more
    tspan = [t_seq_m(1) t_seq_m(end)];

    % Initial calculations for maneuver
    residuals_0 = calc_residuals_lat(y_seq_m, data_seq_maneuver, const_params, lon_params, x0_lat, dt, tspan, y0);
    R_hat_0 = diag(diag(residuals_0' * residuals_0) / N); % Assume cross-covariances to be zero
    fval_0 = cost_fn_lat(x0_lat, weight, residual_weight, R_hat_0, dt, data_seq_maneuver, y0, tspan, y_seq_m, const_params, lon_params);
    
    % Set initial values
    x = x0_lat;
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
        FitnessFunction = @(x) cost_fn_lat(x, weight, residual_weight, R_hat_0, dt, data_seq_maneuver, y0, tspan, y_seq_m, const_params, lon_params);
        [x,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],LB,UB,[],options);
        
        % Calc new covariance matrix
        residuals = calc_residuals_lat(y_seq_m, data_seq_maneuver, const_params, lon_params, x, dt, tspan, y0);
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
    
    writematrix(xs, "lat_params.txt")
end

display("Finished running output-error");