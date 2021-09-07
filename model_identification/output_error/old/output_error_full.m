clc; clear all; close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

maneuver_types = ["roll_211" "pitch_211" "yaw_211"];
data_type = "train";
load_data;
load_const_params;

% Load initial guesses
x0 = readmatrix("model_identification/output_error/results/" + "3_chosen_model_params.csv");

all_params = [const_params;
              x0'];

% Collect recorded data
t_seq = t;
input_seq = [delta_a delta_e delta_r n_p]; % Actuator dynamics simulated beforehand
y_seq = [phi theta psi p q r u v w];

%%
% Optimization

% Variable bounds
allowed_param_change = 0.15; % do not let the parameters change a lot during this last optimization
LB = min([x0 * (1 - allowed_param_change); x0 * (1 + allowed_param_change)]);
UB = max([x0 * (1 - allowed_param_change); x0 * (1 + allowed_param_change)]);

weight = diag([3 3 3 2 2 2 1 1 1]); % weight attitude angles the most

% Opt settings
rng default % For reproducibility
numberOfVariables = length(x0);
options = optimoptions('ga','UseParallel', true, 'UseVectorized', false,...
    'PlotFcn',@gaplotbestf,'Display','iter');
options.InitialPopulationMatrix = x0;
options.FunctionTolerance = 1e-02;

% Run optimization problem on each maneuver separately
xs = zeros(num_maneuvers, length(x0));
for maneuver_i = 1:num_maneuvers
    disp("== Solving for maneuver " + maneuver_i + " ==");
    
    % Organize data for maneuver
    [t_seq_m, y_seq_m, input_seq_m] = extract_man_data_full(maneuver_i, maneuver_indices, t_seq, y_seq, input_seq);
    y0 = y_seq_m(1,:);
    data_seq_maneuver = [t_seq_m input_seq_m];
    N = length(data_seq_maneuver);
    residual_weight = diag(linspace(1,0.5,N)); % Weight states in the beginning more
    tspan = [t_seq_m(1) t_seq_m(end)];

    % Initial calculations for maneuver
    residuals_0 = calc_residuals_full(y_seq_m, data_seq_maneuver, const_params, x0, dt, tspan, y0);
    R_hat_0 = diag(diag(residuals_0' * residuals_0) / N); % Assume cross-covariances to be zero
    fval_0 = cost_fn_full(x0, weight, residual_weight, R_hat_0, dt, data_seq_maneuver, y0, tspan, y_seq_m, const_params);
    
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
        FitnessFunction = @(x) cost_fn_full(x, weight, residual_weight, R_hat_0, dt, data_seq_maneuver, y0, tspan, y_seq_m, const_params);
        [x,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],LB,UB,[],options);
        
        % Calc new covariance matrix
        residuals = calc_residuals_full(y_seq_m, data_seq_maneuver, const_params, x, dt, tspan, y0);
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
    
    writematrix(xs, "full_model_params.txt")
end

display("Finished running output-error");