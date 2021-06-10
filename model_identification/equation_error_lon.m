clc; clear all; close all;
maneuver_types = ["pitch_211"];
load_data;

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Stepwise regression %
%%%%%%%%%%%%%%%%%%%%%%%

%%%% Initialize SR %%%%%%

clc;
F_in = 4;
F_out = 4;

SS_R_prev = 0;

z = c_Z; % output (= dependent variable)
N = length(z);
one = ones(N, 1);

X_curr = [one]; % Chosen regressors

regr = [u_hat w_hat q_hat delta_e n_p]; % Basis regressors

X_pool = [regr regr.^2]; % TODO: Add cross-terms?
pool_names = ["u_hat" "w_hat" "q_hat" "delta_e" "n_p" ...
    "u_hat_sq" "w_hat_sq" "q_hat_sq" "delta_e_sq" "n_p_sq"
    ];
chosen_regressors_names = [];

%%%%%%% START %%%%%

% Find most correlated regressor
top_corr_i = pick_next_regressor_i(X_pool, z);

% Select corresponding regressor from original pool
new_regr = X_pool(:, top_corr_i);

% Add regressor to currently chosen regressors
X_curr_potential = [X_curr new_regr];

% Do regression with new regressors
[th_hat, y_hat, v, SS_R, R_sq, s_sq] = regression_round(X_curr_potential, z);

% Check if hypothesis should be rejected or kept
F_0 = (SS_R - SS_R_prev) / s_sq;
if F_0 < F_in
    disp("Regressor should not be included")
end

% Include regressor!
new_regressor_name = pool_names(top_corr_i);
chosen_regressors_names = [chosen_regressors_names new_regressor_name];
disp("Regressor " + new_regressor_name + " included!");
X_curr = X_curr_potential;

% Remove chosen regressor from pool
X_pool(:, top_corr_i) = [];
pool_names(top_corr_i) = [];

fprintf(['Chosen regressors: ' repmat('%s ', 1, length(chosen_regressors_names)) '\n'], chosen_regressors_names)
disp("R_sq = " + R_sq + "%");
disp(" ")



perform_another_step = true;
while perform_another_step
    perform_another_step = false; % Set to true if either a regressor is added or removed
    
    % Save for comparison
    SS_R_prev = SS_R;
    
    %%%% Do forward step %%%%
    % Calculate new dependent variable that is orthogonal to regressors
    % currently in model (= residuals)
    z_ort = v;

    % Make regressor pool orthogonal to current regressors
    z_interm = X_pool; % Make new LSE with all remaining potential regressors as output variables
    X_interm = X_curr; % Use current regressors as regressors
    interm_th_hat = LSE(X_interm, z_interm);
    X_pool_ort = X_pool - X_interm * interm_th_hat; % Remove everything that can be predicted by current regressors
    
    % Find strongest correlation between regressor and dependent variable
    % that is orthogonal to current model
    top_corr_i = pick_next_regressor_i(X_pool_ort, z_ort);
    
    % Select corresponding regressor from original pool
    new_regr = X_pool(:, top_corr_i);
    
    % Add regressor to currently chosen regressors
    X_curr_potential = [X_curr new_regr]; % this will be the new regressor matrix IF the hypothesis is accepted
    
    % Do regression with new regressors
    [th_hat, y_hat, v, SS_R, R_sq, s_sq] = regression_round(X_curr_potential, z);
    
    % Check if hypothesis should be rejected or kept
    F_0 = (SS_R - SS_R_prev) / s_sq;
    if F_0 < F_in
        disp("Regressor should not be included")
    else
        % Include regressor!
        new_regressor_name = pool_names(top_corr_i);
        chosen_regressors_names = [chosen_regressors_names new_regressor_name];
        disp("Regressor " + new_regressor_name + " included!");
        X_curr = X_curr_potential;

        % Remove chosen regressor from pool
        X_pool(:, top_corr_i) = [];
        pool_names(top_corr_i) = [];

        %%%% Print status %%%%
        fprintf(['Chosen regressors: ' repmat('%s ', 1, length(chosen_regressors_names)) '\n'], chosen_regressors_names)
        disp("F_0 = " + F_0);
        disp("R_sq = " + R_sq + "%");
        
        perform_another_step = true;
    end
    
    %%%% Do backward step %%%%
    
    perform_backward_elimination = true;
    while perform_backward_elimination
        [~, N_cols] = size(X_curr);
        F_0_without_j = zeros(N_cols - 1, 1);
        for j = 2:N_cols % Do not remove bias term
            % Remove regressor j from regressors and estimate
            X_without_j = X_curr;
            X_without_j(:,j) = [];

            [~, ~, ~, SS_R_without_j, ~, s_sq_without_j] = regression_round(X_without_j, z);
            F_0_without_j(j - 1) = (SS_R - SS_R_without_j) / s_sq_without_j;
        end
        
        [F_0_without_j_min, j] = min(F_0_without_j);
        if F_0_without_j_min < F_out
           % Get regressor to remove
           regr_to_remove = X_curr(:,j);
           regr_name_to_remove = chosen_regressors_names(j);
           
           % Remove from current regressors
           X_curr(:,j) = [];
           chosen_regressors_names(j) = [];
           
           % Re-add to pool
           X_pool = [X_pool regr_to_remove];
           pool_names = [pool_names regr_name_to_remove];
           
           disp("Regressor " + regr_name_to_remove + "was removed and this step should be repeated.")
           
           % Do regression with new regressors
           [th_hat, y_hat, v, SS_R, R_sq, s_sq] = regression_round(X_curr, z);
           
           %%%% Print status %%%%
            fprintf(['Chosen regressors: ' repmat('%s ', 1, length(chosen_regressors_names)) '\n'], chosen_regressors_names)
            disp("F_0 = " + F_0);
            disp("R_sq = " + R_sq + "%");

           
           perform_backward_elimination = true;
           perform_another_step = true;
        else
           disp("No regressors should be removed.")
           perform_backward_elimination = false;
        end
    end
    
    disp(" ")
end

figure
plot(t_plot, z, t_plot, y_hat); hold on
legend("$z$", "$\hat{z}$", 'Interpreter','latex')

function [th_hat, y_hat, v, SS_R, R_sq, s_sq] = regression_round(X, z)
    % Calculate total Sum of Squares
    z_bar = mean(z);
    SS_T = (z - z_bar)' * (z - z_bar);

    % Do LSE with new set of regressors
    th_hat = LSE(X, z);

    % Estimate output
    y_hat = X * th_hat; % Estimated output

    % Calculate Regression Sum of Squares
    z_bar = mean(z);
    SS_R = (y_hat - z_bar)' * (y_hat - z_bar);
    
    % Coefficient of Determination
    R_sq = SS_R / SS_T * 100;

    v = z - y_hat; % Residuals
    [N, N_cols] = size(X);
    p = N_cols - 1; % Do not count bias term
    s_sq = (v' * v) / (N - p - 1); % Fit error variance, calculated with new regressor
end

function [top_corr_i] = pick_next_regressor_i(X_pool, z)
    r = calc_corr_coeff(X_pool, z);
    [~, top_corr_i] = max(abs(r));
end

function [th_hat] = LSE(X, z)
    D = (X' * X)^(-1);
    th_hat = D * X' * z;
end

function [r] = calc_corr_coeff(X, z)
    X_bar = mean(X);
    z_bar = mean(z);
    
    cov_Xz = (X - X_bar)' * (z - z_bar);
    var_X = diag((X - X_bar)' * (X - X_bar));
    var_z = diag((z - z_bar)' * (z - z_bar));
    r = cov_Xz ./ sqrt(var_X * var_z);
end