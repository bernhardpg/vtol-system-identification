function [th_hat, chosen_regr_names, y_hat_val, R_sq_val] = stepwise_regression(z, z_val, regr, regr_val, regr_names, nonlin_regr, nonlin_regr_val, nonlin_regr_names, min_r_sq_change)
    disp("##### STEPWISE REGRESSION ROUND #####")

    %%%% Initialize SR %%%%%%
    F_in = 4;
    F_out = 4;

    N = length(z);
    N_val = length(z_val);

    regr_curr = [1]; % Chosen regressors
    X = [ones(N,1) regr nonlin_regr];
    X_val = [ones(N_val,1) regr_val nonlin_regr_val];
    
    [~, total_num_regressors] = size(X);
    regr_names = ["1" regr_names nonlin_regr_names];
    [~, num_linear_regressors] = size(regr);
    regr_pool = 2:num_linear_regressors + 1; % Remaining regressors to choose from
    
    disp("Testing regressors: ")
    fprintf(['Regressor pool: ' repmat('%s ', 1, length(regr_names(regr_pool))) '\n'], regr_names(regr_pool)) 
    
    X_curr = X(:,regr_curr);
    th_hat = LSE(X_curr, z);
    y_hat = X_curr * th_hat;
    
    y_hat_val = X_val(:,regr_curr) * th_hat;
    RSS_val_prev = calc_RSS(y_hat_val, z_val);

    %%%%%%% START %%%%%

    % Find most correlated regressor
    %fprintf(['Regressors: ' repmat('%s ', 1, length(pool_names)) '\n'], pool_names)
    
    X_pool = X(:,regr_pool);
    top_corr_i = pick_next_regressor_i(X_pool, z);
    new_regr = regr_pool(top_corr_i);

    % Add regressor to currently chosen regressors
    regr_potential = [regr_curr new_regr];

    % Do regression with potential new regressor set
    X_potential = X(:,regr_potential);
    th_hat = LSE(X_potential, z);
    y_hat = X_potential * th_hat;
    y_hat_val = X_val(:,regr_potential) * th_hat;
    
    % Check if hypothesis should be rejected or kept
    RSS_val = calc_RSS(y_hat_val, z_val);
    np = length(regr_potential);
    F0 = calc_F_value(RSS_val_prev, RSS_val, np - 1, np, N_val);
    
    R_sq = calc_R_sq(y_hat, z);
    R_sq_val = calc_R_sq(y_hat_val, z_val);
    
    if F0 < F_in
        disp("Regressor should not be included")
    end

    % Include regressor!
    new_regressor_name = regr_names(new_regr);
    regr_curr = regr_potential;

    disp("Regressor " + new_regressor_name + " included!");
    %fprintf(['Chosen regressors: ' repmat('%s ', 1, length(chosen_regressors_names)) '\n'], chosen_regressors_names)
    disp("R_sq (training) = " + R_sq + "%, " + "R_sq (validation) = " + R_sq_val + "%");
    disp(" ")
    
    % Remove chosen regressor from pool
    regr_pool(regr_pool == new_regr) = [];

    % Repeat procedure until no improvements can be made
    perform_another_step = true;
    test_nonlinear_terms = false;
    while perform_another_step
        fprintf(['Regressor pool: ' repmat('%s ', 1, length(regr_names(regr_pool))) '\n'], regr_names(regr_pool)) 
        perform_another_step = false; % Set to true if either a regressor is added or removed
        
        % Calculate current RSS and R_sq
        X_curr = X(:,regr_curr);
        th_hat = LSE(X_curr, z);
        y_hat = X_curr * th_hat;
        y_hat_val = X_val(:,regr_curr) * th_hat;
        RSS_val_prev = calc_RSS(y_hat_val, z_val);
        R_sq_prev = calc_R_sq(y_hat, z);
        R_sq_val_prev = calc_R_sq(y_hat_val, z_val);
        
        %%% Forward step %%%
        % Find next potential regressor
        X_curr = X(:,regr_curr);
        X_pool = X(:,regr_pool);
        top_corr_i = calc_strongest_orth_corr(y_hat, z, X_pool, X_curr);
        new_regr = regr_pool(top_corr_i);
        regr_potential = [regr_curr new_regr]; % this will be the new regressors IF the hypothesis is accepted

        % Do regression with potential new regressor set
        X_potential = X(:,regr_potential);
        th_hat = LSE(X_potential, z);
        y_hat = X_potential * th_hat;
        y_hat_val = X_val(:,regr_potential) * th_hat;
        
        % Check if hypothesis should be rejected or kept
        RSS_val = calc_RSS(y_hat_val, z_val);
        R_sq = calc_R_sq(y_hat, z);
        R_sq_val = calc_R_sq(y_hat_val, z_val);
        np = length(regr_potential);
        F0 = calc_F_value(RSS_val_prev, RSS_val, np - 1, np, N_val);
        R_sq_val_change = (R_sq_val - R_sq_val_prev) / R_sq_val_prev * 100;
        
        if (F0 < F_in)
            disp("Regressor should not be included: " + regr_names(new_regr) + ". Hypothesis not passed with F0 (validation) = " + F0)
            regr_pool(regr_pool == new_regr) = [];
            perform_another_step = true;
        elseif R_sq_val_change < min_r_sq_change
            disp("Regressor should not be included: " + regr_names(new_regr) + ". Change in R_sq (validation) would be = " + R_sq_val_change + "%")
            % Remove regressor from pool
            regr_pool(regr_pool == new_regr) = [];
            perform_another_step = true;
        else
            % Include regressor!
            new_regressor_name = regr_names(new_regr);
            regr_curr = regr_potential;

            % Reset regressor pool and remove chosen regressors
            regr_pool = 2:num_linear_regressors+1;
            if test_nonlinear_terms
                % Add non-linear terms to pool
                regr_pool = [regr_pool num_linear_regressors+2:total_num_regressors];
            end
            for chosen_regr_i = regr_curr
               regr_pool(regr_pool == chosen_regr_i) = [];
            end

            %%%% Print status %%%%
            disp("Regressor " + new_regressor_name + " included! F0 = " + F0 + ", change in R_sq (validation) = " + R_sq_val_change + "%");
            %fprintf(['Chosen regressors: ' repmat('%s ', 1, length(chosen_regressors_names)) '\n'], chosen_regressors_names)
            disp("R_sq (training) = " + R_sq + "%, " + "R_sq (validation) = " + R_sq_val + "%");

            perform_another_step = true;
        end

        %%%% Do backward step %%%%
        % Important, as new regressors may make previous ones uneccesarry

        perform_backward_elimination = true;
        while perform_backward_elimination
            
            % Calculate F_0 for each regressor
            np = length(regr_curr);
            F0_s = Inf(total_num_regressors, 1); % initialize all values to inf so that non-used regressors wont get removed
            
            RSS_val = calc_RSS(y_hat_val, z_val);
            R_sq_val_prev = R_sq_val;
            for regr_to_test_for_removal = regr_curr
                if regr_to_test_for_removal == 1 % Do not remove bias term
                    continue;
                end
                regr_without_j = regr_curr;
                regr_without_j(regr_without_j == regr_to_test_for_removal) = [];
                X_without_j = X(:,regr_without_j);

                % Do regression with potential new regressor set
                th_hat_wo_j = LSE(X_without_j, z);
                y_hat_wo_j = X_without_j * th_hat_wo_j;
                y_hat_val_wo_j = X_val(:,regr_without_j) * th_hat_wo_j;
                R_sq_val_without_j = calc_R_sq(y_hat_val_wo_j, z_val);

                % Check if hypothesis should be rejected or kept
                RSS_val_without_j = calc_RSS(y_hat_val_wo_j, z_val);
%                 if RSS_val_without_j < RSS_val
%                     disp("test")
%                 end
                F0 = calc_F_value(RSS_val_without_j, RSS_val, np - 1, np, N_val);
                F0_s(regr_to_test_for_removal) = F0;
            end

            % If F0_min > F_out, then null hypothesis (which is that model
            % without jth regressor is better) is discarded. Otherwise,
            % null hypothesis is actually stronger, and we need to remove
            % the regressor.
            [F0_min, regr_to_remove] = min(F0_s);
            if F0_min < F_out
               % Remove from current regressors
               regr_curr(regr_curr == regr_to_remove) = [];

               % Do regression with new regressor set
               X_curr = X(:,regr_curr);
               th_hat = LSE(X_curr, z);
               y_hat = X_curr * th_hat;
               y_hat_val = X_val(:,regr_curr) * th_hat;
               RSS_val = calc_RSS(y_hat_val, z_val);
               R_sq = calc_R_sq(y_hat, z);
               R_sq_val = calc_R_sq(y_hat_val, z_val);
               R_sq_val_change = (R_sq_val - R_sq_val_prev) / R_sq_val_prev * 100;

               % Do another step?
               perform_backward_elimination = true;
               perform_another_step = true;
                
               %%%% Print status %%%%
               regr_name_to_remove = regr_names(regr_to_remove);
               disp("Regressor " + regr_name_to_remove + " was removed. F0 = " + F0_min + ", change in R_sq (validation) = " + R_sq_val_change + "%");
               %fprintf(['Chosen regressors: ' repmat('%s ', 1, length(chosen_regressors_names)) '\n'], chosen_regressors_names)
               disp("R_sq (training) = " + R_sq + "%, " + "R_sq (validation) = " + R_sq_val + "%");
            else
               disp("No regressors should be removed.")
               fprintf(['F0 = ' repmat('%2.2f ', 1, length(F0_s)) '\n'], F0_s)
               perform_backward_elimination = false;
            end
        end

        if isempty(regr_pool)
            perform_another_step = false;
        end
        if ~perform_another_step
            if ~test_nonlinear_terms
                disp("No more to do with linear terms, test non-linear terms")
                test_nonlinear_terms = true;
                perform_another_step = true;
                
                % Reset regressor pool and remove chosen regressors
                regr_pool = 2:num_linear_regressors+1;
                for chosen_regr_i = regr_curr
                   regr_pool(regr_pool == chosen_regr_i) = [];
                end
                % Add non-linear terms to pool
                regr_pool = [regr_pool num_linear_regressors+2:total_num_regressors];
            else
                disp("Linear terms and nonlinear terms tested. Finished.")
            end
        end
        disp(" ")
    end

    % Do final regression round on training data
    X_curr = X(:,regr_curr);
    th_hat = LSE(X_curr, z);
    y_hat = X_curr * th_hat;
    RSS = calc_RSS(y_hat, z);
    R_sq = calc_R_sq(y_hat, z);
    
    % Do final regression round on validation data
    X_curr_val = X_val(:,regr_curr);
    y_hat_val = X_curr_val * th_hat;
    RSS_val = calc_RSS(y_hat_val, z_val);
    R_sq_val = calc_R_sq(y_hat_val, z_val);
    
    chosen_regr_names = regr_names(regr_curr);
end

function [top_corr_index] = calc_strongest_orth_corr(y_hat, z, X_pool, X_curr)
    v = z - y_hat;
    
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
    top_corr_index = pick_next_regressor_i(X_pool_ort, z_ort);
end


% See this wiki page: https://en.wikipedia.org/wiki/F-test
function F = calc_F_value(RSS1, RSS2, p1, p2, N)
    % Detemine whether model 2 which has p2 > p1 parameters and is more
    % complex than model 1 is actually siginificantly better than model 1.
    % N = number of datapoints
    F = ((RSS1 - RSS2) / (p2 - p1)) / (RSS2 / (N - p2));
end

% Calculate fit error variance. This assumes that the model structure is
% adequate, and is only used because it is unlikely that time series flight
% data will exactly repeat the same independent variable setting. See p.
% 101 in Klein.
function s_sq = calc_s_sq(y_hat, z)
    v = z - y_hat; % Residuals
    [N, np] = size(X);
    
    s_sq = (v' * v) / (N - np);
end

function [r] = calc_corr_coeff(X, z)
    X_bar = mean(X);
    z_bar = mean(z);
    
    cov_Xz = (X - X_bar)' * (z - z_bar);
    var_X = diag((X - X_bar)' * (X - X_bar));
    var_z = diag((z - z_bar)' * (z - z_bar));
    r = cov_Xz ./ sqrt(var_X * var_z);
end

function [cross_terms] = create_cross_terms(X)
    [N, N_regr] = size(X);
    N_cross_terms = sum(1:N_regr-1);
    
    cross_terms = zeros(N, N_cross_terms);
    count = 0;
    for i = 1:N_regr
        for j = 1:i - 1
            if i == j
                continue
            end
            cross_terms(:, count + 1) = X(:,i) .* X(:,j);
            count = count + 1;
        end
    end
end

function [top_corr_index] = pick_next_regressor_i(X_pool, z)
    r = calc_corr_coeff(X_pool, z);
    %fprintf(['Correlations: ' repmat('%2.2f ',1,length(r)) '\n'], r);
    [~, top_corr_index] = max(abs(r));
end

function [cross_terms_names] = create_cross_terms_names(regr_names)
    N_regr = length(regr_names);
    
    cross_terms_names = [];
    count = 0;
    for i = 1:N_regr
        for j = 1:i - 1
            if i == j
                continue
            end
            cross_terms_names = [cross_terms_names regr_names(i) + "*" + regr_names(j)];
            count = count + 1;
        end
    end
end