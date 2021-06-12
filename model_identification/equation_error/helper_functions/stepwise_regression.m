function [th_hat, chosen_regressors_names, y_hat, R_sq] = stepwise_regression(z, regr, regr_names, use_cross_terms)
    disp("##### STEPWISE REGRESSION ROUND #####")

    %%%% Initialize SR %%%%%%
    F_in = 4;
    F_out = 4;

    N = length(z);
    one = ones(N, 1);

    X_curr = [one]; % Chosen regressors
    
    th_hat = LSE(X_curr, z);
    y_hat = X_curr * th_hat;
    RSS_prev = calc_RSS(y_hat, z);

    if use_cross_terms
        cross_terms = create_cross_terms(regr);
        cross_terms_names = create_cross_terms_names(regr_names);
    else
        cross_terms = [];
        cross_terms_names = [];
    end

    %X_pool = [regr regr.^2 cross_terms];
    X_pool = [regr cross_terms];
    %regr_names_sq = regr_names + "_sq";
    pool_names = [regr_names...
     %   regr_names_sq...
        cross_terms_names
        ];
    chosen_regressors_names = [];

    %%%%%%% START %%%%%

    % Find most correlated regressor
    fprintf(['Regressors: ' repmat('%s ', 1, length(pool_names)) '\n'], pool_names)
    top_corr_index = pick_next_regressor_i(X_pool, z);

    % Select corresponding regressor from original pool
    new_regr = X_pool(:, top_corr_index);

    % Add regressor to currently chosen regressors
    X_potential = [X_curr new_regr];

    % Do regression with potential new regressor set
    th_hat = LSE(X_potential, z);
    y_hat = X_potential * th_hat;
    
    % Check if hypothesis should be rejected or kept
    RSS = calc_RSS(y_hat, z);
    [N, np] = size(X_potential);
    F0 = calc_F_value(RSS_prev, RSS, np - 1, np, N);
    if F0 < F_in
        disp("Regressor should not be included")
    end

    % Include regressor!
    new_regressor_name = pool_names(top_corr_index);
    chosen_regressors_names = [chosen_regressors_names new_regressor_name];
    disp("Regressor " + new_regressor_name + " included!");
    X_curr = X_potential;

    fprintf(['Chosen regressors: ' repmat('%s ', 1, length(chosen_regressors_names)) '\n'], chosen_regressors_names)
    R_sq = calc_R_sq(y_hat, z);
    disp("R_sq = " + R_sq + "%");
    disp(" ")
    
    % Remove chosen regressor from pool
    X_pool(:, top_corr_index) = [];
    pool_names(top_corr_index) = [];

    % Repeat procedure until no improvements can be made
    perform_another_step = true;
    tested_nonlinear_terms = false;
    while perform_another_step
        fprintf(['Regressor pool: ' repmat('%s ', 1, length(pool_names)) '\n'], pool_names)
        
        perform_another_step = false; % Set to true if either a regressor is added or removed

        % Save for comparison
        RSS_prev = RSS;
        R_sq_prev = R_sq;
        
        %%% Forward step %%%
        % Find next potential regressor
        top_corr_index = calc_strongest_orth_corr(y_hat, z, X_pool, X_curr);
        new_regr = X_pool(:, top_corr_index);
        X_potential = [X_curr new_regr]; % this will be the new regressor matrix IF the hypothesis is accepted

        % Do regression with potential new regressor set
        th_hat = LSE(X_potential, z);
        y_hat = X_potential * th_hat;
        
        % Check if hypothesis should be rejected or kept
        RSS = calc_RSS(y_hat, z);
        R_sq = calc_R_sq(y_hat, z);
        [N, np] = size(X_potential);
        F0 = calc_F_value(RSS_prev, RSS, np - 1, np, N);
        R_sq_change = (R_sq - R_sq_prev) / R_sq_prev * 100;
        
        if (F0 < F_in)
            disp("Regressor should not be included: Hypothesis not passed with F0 = " + F0)
        elseif R_sq_change < 0.5
            disp("Regressor should not be included. Change in R_sq would be = " + R_sq_change + "%")
        else
            % Include regressor!
            new_regressor_name = pool_names(top_corr_index);
            chosen_regressors_names = [chosen_regressors_names new_regressor_name];
            X_curr = X_potential;

            % Remove chosen regressor from pool
            X_pool(:, top_corr_index) = [];
            pool_names(top_corr_index) = [];

            %%%% Print status %%%%
            disp("Regressor " + new_regressor_name + " included! F0 = " + F0 + ", change in R_sq = " + R_sq_change + "%");
            fprintf(['Chosen regressors: ' repmat('%s ', 1, length(chosen_regressors_names)) '\n'], chosen_regressors_names)
            disp("R_sq = " + R_sq + "%");

            perform_another_step = true;
        end

        %%%% Do backward step %%%%
        % Important, as new regressors may make previous ones uneccesarry

        perform_backward_elimination = true;
        while perform_backward_elimination
            
            % Calculate F_0 for each regressor
            [N, np] = size(X_curr);
            F0_s = zeros(np - 1, 1);
            for j = 2:np % Do not remove bias term
                
                % Remove regressor j from regressors
                X_without_j = X_curr;
                X_without_j(:,j) = [];

                % Do regression with potential new regressor set
                th_hat = LSE(X_without_j, z);
                y_hat = X_without_j * th_hat;

                % Check if hypothesis should be rejected or kept
                RSS_without_j = calc_RSS(y_hat, z); % Note, now RSS is the one with fewer terms!
                F0 = calc_F_value(RSS_without_j, RSS, np - 1, np, N);
                
                F0_s(j - 1) = F0;
            end

            % If F0_min > F_out, then null hypothesis (which is that model
            % without jth regressor is better) is discarded. Otherwise,
            % null hypothesis is actually stronger, and we need to remove
            % the regressor.
            [F0_min, j] = min(F0_s);
            if F0_min < F_out
               % Get regressor to remove
               regr_to_remove = X_curr(:,j);
               regr_name_to_remove = chosen_regressors_names(j);

               % Remove from current regressors
               X_curr(:,j) = [];
               chosen_regressors_names(j) = [];

               % Do regression with potential new regressor set
                th_hat = LSE(X_curr, z);
                y_hat = X_curr * th_hat;

                % Check if hypothesis should be rejected or kept
                RSS = calc_RSS(y_hat, z);
                R_sq = calc_R_sq(y_hat, z);
                R_sq_change = (R_sq - R_sq_prev) / R_sq_prev * 100;

               % Do another step
                perform_backward_elimination = true;
               perform_another_step = true;
                
               %%%% Print status %%%%
               disp("Regressor " + regr_name_to_remove + " was removed. F0 = " + F0_min);
               fprintf(['Chosen regressors: ' repmat('%s ', 1, length(chosen_regressors_names)) '\n'], chosen_regressors_names)
               disp("R_sq = " + R_sq + "%, indicating a change of " + R_sq_change + "%");
            else
               disp("No regressors should be removed.")
               fprintf(['F0 = ' repmat('%2.2f ', 1, length(F0_s)) '\n'], F0_s)
               perform_backward_elimination = false;
            end
        end

        if ~perform_another_step
            if ~tested_nonlinear_terms
                disp("No more to do with linear terms, test non-linear terms")
                tested_nonlinear_terms = true;
                perform_another_step = true;
                
                % Add non-linear terms to pool
                X_pool = [X_pool regr.^2];
                pool_names = [pool_names regr_names+"_sq"];
            else
                disp("Linear terms and nonlinear terms tested. Finished.")
            end
        end
        disp(" ")
    end

    % Do final regression round
    th_hat = LSE(X_curr, z);
    y_hat = X_curr* th_hat;
    RSS = calc_RSS(y_hat, z);
    R_sq = calc_R_sq(y_hat, z);
end

function [top_corr_indexndex] = calc_strongest_orth_corr(y_hat, z, X_pool, X_curr)
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
    top_corr_indexndex = pick_next_regressor_i(X_pool_ort, z_ort);
end


% See this wiki page: https://en.wikipedia.org/wiki/F-test
function F = calc_F_value(RSS1, RSS2, p1, p2, N)
    % Detemine whether model 2 which has p2 > p1 parameters and is more
    % complex than model 1 is actually siginificantly better than model 1.
    % N = number of datapoints
    F = ((RSS1 - RSS2) / (p2 - p1)) / (RSS2 / (N - p2));
end

function [RSS] = calc_RSS(y_hat, z)
    % Calculate Regression Sum of Squares
    RSS = (y_hat - z)' * (y_hat - z);
end

function [TSS] = calc_TSS(z)
   % Calculate total Sum of Squares
    z_bar = mean(z);
    TSS = (z - z_bar)' * (z - z_bar);
end

function [R_sq] = calc_R_sq(y_hat, z)
    RSS = calc_RSS(y_hat, z);
    TSS = calc_TSS(z);
    
    % Coefficient of Determination
    R_sq = (1 - (RSS / TSS)) * 100;
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
    fprintf(['Correlations: ' repmat('%2.2f ',1,length(r)) '\n'], r);
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