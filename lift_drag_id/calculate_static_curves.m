clc; clear all; close all;

metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Load data
[state, input, c_L, c_D, c_m, AoA_rad] = read_experiment_data(metadata);

q_NB = state(:,1:4);
u = state(:,8);
v = state(:,9);
w = state(:,10);

p = state(:,5);
q = state(:,6);
r = state(:,7);
delta_a = input(:,5);
delta_e = input(:,6);
delta_r = input(:,7);
n_p = input(:,8);

% Load airframe properties
aircraft_properties;

v_B = state(:,8:10);
V_a = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);

[u_NED, v_NED, w_NED] = calc_v_ned(q_NB, v_B);
%% Clean data
indices = 1:length(c_L);
% Add a bunch of data dimensions to make it possible to separate out bad
% data
data = [c_L c_D smoothdata(c_m) AoA_rad q delta_e n_p V_a p r delta_a delta_r u v w w_NED indices'];

% Remove data with throttle
n_p_index = 7;
data = remove_rows_tres(data, n_p_index, 0, 1);

% Remove outliers
data = rmoutliers(data);

% Remove data with high pitch rate
q_index = 5;
[data] = remove_rows_tres(data, q_index, -0.13, 0.13);

% Remove parts of data that does not coincide with the rest, by separating
% on delta_r movement and w_NED movement.
delta_r_index = 12;
[data] = remove_rows_tres(data, delta_r_index, -0.04, 0.04);
w_NED_index = 16;
[data] = remove_rows_tres(data, w_NED_index,-999, 3.5);

% Keep all these for possibilies of separating data on different features
c_L = data(:,1);
c_D = data(:,2);
c_m = data(:,3);
AoA_rad = data(:,4);
AoA_deg = rad2deg(AoA_rad);
q = data(:,5);
delta_e = data(:,6);
n_p = data(:,7);
V_a = data(:,8);
p = data(:,9);
r = data(:,10);
delta_a = data(:,11);
delta_r = data(:,12);
u = data(:,13);
v = data(:,14);
w = data(:,15);
w_NED = data(:,16);
indices = data(:,17);

%% Create plots of data
plot_location = "static_stability_identification/plots/";

comp_against = q;
disp("Using " + length(c_L) + " datapoints");

scatter_lift(AoA_rad, c_L, comp_against, plot_location);
scatter_drag(AoA_rad, c_D, comp_against, plot_location);
scatter_pitch_moment(AoA_rad, c_m, comp_against, plot_location);

%% Fift lift and drag as a function of AoA and q
[c_L_0, c_L_alpha, c_L_q] = calculate_lift_curve(AoA_rad, c_L, q, nondim_constant_lon)
[c_D_p, c_D_alpha_sq, c_D_q] = calculate_drag_curve(AoA_rad, c_D, q, nondim_constant_lon)

plot_lift_surface(c_L_0, c_L_alpha, c_L_q, AoA_deg, q, c_L, nondim_constant_lon, plot_location)
plot_drag_surface(c_D_p, c_D_alpha_sq, c_D_q, AoA_deg, q, c_D, nondim_constant_lon, plot_location)

c_L_estimated = c_L_0 + c_L_alpha .* AoA_rad + c_L_q * nondim_constant_lon .* q;
c_D_estimated = c_D_p + c_D_alpha_sq * AoA_rad .^2 + c_D_q * nondim_constant_lon .* q;

c_L_rmse = calculate_rmse(c_L, c_L_estimated)
c_L_r_sq = calculate_r_sq(c_L, c_L_estimated)
c_D_rmse = calculate_rmse(c_D, c_D_estimated)
c_D_r_sq = calculate_r_sq(c_D, c_D_estimated)

%print_results(c_L0, c_Lalpha, c_Lq, c_Dp, c_Dalpha, c_Dq);

%% Add linear term to drag for comparison
[c_L_0, c_L_alpha, c_L_q] = calculate_lift_curve(AoA_rad, c_L, q, nondim_constant_lon);
[c_D_p, c_D_alpha, c_D_alpha_sq, c_D_q] = calculate_drag_curve_quadratic(AoA_rad, c_D, q, nondim_constant_lon);

plot_lift_surface(c_L_0, c_L_alpha, c_L_q, AoA_deg, q, c_L, nondim_constant_lon)
plot_drag_surface_w_linear_term(c_D_p, c_D_alpha, c_D_alpha_sq, c_D_q, AoA_deg, q, c_D, nondim_constant_lon)

c_L_estimated = c_L_0 + c_L_alpha .* AoA_rad + c_L_q * nondim_constant_lon .* q;
c_D_estimated = c_D_p + c_D_alpha * AoA_rad + c_D_alpha_sq * AoA_rad .^2 + c_D_q * nondim_constant_lon .* q;

c_L_rmse = calculate_rmse(c_L, c_L_estimated)
c_L_r_sq = calculate_r_sq(c_L, c_L_estimated)
c_D_rmse = calculate_rmse(c_D, c_D_estimated)
c_D_r_sq = calculate_r_sq(c_D, c_D_estimated)

%print_results(c_L0, c_Lalpha, c_Lq, c_Dp, c_Dalpha, c_Dq);

% Linear term doesn't do much, so do not use this.


%% Fit lift and drag as a function of only AoA
%%%% USE THIS!
[c_L_0, c_L_alpha] = calculate_2d_lift_curve(AoA_rad, c_L)
c_L_estimated = c_L_0 + c_L_alpha .* AoA_rad;
c_L_rmse = calculate_rmse(c_L, c_L_estimated)
c_L_r_sq = calculate_r_sq(c_L, c_L_estimated)

% Purely quadratic drag
% [c_D_p, c_D_alpha] = calculate_2d_drag_curve(AoA_rad, c_D)
% c_D_estimated = c_D_p + c_D_alpha_sq .* AoA_rad .^2;
% plot_drag_curve(c_D_p, 0, c_D_alpha, AoA_deg, c_D, plot_location);
% c_D_rmse = calculate_rmse(c_D, c_D_estimated)
% c_D_r_sq = calculate_r_sq(c_D, c_D_estimated)

% Drag with both linear and quadratic term
[c_D_p, c_D_alpha, c_D_alpha_sq] = calculate_2d_drag_curve_full_quadratic(AoA_rad, c_D)
c_D_estimated = c_D_p + c_D_alpha .* AoA_rad + c_D_alpha_sq .* AoA_rad .^ 2;
c_D_rmse = calculate_rmse(c_D, c_D_estimated)
c_D_r_sq = calculate_r_sq(c_D, c_D_estimated)

plot_lift_curve(c_L_0, c_L_alpha, AoA_deg, c_L, plot_location);
plot_drag_curve(c_D_p, c_D_alpha, c_D_alpha_sq, AoA_deg, c_D, plot_location);

%% Fit pitch moment as a function of both AoA and q
[c_m0, c_malpha, c_mq] = calculate_pitch_moment_surface(AoA_rad, q, c_m, nondim_constant_long);

c_m_estimated = c_m0 + c_malpha * AoA_rad .^2 + ...
    nondim_constant_long * c_mq .* q;

plot_pitch_moment_surface(c_m0, c_malpha, c_mq, AoA_deg, q, c_m, nondim_constant_long);

c_m_rmse = calculate_rmse(c_m, c_m_estimated)
c_m_r_sq = calculate_r_sq(c_m, c_m_estimated)

%% Fit pitch moment as a function of only AoA
[c_m_0, c_m_alpha] = calculate_pitch_moment_curve(AoA_rad, c_m)

c_m_estimated = c_m_0 + c_m_alpha * AoA_rad;
plot_pitch_moment_curve(c_m_0, c_m_alpha, AoA_deg, c_m, plot_location);

c_m_rmse = calculate_rmse(c_m, c_m_estimated)
c_m_r_sq = calculate_r_sq(c_m, c_m_estimated)
%%
function [u_NED, v_NED, w_NED] = calc_v_ned(q_NB, v_B)
    N = length(v_B);
    R_NB = quat2rotm(q_NB);

    v_N = zeros(N, 3);
    for i = 1:N
       % Notice how the Rotation matrix has to be inverted here to get the
       % right result, indicating that q is in fact q_NB and not q_BN.
       v_N(i,:) = (R_NB(:,:,i) * v_B(i,:)')';
    end
    u_NED = v_B(:,1);
    v_NED = v_B(:,2);
    w_NED = v_B(:,3);
end


function [data] = remove_rows_tres(data, state_num, min_tres, max_tres)
    i = 1;
    while i < length(data)

       % Remove rows with large q
       if (data(i, state_num) > max_tres) || (data(i, state_num) < min_tres)
          data(i,:) = [];
          i = i - 1;
       end
       i = i + 1;
    end
end

function [RMSE] = calculate_rmse(y, y_estimated)
    MSE = mean((y - y_estimated).^2);
    RMSE = sqrt(MSE);
end

function [r_sq] = calculate_r_sq(y, y_estimated)
    sum_squares_total = sum((y - mean(y)).^2);
    sum_squares_residuals = sum((y - y_estimated).^2);
    r_sq = 1 - (sum_squares_residuals / sum_squares_total);
end


function [] = print_results(c_L0, c_Lalpha, c_Lq, c_Dp, c_Dalpha, c_Dq)
    disp("c_L = c_L0 + c_Lalpha * alpha + c_Lq * q");
    disp("c_L0 = " + c_L0);
    disp("c_Lalpha = " + c_Lalpha);
    disp("c_Lq = " + c_Lq);
    disp("c_D = c_Dp + c_Dalpha * alpha.^2 + c_Dq * q");
    disp("c_Dp = " + c_Dp);
    disp("c_Dalpha = " + c_Dalpha);
    disp("c_Dq = " + c_Dq);
end

function [] = calculate_correlations(c_L, c_D, AoA_rad, q, delta_a, delta_e, delta_r)
    corr_lift_aoa = corrcoef(c_L, AoA_rad)
    corr_lift_q = corrcoef(c_L, q)
    corr_lift_elevator = corrcoef(c_L, delta_e)
    corr_lift_aileron = corrcoef(c_L, delta_a)
    corr_lift_rudder = corrcoef(c_L, delta_r)
    corr_elevator_rudder = corrcoef(delta_e, delta_r)
end


function [] = remove_high_pitch_rates(c_L, c_D, c_m, AoA_rad)


end

function [state, input, c_L, c_D, c_m, AoA_rad] = read_experiment_data(metadata)
    num_experiments = length(metadata.Experiments);
    experiment_data_path = "data/experiments/";

    state = [];
    input = [];
    c_L = [];
    c_D = [];
    AoA_rad = [];
    c_m = [];
    maneuver_start_indices = [];
    
    num_skipped_exp = 0;

    for exp_i = 1:num_experiments
        datapath = experiment_data_path + "experiment_" + metadata.Experiments(exp_i).Number ...
            + "/sweep/output/";
        if ~isfolder(datapath)
            continue;
        end

        state_exp = readmatrix(datapath + "state.csv");
        input_exp = readmatrix(datapath + "input.csv");
        c_L_exp = readmatrix(datapath + "cl.csv");
        c_D_exp = readmatrix(datapath + "cd.csv");
        c_m_exp = readmatrix(datapath + "cm.csv");
        AoA_rad_exp = readmatrix(datapath + "aoa_rad.csv");
        maneuver_start_indices_exp = readmatrix(datapath + "maneuver_start_indices.csv");
        AoA_deg_exp = AoA_rad_exp .* (180 / pi);
        
        % Iterate through each maneuver
        for maneuver_i = 1:length(maneuver_start_indices_exp)
            start_index = maneuver_start_indices_exp(maneuver_i);
            
            if maneuver_i == length(maneuver_start_indices_exp)
                end_index = length(state_exp);
            else
                end_index = maneuver_start_indices_exp(maneuver_i + 1);
            end
            
            state_maneuver = state_exp(start_index:end_index,:);
            input_maneuver = input_exp(start_index:end_index,:);
            c_L_maneuver = c_L_exp(start_index:end_index);
            c_D_maneuver = c_D_exp(start_index:end_index);
            c_m_maneuver = c_m_exp(start_index:end_index);
            AoA_rad_maneuver = AoA_rad_exp(start_index:end_index);
            AoA_deg_maneuver = AoA_deg_exp(start_index:end_index);
            
%             % Do not aggregate maneuvers with large q values
%             q = state_maneuver(:,6);
%             max_q = max(q);
%             if max_q > 0.4
%                 num_skipped_exp = num_skipped_exp + 1;
%                 continue
%             end
            
            % Aggregate maneuver
            state = [state;
                     state_maneuver];
            input = [input;
                     input_maneuver];
            c_L = [c_L;
                   c_L_maneuver];
            c_D = [c_D;
                   c_D_maneuver];
            AoA_rad = [AoA_rad;
                       AoA_rad_maneuver];
            c_m = [c_m;
                   c_m_maneuver];
            maneuver_start_indices = [maneuver_start_indices...
                start_index]; 

        end
    end
    
    disp("Loaded " + length(maneuver_start_indices) + " maneuvers.")
    disp("Skipped " + num_skipped_exp + " maneuvers.")
end

function [c_L0, c_Lalpha] = calculate_2d_lift_curve(AoA_rad, c_L)
    N = length(AoA_rad);
    
    % Lift shape
    % c_L = c_L0 + c_Lalpha * alpha
    Phi = [ones(1,N);
           AoA_rad'];
    Y = c_L;

    theta = least_squares_est(Phi, Y);
    c_L0 = theta(1);
    c_Lalpha = theta(2);
end

function [c_L0, c_Lalpha, c_Lq] = calculate_lift_curve(AoA_rad, c_L, q, q_constant)
    N = length(AoA_rad);
    
    Phi = [ones(1,N);
           AoA_rad';
           q_constant * q'];
    Y = c_L;

    theta = least_squares_est(Phi, Y);
    c_L0 = theta(1);
    c_Lalpha = theta(2);
    c_Lq = theta(3);
end


function [c_Dp, c_Dalpha, c_Dq] = calculate_drag_curve(AoA_rad, c_D, q, q_constant)
    N = length(AoA_rad);
    
    Phi = [ones(1,N);
           AoA_rad'.^2;
           q_constant * q'];
    Y = c_D;
    theta = least_squares_est(Phi, Y);

    c_Dp = theta(1);
    c_Dalpha = theta(2);
    c_Dq = theta(3);
end

function [c_Dp, c_Dalpha, c_Dalpha_sq, c_Dq] = calculate_drag_curve_quadratic(AoA_rad, c_D, q, q_constant)
    N = length(AoA_rad);
    
    Phi = [ones(1,N);
           AoA_rad';
           AoA_rad'.^2;
           q_constant * q'];
    Y = c_D;
    theta = least_squares_est(Phi, Y);

    c_Dp = theta(1);
    c_Dalpha = theta(2);
    c_Dalpha_sq = theta(3);
    c_Dq = theta(4);
end

function [c_Dp, c_Dalpha] = calculate_2d_drag_curve(AoA_rad, c_D)
    N = length(AoA_rad);
    
    % Quadratic drag
    % c_D = c_Dp + c_Dalpha * alpha^2
    Phi = [ones(1,N);
           AoA_rad' .^ 2];
    Y = c_D;
    theta = least_squares_est(Phi, Y);

    c_Dp = theta(1);
    c_Dalpha = theta(2);
end

function [c_Dp, c_Dalpha, c_Dalpha_sq] = calculate_2d_drag_curve_full_quadratic(AoA_rad, c_D)
    N = length(AoA_rad);
    
    % Quadratic drag
    % c_D = c_Dp + c_Dalpha * alpha^2
    Phi = [ones(1,N);
           AoA_rad';
           AoA_rad'.^2];
    Y = c_D;
    theta = least_squares_est(Phi, Y);

    c_Dp = theta(1);
    c_Dalpha = theta(2);
    c_Dalpha_sq = theta(3);
end


function [c_m0, c_malpha, c_mq] = calculate_pitch_moment_surface(AoA_rad, q, c_m, q_constant)
    % Scatter plot of all maneuvers
    N = length(AoA_rad);
    Phi = [ones(1,N);
           AoA_rad';
           q_constant * q'];
    Y = c_m;

    theta = least_squares_est(Phi, Y);
    c_m0 = theta(1);
    c_malpha = theta(2);
    c_mq = theta(3);
end

function [c_m0, c_malpha] = calculate_pitch_moment_curve(AoA_rad, c_m)
    % Scatter plot of all maneuvers
    N = length(AoA_rad);
    Phi = [ones(1,N);
           AoA_rad'];
    Y = c_m;

    theta = least_squares_est(Phi, Y);
    c_m0 = theta(1);
    c_malpha = theta(2);
end



function [] = plot_lift_surface(c_L0, c_Lalpha, c_Lq, AoA_deg_exp, q_exp, c_L_exp, constant_q, plot_location)
    AoA_deg = -2:0.5:12;
    AoA_rad = AoA_deg .* (pi / 180);
    q = 0:0.05:0.45;
    [X,Y] = meshgrid(AoA_rad, q);
    [X_deg,Y_deg] = meshgrid(AoA_deg, q);

    c_L_estimated = c_L0 + c_Lalpha .* X + ...
        constant_q * c_Lq .* Y;

    fig = figure;
    fig.Position = [100 100 500 300];
    surf(X_deg,Y_deg,c_L_estimated); hold on; alpha(0.7)
    s = scatter3(AoA_deg_exp, q_exp, c_L_exp);
    alpha(s, 0.4);
    xlabel("\alpha [deg]")
    ylabel("q [rad/s]")
    zlabel("c_L")
    zlim([0 1.15])
    xlim([0 12])
    title("Lift coefficient")
    
    filename = "lift_coeff_surface";
    saveas(fig, plot_location + filename, 'epsc')
end

function [] = plot_drag_surface(c_Dp, c_Dalpha, c_Dq, AoA_deg_exp, q_exp, c_D_exp, constant_q, plot_location)
    AoA_deg = -2:0.5:12;
    AoA_rad = AoA_deg .* (pi / 180);
    q = 0:0.05:0.45;
    [X,Y] = meshgrid(AoA_rad, q);
    [X_deg,Y_deg] = meshgrid(AoA_deg, q);

    c_D_estimated = c_Dp + c_Dalpha * X .^2 + ...
        constant_q * c_Dq .* Y;

    fig = figure;
    fig.Position = [100 100 500 300];
    surf(X_deg,Y_deg,c_D_estimated); hold on; alpha(0.7)
    s = scatter3(AoA_deg_exp, q_exp, c_D_exp);
    alpha(s, 0.4);
    xlabel("\alpha [deg]")
    ylabel("q [rad/s]")
    zlabel("c_D")
    zlim([0 0.25])
    xlim([0 12])
    title("Drag coefficient")
    
    filename = "drag_coeff_surface";
    saveas(fig, plot_location + filename, 'epsc')
    %savefig(plot_location + filename + '.fig')
end

function [] = plot_drag_surface_w_linear_term(c_Dp, c_Dalpha, c_Dalpha_sq, c_Dq, AoA_deg_exp, q_exp, c_D_exp, constant_q, plot_location)
    AoA_deg = -2:0.5:12;
    AoA_rad = AoA_deg .* (pi / 180);
    q = 0:0.05:0.45;
    [X,Y] = meshgrid(AoA_rad, q);
    [X_deg,Y_deg] = meshgrid(AoA_deg, q);

    c_D_estimated = c_Dp + c_Dalpha * X + c_Dalpha_sq * X .^2 + ...
        constant_q * c_Dq .* Y;

    fig = figure;
    fig.Position = [100 100 500 300];
    surf(X_deg,Y_deg,c_D_estimated); hold on; alpha(0.7)
    s = scatter3(AoA_deg_exp, q_exp, c_D_exp);
    alpha(s, 0.4);
    xlabel("\alpha [deg]")
    ylabel("q [rad/s]")
    zlabel("c_D")
    zlim([0 0.25])
    xlim([0 12])
    title("Drag coefficient")
    
    filename = "drag_coeff_surface";
    saveas(fig, plot_location + filename, 'epsc')
    %savefig(plot_location + filename + '.fig')
end

function [] = plot_pitch_moment_curve(c_m0, c_malpha, AoA_deg_exp, c_m_exp, plot_location)
    AoA_deg = 0:0.5:12;
    AoA_rad = AoA_deg .* (pi / 180);
    
    c_m_estimated = c_m0 + c_malpha .* AoA_rad;

    fig = figure;
    fig.Position = [100 100 500 300];
    s = scatter(AoA_deg_exp, c_m_exp, 'filled'); hold on;
    alpha(s, 0.7)
    plot(AoA_deg, c_m_estimated); 
    xlabel("\alpha [deg]")
    ylabel("c_m")
    title("Pitch moment coefficient")
    
    filename = "pitch_moment_coeff_curve";
    saveas(fig,plot_location + filename, 'epsc')
    %savefig(plot_location + filename + '.fig')
end

function [] = plot_drag_curve(c_Dp, c_Dalpha, c_Dalpha_sq, AoA_deg_exp, c_D_exp, plot_location)
    AoA_deg = -2:0.5:12;
    AoA_rad = AoA_deg .* (pi / 180);
    
    c_D_estimated = c_Dp + c_Dalpha * AoA_rad + c_Dalpha_sq * AoA_rad .^ 2;

    fig = figure;
    fig.Position = [100 100 500 300];
    s = scatter(AoA_deg_exp, c_D_exp, 'filled'); hold on;
    alpha(s, 0.3)
    plot(AoA_deg, c_D_estimated, 'LineWidth', 2); 
    xlabel("\alpha [deg]")
    ylabel("c_D")
    title("Drag coefficient")
    ylim([0 0.25])
    xlim([-2 12])
    
    filename = "drag_coeff_curve";
    saveas(fig, plot_location + filename, 'epsc')
    %savefig(plot_location + filename + '.fig')
end

function [] = plot_lift_curve(c_L0, c_Lalpha, AoA_deg_exp, c_L_exp, plot_location)
    AoA_deg = -2:0.5:12;
    AoA_rad = AoA_deg .* (pi / 180);
    
    c_L_estimated = c_L0 + c_Lalpha .* AoA_rad;

    fig = figure;
    fig.Position = [100 100 500 300];
    s = scatter(AoA_deg_exp, c_L_exp, 'filled'); hold on;
    alpha(s, 0.3)
    plot(AoA_deg, c_L_estimated, 'LineWidth', 2); 
    xlabel("\alpha [deg]")
    ylabel("c_D")
    title("Lift coefficient")
    ylim([0 1.15])
    xlim([-2 12])
    
    filename = "lift_coeff_curve";
    saveas(fig, plot_location + filename, 'epsc')
    %savefig(plot_location + filename + '.fig')
end

function [] = plot_pitch_moment_surface(c_m0, c_malpha, c_mq, AoA_deg_exp, q_exp, c_m_exp, constant_q, plot_location)
    AoA_deg = 0:0.5:12;
    AoA_rad = AoA_deg .* (pi / 180);
    q = 0:0.05:0.45;
    [X,Y] = meshgrid(AoA_rad, q);
    [X_deg,Y_deg] = meshgrid(AoA_deg, q);
    
    c_m_estimated = c_m0 + c_malpha .* X + ...
        constant_q * c_mq .* Y;

    fig = figure;
    fig.Position = [100 100 500 300];
    surf(X_deg,Y_deg,c_m_estimated); hold on; alpha(0.7)
    s = scatter3(AoA_deg_exp, q_exp, c_m_exp);
    alpha(s, 0.4);
    xlabel("\alpha [deg]")
    ylabel("q [rad/s]")
    zlabel("c_m")
    title("Pitch moment coefficient")
    
    filename = "pitch_moment_coeff_surface";
    saveas(fig, plot_location + filename, 'epsc')
    %savefig(plot_location + filename + '.fig')
end

function [] = scatter_lift(AoA_rad, c_L, q, plot_location)
    fig = figure;
    fig.Position = [100 100 500 300];
    scatter(AoA_rad * 180 / pi, c_L, [], q, 'filled'); alpha(0.25);
    c = colorbar;
    c.Label.String = 'q [rad/s]';
    xlabel("\alpha [deg]")
    ylabel("c_L")
    title("Lift coefficients obtained from experimental data")
    ylim([-0.5 1.5])
    filename = "lift_coeff_experiments";
    saveas(fig, plot_location + filename, 'epsc')
    %savefig(plot_location + filename + '.fig')
end

function [] = scatter_drag(AoA_rad, c_D, q, plot_location)
    fig = figure;
    fig.Position = [100 100 500 300];
    scatter(AoA_rad * 180 / pi, c_D, [], q, 'filled'); alpha(0.25);
    c = colorbar;
    c.Label.String = 'q [rad/s]';
    xlabel("\alpha [deg]")
    ylabel("c_D")
    title("Drag coefficients obtained from experimental data")
    ylim([-0.1 0.25])
    filename = "drag_coeff_experiments";
    saveas(fig, plot_location + filename, 'epsc')
    %savefig(plot_location + filename + '.fig')
end

function [] = scatter_pitch_moment(AoA_rad, c_m, q, plot_location)
    fig = figure;
    fig.Position = [100 100 500 300];
    scatter(AoA_rad * 180 / pi, c_m, [], q, 'filled'); alpha(0.25);
    c = colorbar;
    c.Label.String = 'q [rad/s]';
    xlabel("\alpha [deg]")
    ylabel("c_m")
    title("Pitch moment coefficients obtained from experimental data")
    ylim([-0.15 0.15])
    filename = "pitch_coeff_experiments";
    saveas(fig, plot_location + filename, 'epsc')
    %savefig(plot_location + filename + '.fig')
end

function [] = plot_3d_pitch_moment(AoA_rad, q, delta_e, c_m)
    fig = figure;
    fig.Position = [100 100 1000 300];
    subplot(1,2,1)
    scatter3(AoA_rad, q, c_m); alpha(0.3);
    xlabel("AoA")
    ylabel("q")
    zlabel("c_m")
    zlim([0 3*1e-3])

    subplot(1,2,2)
    scatter3(AoA_rad, delta_e, c_m); alpha(0.3);
    xlabel("AoA")
    ylabel("delta_e")
    zlabel("c_m")
    zlim([0 3*1e-3])

end

function [] = plot_3d_lift_drag(AoA_rad, q, c_L, c_D)
    fig = figure;
    fig.Position = [100 100 1000 300];
    subplot(1,2,1)
    scatter3(AoA_rad, q, c_L); alpha(0.3);
    xlabel("AoA")
    ylabel("q")
    zlabel("c_L")
    zlim([0 1.15])

    subplot(1,2,2)
    scatter3(AoA_rad, q, c_D); alpha(0.3);
    xlabel("AoA")
    ylabel("q")
    zlabel("c_D")
    zlim([0 0.2])
end

function [theta] = least_squares_est(Phi, Y)
    P = inv(Phi * Phi');
    B = Phi * Y;

    theta = P * B;
end

function [sigma] = blending_function(alpha_stall_rad, M, alpha_rad)
    num = 1 + exp(-M * (alpha_rad - alpha_stall_rad)) + exp(M * (alpha_rad + alpha_stall_rad));
    den = (1 + exp(-M * (alpha_rad - alpha_stall_rad))) .* (1 + exp(M * (alpha_rad + alpha_stall_rad)));
    sigma = num ./ den;
end

function [c_L_linear] = lift_linear(c_L0, c_Lalpha, alpha_rad)
    c_L_linear = c_L0 + c_Lalpha * alpha_rad;
end

function [c_L_flatplate] = lift_flat_plate(alpha_rad)
    c_L_flatplate = 2 .* sign(alpha_rad) .* sin(alpha_rad).^2 .* cos(alpha_rad);
end
