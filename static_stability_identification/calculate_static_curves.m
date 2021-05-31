clc; clear all; close all;

metadata_filename = "data/metadata.json";
metadata = read_metadata(metadata_filename);

% Load data
[state, input, c_L, c_D, c_m, AoA_rad] = read_experiment_data(metadata);
AoA_deg = rad2deg(AoA_rad);

p = state(:,5);
q = state(:,6);
r = state(:,7);
delta_a = input(:,5);
delta_e = input(:,6);
delta_r = input(:,7);

% Load airframe properties
aircraft_properties;

v_B = state(:,8:10);
V_a = sqrt(v_B(:,1).^2 + v_B(:,2).^2 + v_B(:,3).^2);


%% Create plots of data
scatter_lift(AoA_rad, c_L, q);
scatter_drag(AoA_rad, c_D, q);
scatter_pitch_moment(AoA_rad, c_m, q);

%% Fift lift and drag as a function of AoA and q
[c_L_0, c_L_alpha, c_L_q] = calculate_lift_curve(AoA_rad, c_L, q, nondim_constant_lon)
[c_D_p, c_D_alpha_sq, c_D_q] = calculate_drag_curve(AoA_rad, c_D, q, nondim_constant_lon)

plot_lift_surface(c_L_0, c_L_alpha, c_L_q, AoA_deg, q, c_L, nondim_constant_lon)
plot_drag_surface(c_D_p, c_D_alpha_sq, c_D_q, AoA_deg, q, c_D, nondim_constant_lon)

c_L_estimated = c_L_0 + c_L_alpha .* AoA_rad + c_L_q * nondim_constant_lon .* q;
c_D_estimated = c_D_p + c_D_alpha_sq * AoA_rad .^2 + c_D_q * nondim_constant_lon .* q;

c_L_rmse = calculate_rmse(c_L, c_L_estimated)
c_L_r_sq = calculate_r_sq(c_L, c_L_estimated)
c_D_rmse = calculate_rmse(c_D, c_D_estimated)
c_D_r_sq = calculate_r_sq(c_D, c_D_estimated)

%print_results(c_L0, c_Lalpha, c_Lq, c_Dp, c_Dalpha, c_Dq);

%% Add linear term to drag for comparison
[c_L_0, c_L_alpha, c_L_q] = calculate_lift_curve(AoA_rad, c_L, q, nondim_constant_lon);
[c_D_p, c_Dalpha, c_D_alpha_sq, c_D_q] = calculate_drag_curve_quadratic(AoA_rad, c_D, q, nondim_constant_lon);

plot_lift_surface(c_L_0, c_L_alpha, c_L_q, AoA_deg, q, c_L, nondim_constant_lon)
plot_drag_surface_w_linear_term(c_D_p, c_Dalpha, c_D_alpha_sq, c_D_q, AoA_deg, q, c_D, nondim_constant_lon)

c_L_estimated = c_L_0 + c_L_alpha .* AoA_rad + c_L_q * nondim_constant_lon .* q;
c_D_estimated = c_D_p + c_Dalpha * AoA_rad + c_D_alpha_sq * AoA_rad .^2 + c_D_q * nondim_constant_lon .* q;

c_L_rmse = calculate_rmse(c_L, c_L_estimated)
c_L_r_sq = calculate_r_sq(c_L, c_L_estimated)
c_D_rmse = calculate_rmse(c_D, c_D_estimated)
c_D_r_sq = calculate_r_sq(c_D, c_D_estimated)

%print_results(c_L0, c_Lalpha, c_Lq, c_Dp, c_Dalpha, c_Dq);

% Linear term doesn't do much, so do not use this.


%% Fit lift and drag as a function of only AoA
[c_L_0, c_L_alpha] = calculate_2d_lift_curve(AoA_rad, c_L);
c_L_estimated = c_L_0 + c_L_alpha .* AoA_rad;
c_L_rmse = calculate_rmse(c_L, c_L_estimated)
c_L_r_sq = calculate_r_sq(c_L, c_L_estimated)

% Purely quadratic drag
[c_D_p, c_Dalpha] = calculate_2d_drag_curve(AoA_rad, c_D);
c_D_estimated = c_D_p + c_Dalpha .* AoA_rad .^2;
plot_drag_curve(c_D_p, 0, c_Dalpha, AoA_deg, c_D);
c_D_rmse = calculate_rmse(c_D, c_D_estimated)
c_D_r_sq = calculate_r_sq(c_D, c_D_estimated)

% Drag with both linear and quadratic term
[c_D_p, c_Dalpha, c_D_alpha_sq] = calculate_2d_drag_curve_full_quadratic(AoA_rad, c_D);
c_D_estimated = c_D_p + c_Dalpha .* AoA_rad + c_D_alpha_sq .* AoA_rad .^ 2;
c_D_rmse = calculate_rmse(c_D, c_D_estimated)
c_D_r_sq = calculate_r_sq(c_D, c_D_estimated)

plot_lift_curve(c_L_0, c_L_alpha, AoA_deg, c_L);
plot_drag_curve(c_D_p, c_Dalpha, c_D_alpha_sq, AoA_deg, c_D);

%% Fit pitch moment as a function of both AoA and q
[c_m0, c_malpha, c_mq] = calculate_pitch_moment_surface(AoA_rad, q, c_m, nondim_constant_long);

c_m_estimated = c_m0 + c_malpha * AoA_rad .^2 + ...
    nondim_constant_long * c_mq .* q;

plot_pitch_moment_surface(c_m0, c_malpha, c_mq, AoA_deg, q, c_m, nondim_constant_long);

c_m_rmse = calculate_rmse(c_m, c_m_estimated)
c_m_r_sq = calculate_r_sq(c_m, c_m_estimated)

%% Fit pitch moment as a function of only AoA
[c_m0, c_malpha] = calculate_pitch_moment_curve(AoA_rad, c_m);

c_m_estimated = c_m0 + c_malpha * AoA_rad;
plot_pitch_moment_curve(c_m0, c_malpha, AoA_deg, c_m);

[fit_c_m, ~] = calculate_fit(c_m, c_m_estimated)

%%
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

    for i = 1:num_experiments
        datapath = experiment_data_path + "experiment_" + metadata.Experiments(i).Number ...
            + "/sweep/output/";

        state_exp = readmatrix(datapath + "state.csv");
        input_exp = readmatrix(datapath + "input.csv");
        c_L_exp = readmatrix(datapath + "cl.csv");
        c_D_exp = readmatrix(datapath + "cd.csv");
        c_m_exp = readmatrix(datapath + "cm.csv");
        AoA_rad_exp = readmatrix(datapath + "aoa_rad.csv");
        maneuver_start_indices_exp = readmatrix(datapath + "maneuver_start_indices.csv");
        AoA_deg_exp = AoA_rad .* (180 / pi);

        state = [state;
                 state_exp];
        input = [input;
                 input_exp];
        c_L = [c_L;
               c_L_exp];
        c_D = [c_D;
               c_D_exp];
        AoA_rad = [AoA_rad;
                   AoA_rad_exp];
        c_m = [c_m;
               c_m_exp];
        maneuver_start_indices = [maneuver_start_indices...
            maneuver_start_indices_exp];
    end
    
    disp("Loaded " + length(maneuver_start_indices) + " maneuvers.")
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



function [] = plot_lift_surface(c_L0, c_Lalpha, c_Lq, AoA_deg_exp, q_exp, c_L_exp, constant_q)
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
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
end

function [] = plot_drag_surface(c_Dp, c_Dalpha, c_Dq, AoA_deg_exp, q_exp, c_D_exp, constant_q)
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
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
end

function [] = plot_drag_surface_w_linear_term(c_Dp, c_Dalpha, c_Dalpha_sq, c_Dq, AoA_deg_exp, q_exp, c_D_exp, constant_q)
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
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
end

function [] = plot_pitch_moment_curve(c_m0, c_malpha, AoA_deg_exp, c_m_exp)
    AoA_deg = 0:0.5:9;
    AoA_rad = AoA_deg .* (pi / 180);
    
    c_m_estimated = c_m0 + c_malpha .* AoA_rad;

    fig = figure;
    fig.Position = [100 100 500 300];
    s = scatter(AoA_deg_exp, c_m_exp); hold on;
    alpha(s, 0.7)
    plot(AoA_deg, c_m_estimated); 
    xlabel("\alpha [deg]")
    ylabel("c_m")
    title("Pitch moment coefficient")
    
    filename = "pitch_moment_coeff_curve";
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
end

function [] = plot_drag_curve(c_Dp, c_Dalpha, c_Dalpha_sq, AoA_deg_exp, c_D_exp)
    AoA_deg = -2:0.5:9;
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
    ylim([0 0.1])
    xlim([-2 9])
    
    filename = "drag_coeff_curve";
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
end

function [] = plot_lift_curve(c_L0, c_Lalpha, AoA_deg_exp, c_L_exp)
    AoA_deg = -2:0.5:9;
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
    xlim([-2 9])
    
    filename = "lift_coeff_curve";
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
end

function [] = plot_pitch_moment_surface(c_m0, c_malpha, c_mq, AoA_deg_exp, q_exp, c_m_exp, constant_q)
    AoA_deg = 0:0.5:9;
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
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
end

function [] = scatter_lift(AoA_rad, c_L, q)
    fig = figure;
    fig.Position = [100 100 500 300];
    scatter(AoA_rad * 180 / pi, c_L, [], q, 'filled'); alpha(0.25);
    c = colorbar;
    c.Label.String = 'q [rad/s]';
    xlabel("\alpha [deg]")
    ylabel("c_L")
    title("Lift coefficients obtained from experimental data")
    ylim([0 1.15])
    filename = "lift_coeff_experiments";
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
end

function [] = scatter_drag(AoA_rad, c_D, q)
    fig = figure;
    fig.Position = [100 100 500 300];
    scatter(AoA_rad * 180 / pi, c_D, [], q, 'filled'); alpha(0.25);
    c = colorbar;
    c.Label.String = 'q [rad/s]';
    xlabel("\alpha [deg]")
    ylabel("c_D")
    title("Drag coefficients obtained from experimental data")
    ylim([0 0.2])
    filename = "drag_coeff_experiments";
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
end

function [] = scatter_pitch_moment(AoA_rad, c_m, q)
    fig = figure;
    fig.Position = [100 100 500 300];
    scatter(AoA_rad * 180 / pi, c_m, [], q, 'filled'); alpha(0.25);
    c = colorbar;
    c.Label.String = 'q [rad/s]';
    xlabel("\alpha [deg]")
    ylabel("c_m")
    title("Pitch moment coefficients obtained from experimental data")
    %ylim([0 1.15])
    filename = "pitch_coeff_experiments";
    saveas(fig, "plots/" + filename, 'epsc')
    savefig("plots/" + filename + '.fig')
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
