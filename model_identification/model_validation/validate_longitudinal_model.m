clear all; close all; clc;

% Load FPR data which contains training data and validation data
load("data/flight_data/selected_data/fpr_data_lon.mat");

% Import ss model from AVL
avl_state_space_model;

% Import coeffs from AVL
load("avl_model/avl_results/avl_coeffs_lon.mat");
avl_nonlin_lon_model = NonlinearModel(avl_coeffs_lon, zeros(6,3));

% Longitudinal System
% State = [u w q theta]
% Input = [delta_e delta_t]

maneuver_types = ["pitch_211"];
use_avl_model = true;
use_nonlin_model = false;

% Simulate maneuvers with different models
for maneuver_type = maneuver_types
    for maneuver_i = 1:length(fpr_data_lon.validation.(maneuver_type))
        maneuver = fpr_data_lon.validation.(maneuver_type)(maneuver_i);
        input_sequence = maneuver.get_lon_input_sequence();
        t_data_seq = maneuver.Time;
        y_0 = maneuver.get_lon_state_initial();
        tspan = [maneuver.Time(1) maneuver.Time(end)];
        lat_state_seq = maneuver.get_lat_state_sequence();
        
        %%%%%%%%%%
        % AVL MODEL
        %%%%%%%%%%
        if use_avl_model
            % Simulate AVL state space model
            % First create perturbation state quantities, as this is what
            % the model uses
            u_nom = avl_nonlin_lon_model.Params.u_nom;
            w_nom = avl_nonlin_lon_model.Params.w_nom;
            y_nom = [u_nom w_nom 0 0];
            y_0_perturbation = y_0 - y_nom;
            
            delta_u = detrend(input_sequence,0); % AVL model assumes perturbation quantities
            [y_avl_ss_model_perturbations, t_avl_ss_model] = lsim(lon_sys, delta_u(:,1), t_data_seq, y_0_perturbation);
            y_avl_ss_model = y_avl_ss_model_perturbations + y_nom;

            % Simulate nonlinear AVL model
            [t_avl_nonlin_model, y_avl_nonlin_model] = ode45(@(t,y) avl_nonlin_lon_model.dynamics_lon_model(t, y, t_data_seq, delta_u, lat_state_seq), tspan, y_0);

            % Collect all simulations
            y_all_models = {y_avl_ss_model, y_avl_nonlin_model};
            t_all_models = {t_avl_ss_model, t_avl_nonlin_model};

            % Compare with real flight data
            model_names = ["Real data" "State Space (AVL)" "Nonlinear (AVL)"];
            plot_styles = ["-" "--" "-"];
            maneuver.show_plot_longitudinal_validation(t_all_models, y_all_models, model_names, plot_styles);
        end
        if use_nonlin_model
            
            disp("placeholder");
        end
    end
end