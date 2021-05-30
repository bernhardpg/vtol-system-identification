function [] = sim_responses(experiments_to_use, nlgr_model, data, data_full_state, model_path, save_plots, model_type)
    % Experiments to use is only used to label plots with correct
    % experiment numbers
    num_experiments = length(experiments_to_use);
    
    if model_type == 'lon'
       input_trims = [nlgr_model.Parameters(11).Value]; % elevator % TODO: This is not correct index!!
    elseif model_type == 'lat'
        aileron_trim = nlgr_model.Parameters(12).Value;
        rudder_trim = nlgr_model.Parameters(14).Value;
        input_trims = [aileron_trim rudder_trim]; % aileron and rudder
    end
    
    for i = 1:num_experiments
        exp_i = experiments_to_use(i); % only used for plot name
        y = sim(nlgr_model, data);

        % Handle datatypes being different for single and multiple experiments
        if num_experiments == 1
            predicted_output = y.y;
            full_state = data_full_state.y;
            input = data.u;
            dt = data.Ts;
        else
            predicted_output = cell2mat(y.y(i));
            full_state = cell2mat(data_full_state.y(i));
            input = cell2mat(data.u(i));
            dt = cell2mat(data.Ts(i));
        end
        
        plot_response(exp_i, full_state, predicted_output, input, dt, input_trims, true, model_path, save_plots, model_type);
    end
end

function [] = plot_response(exp_i, full_state, predicted_output, input, dt, input_trims, plot_actual_trajectory, model_path, save_plots, model_type)
    tf = length(full_state) * dt - dt;
    t = 0:dt:tf;
    
    % Read measured state
    e0 = full_state(:,1);
    e1 = full_state(:,2);
    e2 = full_state(:,3);
    e3 = full_state(:,4);
    p = full_state(:,5);
    q = full_state(:,6);
    r = full_state(:,7);
    u = full_state(:,8);
    v = full_state(:,9);
    w = full_state(:,10);

    quat = [e0 e1 e2 e3];
    eul = quat2eul(quat);
    yaw = eul(:,1);
    pitch = eul(:,2);
    roll = eul(:,3);
    
    if model_type == "lon"
        % Read predicted state
        e0_pred = predicted_output(:,1);
        e2_pred = predicted_output(:,2);
        q_pred = predicted_output(:,3);
        u_pred = predicted_output(:,4);
        w_pred = predicted_output(:,5);
        
        elevator_trim = input_trims;

        quat_pred = [e0_pred zeros(size(e0_pred)) e2_pred zeros(size(e0_pred))]; % Model assumes only pitch movement
        eul_pred = quat2eul(quat_pred);

        roll_pred = eul_pred(:,3);
        pitch_pred = eul_pred(:,2);
        yaw_pred = eul_pred(:,1);

        % Plot
        fig = figure;
        fig.Position = [100 100 900 900];

        subplot(8,1,1)
        plot(t, roll_pred); 
        if plot_actual_trajectory
            hold on
            plot(t, roll);
        end
        legend("\phi (not part of model)", "\phi")

        subplot(8,1,2)
        plot(t, pitch_pred); 
        if plot_actual_trajectory
            hold on
            plot(t, pitch);
        end
        legend("\theta (estimated)", "\theta")

        subplot(8,1,3)
        plot(t, yaw_pred); 
        if plot_actual_trajectory
            hold on
            plot(t, yaw);
        end
        legend("\psi (not part of model)", "\psi")

        subplot(8,1,4)
        plot(t, q_pred);
        if plot_actual_trajectory
            hold on
            plot(t, q);
        end
        legend("q (estimated)", "q")

        subplot(8,1,5)
        plot(t, u_pred);
        if plot_actual_trajectory
            hold on
            plot(t, u);
        end
        legend("u (estimated)", "u")

        subplot(8,1,6)
        plot(t, w_pred);
        if plot_actual_trajectory
            hold on
            plot(t, w);
        end
        legend("w (estimated)", "w")

        subplot(8,1,7)
        plot(t, input(:,1) - elevator_trim);
        legend("\delta_e (trim subtracted)")

        subplot(8,1,8)
        plot(t, input(:,2));
        legend("n_p")

        sgtitle("experiment index: " + exp_i)
        
    elseif model_type == "lat"
        % Read predicted state
        e0_pred = predicted_output(:,1);
        e1_pred = predicted_output(:,2);
        e2_pred = predicted_output(:,3);
        e3_pred = predicted_output(:,4);
        p_pred = predicted_output(:,5);
        r_pred = predicted_output(:,6);
        v_pred = predicted_output(:,7);
        
        aileron_trim = input_trims(1);
        rudder_trim = input_trims(2);

        quat_pred = [e0_pred e1_pred e2_pred e3_pred]; % Model assumes only pitch movement
        eul_pred = quat2eul(quat_pred);

        roll_pred = eul_pred(:,3);
        pitch_pred = eul_pred(:,2);
        yaw_pred = eul_pred(:,1);

        % Plot
        fig = figure;
        fig.Position = [100 100 600 600];
        num_plots = 8;

        subplot(num_plots,1,1)
        plot(t, rad2deg(roll_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(roll));
        end
        legend("\phi (estimated)", "\phi")
        ylabel("[deg]")

        subplot(num_plots,1,2)
        plot(t, rad2deg(pitch_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(pitch));
        end
        legend("\theta (estimated)", "\theta")
        ylabel("[deg]")

        subplot(num_plots,1,3)
        plot(t, rad2deg(yaw_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(yaw));
        end
        legend("\psi (estimated)", "\psi")
        ylabel("[deg]")

        subplot(num_plots,1,4)
        plot(t, rad2deg(p_pred));
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(p));
        end
        legend("p (estimated)", "p")
        ylabel("[deg/s]")

        subplot(num_plots,1,5)
        plot(t, rad2deg(r_pred));
        if plot_actual_trajectory
            hold on
            plot(t, r);
        end
        legend("r (estimated)", "r")
        ylabel("[deg/s]")

        subplot(num_plots,1,6)
        plot(t, v_pred);
        if plot_actual_trajectory
            hold on
            plot(t, v);
        end
        legend("v (estimated)", "v")
        ylabel("[m/s]")

        subplot(num_plots,1,7)
        plot(t, rad2deg(input(:,1) - aileron_trim));
        legend("\delta_a (trim subtracted)")
        ylabel("[deg]");

        subplot(num_plots,1,8)
        plot(t, rad2deg(input(:,2) - rudder_trim));
        legend("\delta_r (trim subtracted)")
        ylabel("[deg]");
        
        sgtitle("experiment index: " + exp_i)
    end
    
    if save_plots
        filename = exp_i + model_type;
        plot_location = model_path + "plots/";
        mkdir(plot_location);
        saveas(fig, plot_location + filename, 'epsc')
        savefig(plot_location + filename + '.fig')
    end
end