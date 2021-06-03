function [] = sim_responses(nlgr_model, data, data_full_state, model_path, save_plots, model_type, show_plot)
    % Subtract trims before plotting, to plot what the model sees
    aileron_trim = nlgr_model.Parameters(16).Value;
    elevator_trim = nlgr_model.Parameters(17).Value;
    rudder_trim = nlgr_model.Parameters(18).Value;
    input_trims = [aileron_trim elevator_trim rudder_trim];
    
    num_experiments = length(data.ExperimentName);
    for i = 1:num_experiments
        exp_name = string(data(:,:,:,1).ExperimentName);
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
        
        plot_response(exp_name, full_state, predicted_output, input, dt, input_trims, true, model_path, save_plots, model_type, show_plot);
    end
end

function [] = plot_response(exp_name, full_state, predicted_output, input, dt, input_trims, plot_actual_trajectory, model_path, save_plots, model_type, show_plot)
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
    
    V_a = sqrt(u .^ 2 + v .^ 2 + w .^ 2);

    quat = [e0 e1 e2 e3];
    eul = quat2eul(quat);
    yaw = eul(:,1);
    pitch = eul(:,2);
    roll = eul(:,3);

    aileron_trim = input_trims(1);
    elevator_trim = input_trims(2);
    rudder_trim = input_trims(3);
    
    if model_type == "lon"
        % Read predicted state
        e0_pred = predicted_output(:,1);
        e2_pred = predicted_output(:,2);
        q_pred = predicted_output(:,3);
        u_pred = predicted_output(:,4);
        w_pred = predicted_output(:,5);
        
        alpha_pred = atan2(w_pred, u_pred);
        alpha = atan2(w, u);

        quat_pred = [e0_pred zeros(size(e0_pred)) e2_pred zeros(size(e0_pred))]; % Model assumes only pitch movement
        eul_pred = quat2eul(quat_pred);

        roll_pred = eul_pred(:,3);
        pitch_pred = eul_pred(:,2);
        yaw_pred = eul_pred(:,1);

        % Plot
        fig = figure;
        if ~show_plot
            fig.Visible = 'off';
        end
        fig.Position = [100 100 900 900];
        numplots = 9;

        subplot(numplots,1,1)
        plot(t, rad2deg(roll_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(roll));
        end
        legend("\phi (not part of model)", "\phi")
        ylabel("[deg]")

        subplot(numplots,1,2)
        plot(t, rad2deg(pitch_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(pitch));
        end
        legend("\theta (predicted)", "\theta")
        ylabel("[deg]")
        
        subplot(numplots,1,3)
        plot(t, rad2deg(yaw_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(yaw));
        end
        legend("\psi (not part of model)", "\psi")
        ylabel("[deg]")
        
        subplot(numplots,1,4)
        plot(t, rad2deg(alpha_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(alpha));
        end
        legend("\alpha (predicted)", "\alpha")
        ylabel("[deg]")

        subplot(numplots,1,5)
        plot(t, rad2deg(q_pred))
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(q));
        end
        legend("q (predicted)", "q")
        ylabel("[deg/s]")

        subplot(numplots,1,6)
        plot(t, u_pred);
        if plot_actual_trajectory
            hold on
            plot(t, u);
        end
        legend("u (predicted)", "u")
        ylabel("[m/s]")

        subplot(numplots,1,7)
        plot(t, w_pred);
        if plot_actual_trajectory
            hold on
            plot(t, w);
        end
        legend("w (predicted)", "w")
        ylabel("[deg]")
        
        subplot(numplots,1,8)
        plot(t, rad2deg(input(:,1) - elevator_trim));
        legend("\delta_e")
        ylabel("[deg]")

        subplot(numplots,1,9)
        plot(t, input(:,2));
        legend("n_p")

        sgtitle(exp_name)
        
    elseif model_type == "lat"
        % Read predicted state
        e0_pred = predicted_output(:,1);
        e1_pred = predicted_output(:,2);
        e2_pred = predicted_output(:,3);
        e3_pred = predicted_output(:,4);
        p_pred = predicted_output(:,5);
        r_pred = predicted_output(:,6);
        v_pred = predicted_output(:,7);

        quat_pred = [e0_pred e1_pred e2_pred e3_pred];
        eul_pred = quat2eul(quat_pred);

        roll_pred = eul_pred(:,3);
        pitch_pred = eul_pred(:,2);
        yaw_pred = eul_pred(:,1);
        
        beta = asin(v ./ V_a);
        V_a_pred = sqrt(u .^ 2 + v_pred .^ 2 + w .^ 2);
        beta_pred = asin(v_pred ./ V_a_pred);

        % Plot
        fig = figure;
        if ~show_plot
            fig.Visible = 'off';
        end
        fig.Position = [100 100 600 600];
        num_plots = 15;

        subplot(num_plots,1,1)
        plot(t, rad2deg(roll_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(roll));
        end
        legend("\phi (predicted)", "\phi")
        ylabel("[deg]")

        subplot(num_plots,1,2)
        plot(t, rad2deg(pitch_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(pitch));
        end
        legend("\theta (predicted)", "\theta")
        ylabel("[deg]")

        subplot(num_plots,1,3)
        plot(t, rad2deg(yaw_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(yaw));
        end
        legend("\psi (predicted)", "\psi")
        ylabel("[deg]")
        
        subplot(num_plots,1,4)
        plot(t, rad2deg(beta_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(beta));
        end
        legend("\beta (predicted)", "\beta")
        ylabel("[deg]")

        subplot(num_plots,1,5)
        plot(t, rad2deg(p_pred));
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(p));
        end
        legend("p (predicted)", "p")
        ylabel("[deg/s]")

        subplot(num_plots,1,6)
        plot(t, rad2deg(r_pred));
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(r));
        end
        legend("r (predicted)", "r")
        ylabel("[deg/s]")

        subplot(num_plots,1,7)
        plot(t, v_pred);
        if plot_actual_trajectory
            hold on
            plot(t, v);
        end
        legend("v (predicted)", "v")
        ylabel("[m/s]")

        subplot(num_plots,1,8)
        plot(t, rad2deg(input(:,1) - aileron_trim));
        legend("\delta_a (trim subtracted)")
        ylabel("[deg]");

        subplot(num_plots,1,9)
        plot(t, rad2deg(input(:,2) - rudder_trim));
        legend("\delta_r (trim subtracted)")
        ylabel("[deg]");
        
        sgtitle(exp_name)
        
    elseif model_type == "full"
        % Read predicted state
        e0_pred = predicted_output(:,1);
        e1_pred = predicted_output(:,2);
        e2_pred = predicted_output(:,3);
        e3_pred = predicted_output(:,4);
        p_pred = predicted_output(:,5);
        q_pred = predicted_output(:,6);
        r_pred = predicted_output(:,7);
        u_pred = predicted_output(:,8);
        v_pred = predicted_output(:,9);
        w_pred = predicted_output(:,10);

        quat_pred = [e0_pred e1_pred e2_pred e3_pred];
        eul_pred = quat2eul(quat_pred);

        roll_pred = eul_pred(:,3);
        pitch_pred = eul_pred(:,2);
        yaw_pred = eul_pred(:,1); 
                
        alpha_pred = atan2(w_pred, u_pred);
        alpha = atan2(w, u);
        
        beta = asin(v ./ V_a);
        V_a_pred = sqrt(u_pred .^ 2 + v_pred .^ 2 + w_pred .^ 2);
        beta_pred = asin(v_pred ./ V_a_pred);

        % Plot
        fig = figure;
        if ~show_plot
            fig.Visible = 'off';
        end
        fig.Position = [100 100 1500 1000];
        num_plots = 9;

        subplot(num_plots,2,1)
        plot(t, rad2deg(roll_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(roll));
        end
        legend("\phi (predicted)", "\phi")
        ylabel("[deg]")

        subplot(num_plots,2,3)
        plot(t, rad2deg(pitch_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(pitch));
        end
        legend("\theta (predicted)", "\theta")
        ylabel("[deg]")

        subplot(num_plots,2,5)
        plot(t, rad2deg(yaw_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(yaw));
        end
        legend("\psi (predicted)", "\psi")
        ylabel("[deg]")
        
        subplot(num_plots,2,2)
        plot(t, rad2deg(alpha_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(alpha));
        end
        legend("\alpha (predicted)", "\alpha")
        ylabel("[deg]")

        subplot(num_plots,2,4)
        plot(t, rad2deg(beta_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(beta));
        end
        legend("\beta (predicted)", "\beta")
        ylabel("[deg]")

        subplot(num_plots,2,7)
        plot(t, rad2deg(p_pred));
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(p));
        end
        legend("p (predicted)", "p")
        ylabel("[deg/s]")
        
        subplot(num_plots,2,9)
        plot(t, rad2deg(q_pred));
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(q));
        end
        legend("q (predicted)", "q")
        ylabel("[deg/s]")

        subplot(num_plots,2,11)
        plot(t, rad2deg(r_pred));
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(r));
        end
        legend("r (predicted)", "r")
        ylabel("[deg/s]")

        subplot(num_plots,2,13)
        plot(t, u_pred);
        if plot_actual_trajectory
            hold on
            plot(t, u);
        end
        legend("u (predicted)", "u")
        ylabel("[m/s]")
        
        subplot(num_plots,2,15)
        plot(t, v_pred);
        if plot_actual_trajectory
            hold on
            plot(t, v);
        end
        legend("v (predicted)", "v")
        ylabel("[m/s]")
        
        subplot(num_plots,2,17)
        plot(t, w_pred);
        if plot_actual_trajectory
            hold on
            plot(t, w);
        end
        legend("w (predicted)", "w")
        ylabel("[m/s]")

        subplot(num_plots,2,6)
        plot(t, rad2deg(input(:,5) - aileron_trim));
        legend("\delta_a (trim subtracted)")
        ylabel("[deg]");
        
        subplot(num_plots,2,8)
        plot(t, rad2deg(input(:,6) - elevator_trim));
        legend("\delta_e")
        ylabel("[deg]");

        subplot(num_plots,2,10)
        plot(t, rad2deg(input(:,7) - rudder_trim));
        legend("\delta_r (trim subtracted)")
        ylabel("[deg]");
        
        sgtitle(exp_name)
    
    elseif model_type == "full_lat_fixed"
        % Read predicted state
        e0_pred = predicted_output(:,1);
        e1_pred = predicted_output(:,2);
        e2_pred = predicted_output(:,3);
        e3_pred = predicted_output(:,4);
        q_pred = predicted_output(:,5);
        u_pred = predicted_output(:,6);
        w_pred = predicted_output(:,7);
        
        p_pred = input(:,9);
        r_pred = input(:,10);
        v_pred = input(:,11);

        quat_pred = [e0_pred e1_pred e2_pred e3_pred];
        eul_pred = quat2eul(quat_pred);

        roll_pred = eul_pred(:,3);
        pitch_pred = eul_pred(:,2);
        yaw_pred = eul_pred(:,1); 
                
        alpha_pred = atan2(w_pred, u_pred);
        alpha = atan2(w, u);
        
        beta = asin(v ./ V_a);
        V_a_pred = sqrt(u_pred .^ 2 + v_pred .^ 2 + w_pred .^ 2);
        beta_pred = asin(v_pred ./ V_a_pred);

        % Plot
        fig = figure;
        if ~show_plot
            fig.Visible = 'off';
        end
        fig.Position = [100 100 1500 1000];
        num_plots = 9;

        subplot(num_plots,2,1)
        plot(t, rad2deg(roll_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(roll));
        end
        legend("\phi (predicted)", "\phi")
        ylabel("[deg]")

        subplot(num_plots,2,3)
        plot(t, rad2deg(pitch_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(pitch));
        end
        legend("\theta (predicted)", "\theta")
        ylabel("[deg]")

        subplot(num_plots,2,5)
        plot(t, rad2deg(yaw_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(yaw));
        end
        legend("\psi (predicted)", "\psi")
        ylabel("[deg]")
        
        subplot(num_plots,2,2)
        plot(t, rad2deg(alpha_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(alpha));
        end
        legend("\alpha (predicted)", "\alpha")
        ylabel("[deg]")

        subplot(num_plots,2,4)
        plot(t, rad2deg(beta_pred)); 
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(beta));
        end
        legend("\beta (predicted)", "\beta")
        ylabel("[deg]")

        subplot(num_plots,2,7)
        plot(t, rad2deg(p_pred));
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(p));
        end
        legend("p (predicted)", "p")
        ylabel("[deg/s]")
        
        subplot(num_plots,2,9)
        plot(t, rad2deg(q_pred));
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(q));
        end
        legend("q (predicted)", "q")
        ylabel("[deg/s]")

        subplot(num_plots,2,11)
        plot(t, rad2deg(r_pred));
        if plot_actual_trajectory
            hold on
            plot(t, rad2deg(r));
        end
        legend("r (predicted)", "r")
        ylabel("[deg/s]")

        subplot(num_plots,2,13)
        plot(t, u_pred);
        if plot_actual_trajectory
            hold on
            plot(t, u);
        end
        legend("u (predicted)", "u")
        ylabel("[m/s]")
        
        subplot(num_plots,2,15)
        plot(t, v_pred);
        if plot_actual_trajectory
            hold on
            plot(t, v);
        end
        legend("v (predicted)", "v")
        ylabel("[m/s]")
        
        subplot(num_plots,2,17)
        plot(t, w_pred);
        if plot_actual_trajectory
            hold on
            plot(t, w);
        end
        legend("w (predicted)", "w")
        ylabel("[m/s]")

        subplot(num_plots,2,6)
        plot(t, rad2deg(input(:,5) - aileron_trim));
        legend("\delta_a (trim subtracted)")
        ylabel("[deg]");
        
        subplot(num_plots,2,8)
        plot(t, rad2deg(input(:,6) - elevator_trim));
        legend("\delta_e")
        ylabel("[deg]");

        subplot(num_plots,2,10)
        plot(t, rad2deg(input(:,7) - rudder_trim));
        legend("\delta_r (trim subtracted)")
        ylabel("[deg]");
        
        subplot(num_plots,2,12)
        plot(t, input(:,8));
        legend("n_p");
        ylabel("[RPM]");
        
        sgtitle(exp_name)
    end
    
    if save_plots
        filename = exp_name;
        plot_location = model_path + "plots/";
        mkdir(plot_location);
        saveas(fig, plot_location + filename, 'epsc')
        %savefig(plot_location + filename + '.fig')
    end
end