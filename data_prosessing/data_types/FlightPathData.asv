classdef FlightPathData
    properties
        Id
        ManeuverType
        Time
        Dt
        Alpha
        AlphaAbsolute
        Beta
        VelTot
        EulPhi
        EulTheta
        EulPsi
        VelU
        VelV
        VelW
        AngP
        AngQ
        AngR
        PDot
        QDot
        RDot
        AccX
        AccY
        AccZ
        DeltaA
        DeltaE
        DeltaR
        DeltaT
        DeltaASp
        DeltaESp
        DeltaESpAbsolute
        DeltaRSp
        C_X
        C_Y
        C_Z
        C_L
        C_D
        C_l
        C_m
        C_n
        VelUHat
        VelVHat
        VelWHat
        AngPHat
        AngQHat
        AngRHat
        RawData = ManeuverRawData.empty
        DeltaETrim
        AlphaTrim
    end
    methods
        % Construct a FlightPathData object from a ManeuverRawData
        function obj = FlightPathData(id, maneuver_type)
            obj.Id = id;
            obj.ManeuverType = maneuver_type;
            obj.RawData = ManeuverRawData(id, maneuver_type);
            
            airframe_static_properties;
            obj.DeltaETrim = delta_e_nom;
            obj.AlphaTrim = alpha_nom;
        end
        function obj = calc_fpr_from_rawdata(obj, dt_desired, dt_knots)
            obj.Time = (obj.RawData.Time(1):dt_desired:obj.RawData.Time(end))';
            obj.Dt = dt_desired;
            
            % Smooth eul angles and calculate derivatives
            eul_smoothed = smooth_signal([obj.RawData.EulPhi obj.RawData.EulTheta obj.RawData.EulPsi]);
            obj.EulPhi = calc_spline_derivative(obj.RawData.Time, eul_smoothed(:,1), obj.Time, dt_knots, 0);
            phi_dot = calc_spline_derivative(obj.RawData.Time, eul_smoothed(:,1), obj.Time, dt_knots, 1);
            obj.EulTheta = calc_spline_derivative(obj.RawData.Time, eul_smoothed(:,2), obj.Time, dt_knots, 0);
            theta_dot = calc_spline_derivative(obj.RawData.Time, eul_smoothed(:,2), obj.Time, dt_knots, 1);
            obj.EulPsi = calc_spline_derivative(obj.RawData.Time, eul_smoothed(:,3), obj.Time, dt_knots, 0);
            psi_dot = calc_spline_derivative(obj.RawData.Time, eul_smoothed(:,3), obj.Time, dt_knots, 1);
            
            % Smooth body vel and calculate derivatives
            body_vel_smoothed = smooth_signal([obj.RawData.VelBodyU obj.RawData.VelBodyV obj.RawData.VelBodyW]);
            obj.VelU = calc_spline_derivative(obj.RawData.Time, body_vel_smoothed(:,1), obj.Time, dt_knots, 0);
            u_dot = calc_spline_derivative(obj.RawData.Time, body_vel_smoothed(:,1), obj.Time, dt_knots, 1);
            obj.VelV = calc_spline_derivative(obj.RawData.Time, body_vel_smoothed(:,2), obj.Time, dt_knots, 0);
            v_dot = calc_spline_derivative(obj.RawData.Time, body_vel_smoothed(:,2), obj.Time, dt_knots, 1);
            obj.VelW = calc_spline_derivative(obj.RawData.Time, body_vel_smoothed(:,3), obj.Time, dt_knots, 0);
            w_dot = calc_spline_derivative(obj.RawData.Time, body_vel_smoothed(:,3), obj.Time, dt_knots, 1);
            
            % Calc ang veloticies from kinematic relationships
            [obj.AngP, obj.AngQ, obj.AngR] = calc_ang_vel(obj.EulPhi, obj.EulTheta, phi_dot, theta_dot, psi_dot);

            % Calc translational accelerations from kinematic relationships
            airframe_static_properties; % to get g constant
            [obj.AccX, obj.AccY, obj.AccZ] = calc_trans_acc(g, obj.EulPhi, obj.EulTheta, obj.AngP, obj.AngQ, obj.AngR, obj.VelU, obj.VelV, obj.VelW, u_dot, v_dot, w_dot);
            
            % Calculate angular accelerations
            obj.PDot = calc_spline_derivative(obj.Time, obj.AngP, obj.Time, dt_knots, 1);
            obj.QDot = calc_spline_derivative(obj.Time, obj.AngQ, obj.Time, dt_knots, 1);
            obj.RDot = calc_spline_derivative(obj.Time, obj.AngR, obj.Time, dt_knots, 1);
            
            % Save actuator setpoints by doing linear interpolation to
            % correct time frame
            obj.DeltaASp = interp1(obj.RawData.TimeInput, obj.RawData.DeltaASp, obj.Time);
            obj.DeltaESpAbsolute = interp1(obj.RawData.TimeInput, obj.RawData.DeltaESp, obj.Time);
            obj.DeltaESp = obj.DeltaESpAbsolute - obj.DeltaETrim;
            obj.DeltaRSp = interp1(obj.RawData.TimeInput, obj.RawData.DeltaRSp, obj.Time);
            obj.DeltaT = interp1(obj.RawData.TimeInput, obj.RawData.DeltaT, obj.Time);
            
            % Estimate actuator deflections
            obj.DeltaA = simulate_control_surface_dynamics(obj.Time, obj.DeltaASp);
            obj.DeltaE = simulate_control_surface_dynamics(obj.Time, obj.DeltaESp);
            obj.DeltaR = simulate_control_surface_dynamics(obj.Time, obj.DeltaRSp);
            
            % Calculate aerodynamic angles
            obj.VelTot = sqrt(obj.VelU .^ 2 + obj.VelV .^ 2 + obj.VelW .^ 2);
            obj.AlphaAbsolute = atan2(obj.VelW, obj.VelU);
            obj.Alpha = obj.AlphaAbsolute - obj.AlphaTrim;
            obj.Beta = asin(obj.VelV ./ obj.VelTot);
        end
        
        function plot(obj, show_plot, save_plot, filename, plot_location)
            fig = figure;
            if ~show_plot
                fig.Visible = 'off';
            end
            fig.Position = [100 100 1500 1000];
            num_plots_rows = 9;

            subplot(num_plots_rows,2,1)
            plot(obj.Time, rad2deg(obj.EulPhi)); 
            legend("$\phi$");
            ylabel("[deg]")
            ylim([-60 60])
            
            subplot(num_plots_rows,2,3)
            plot(obj.Time, rad2deg(obj.AngP))
            legend("$p$");
            ylim([-3.5*180/pi 3.5*180/pi]);
            ylabel("[deg/s]")
            
            subplot(num_plots_rows,2,5)
            plot(obj.Time, rad2deg(obj.DeltaA), obj.Time, rad2deg(obj.DeltaASp), '--'); 
            legend("$\delta_a$", "$\delta_a^{sp}$");
            ylabel("[deg]")
            ylim([-28 28])
            
            subplot(num_plots_rows,2,7)
            plot(obj.Time, rad2deg(obj.EulTheta)); 
            legend("$\theta$");
            ylabel("[deg]")
            ylim([-30 30])
            
            subplot(num_plots_rows,2,9)
            plot(obj.Time, rad2deg(obj.AngQ))
            legend("$q$");
            ylim([-2*180/pi 2*180/pi]);
            ylabel("[deg/s]")
            
            subplot(num_plots_rows,2,11)
            plot(obj.Time, rad2deg(obj.DeltaE), obj.Time, rad2deg(obj.DeltaESp), '--'); 
            legend("$\delta_e$", "$\delta_e^{sp}$");
            ylabel("[deg]")
            ylim([-28 28])
            
            subplot(num_plots_rows,2,13)
            plot(obj.Time, rad2deg(obj.EulPsi)); 
            legend("$\psi$");
            ylabel("[deg]")
            psi_mean_deg = mean(rad2deg(obj.EulPsi));
            ylim([psi_mean_deg - 50 psi_mean_deg + 50])
            
            subplot(num_plots_rows,2,15)
            plot(obj.Time, rad2deg(obj.AngR))
            legend("$r$");
            ylim([-2*180/pi 2*180/pi]);
            ylabel("[deg/s]")
            
            subplot(num_plots_rows,2,17)
            plot(obj.Time, rad2deg(obj.DeltaR), obj.Time, rad2deg(obj.DeltaRSp), '--'); 
            legend("$\delta_r$", "$\delta_r^{sp}$");
            ylabel("[deg]")
            ylim([-28 28])
            
            subplot(num_plots_rows,2,2)
            plot(obj.Time, obj.VelU); 
            legend("$u$");
            ylabel("[m/s]")
            ylim([15 27])
            
            subplot(num_plots_rows,2,4)
            plot(obj.Time, obj.VelV); 
            legend("$v$");
            ylabel("[m/s]")
            ylim([-7 7])
            
            subplot(num_plots_rows,2,6)
            plot(obj.Time, obj.VelW); 
            legend("$w$");
            ylabel("[m/s]")
            ylim([-7 7])

            subplot(num_plots_rows,2,8)
            plot(obj.Time, obj.calc_airspeed()); 
            legend("$V$");
            ylabel("[m/s]")
            ylim([15 27])
            
            subplot(num_plots_rows,2,10)
            plot(obj.Time, obj.DeltaT, 'r--'); 
            legend("$\delta_t$");
            ylabel("[rev/s]")
            ylim([0 130])

            sgtitle("Raw Maneuver Data: " + obj.ManeuverType);
            
            if save_plot
                saveas(fig, plot_location + filename, 'epsc')
            end
        end
        
        function save_plot(obj, plot_location)
            filename = "plot_" + obj.ManeuverType + "_" + obj.Id;
            obj.plot(false, true, filename, plot_location);
        end
        
        function show_plot(obj)
            obj.plot(true, false, '', '');
        end
        
        function V = calc_airspeed(obj)
            V = sqrt(obj.VelU .^ 2 + obj.VelV .^ 2 + obj.VelW .^ 2); 
        end
        
        function plot_lateral(obj, show_plot, save_plot, filename, plot_location)
            fig = figure;
            if ~show_plot
                fig.Visible = 'off';
            end
            fig.Position = [100 100 1500 1000];
            num_plots_rows = 7;

            subplot(num_plots_rows,2,1)
            plot(obj.Time, rad2deg(obj.EulPhi)); 
            legend("$\phi$");
            ylabel("[deg]")
            ylim([-60 60])
            
            subplot(num_plots_rows,2,3)
            plot(obj.Time, rad2deg(obj.AngP))
            legend("$p$");
            ylim([-3.5*180/pi 3.5*180/pi]);
            ylabel("[deg/s]")
            
            subplot(num_plots_rows,2,5)
            plot(obj.Time, rad2deg(obj.DeltaA), obj.Time, rad2deg(obj.DeltaASp), '--'); 
            legend("$\delta_a$", "$\delta_a^{sp}$");
            ylabel("[deg]")
            ylim([-28 28])
            
            subplot(num_plots_rows,2,7)
            plot(obj.Time, rad2deg(obj.EulPsi)); 
            legend("$\psi$");
            ylabel("[deg]")
            psi_mean_deg = mean(rad2deg(obj.EulPsi));
            ylim([psi_mean_deg - 50 psi_mean_deg + 50])
            
            subplot(num_plots_rows,2,9)
            plot(obj.Time, rad2deg(obj.AngR))
            legend("$r$");
            ylim([-2*180/pi 2*180/pi]);
            ylabel("[deg/s]")
            
            subplot(num_plots_rows,2,11)
            plot(obj.Time, rad2deg(obj.DeltaR), obj.Time, rad2deg(obj.DeltaRSp), '--'); 
            legend("$\delta_r$", "$\delta_r^{sp}$");
            ylabel("[deg]")
            ylim([-28 28])
            
            subplot(num_plots_rows,2,2)
            plot(obj.Time, obj.VelV); 
            legend("$v$");
            ylabel("[m/s]")
            ylim([-7 7])
            
            subplot(num_plots_rows,2,4)
            plot(obj.Time, obj.AccY); 
            legend("$a_y$");
            ylabel("[m/s^2]")
            ylim([-4 4])
            
            subplot(num_plots_rows,2,6)
            plot(obj.Time, obj.PDot); 
            legend("$\dot{p}$");
            ylabel("[rad/s^2]")
            ylim([-8 8])
            
            subplot(num_plots_rows,2,8)
            plot(obj.Time, obj.RDot); 
            legend("$\dot{r}$");
            ylabel("[rad/s^2]")
            ylim([-8 8])
            
            subplot(num_plots_rows,2,10)
            plot(obj.Time, obj.C_Y); 
            legend("$c_y$");
            ylabel("")
            
            subplot(num_plots_rows,2,12)
            plot(obj.Time, obj.C_l); 
            legend("$c_l$");
            ylabel("")
            
            subplot(num_plots_rows,2,14)
            plot(obj.Time, obj.C_n); 
            legend("$c_n$");
            ylabel("")
            
            
            
            sgtitle("Raw Maneuver Data: " + obj.ManeuverType);
            
            if save_plot
                saveas(fig, plot_location + filename, 'epsc')
            end
        end
        
        function plot_longitudinal(obj, show_plot, save_plot, filename, plot_location)
            fig = figure;
            if ~show_plot
                fig.Visible = 'off';
            end
            fig.Position = [100 100 1500 1000];
            num_plots_rows = 7;

            subplot(num_plots_rows,2,1)
            plot(obj.Time, rad2deg(obj.EulTheta)); 
            legend("$\theta$");
            ylabel("[deg]")
            ylim([-50 50])
            
            subplot(num_plots_rows,2,3)
            plot(obj.Time, rad2deg(obj.AngQ))
            legend("$q$");
            ylim([-200 200]);
            ylabel("[deg/s]")
            
            subplot(num_plots_rows,2,5)
            plot(obj.Time, rad2deg(obj.DeltaE), obj.Time, rad2deg(obj.DeltaESp), '--'); 
            legend("$\delta_e$", "$\delta_e^{sp}$");
            ylabel("[deg]")
            ylim([-28 28])
            
            subplot(num_plots_rows,2,7)
            plot(obj.Time, obj.VelU); 
            legend("$u$");
            ylabel("[m/s]")
            ylim([0 30])
            
            subplot(num_plots_rows,2,9)
            plot(obj.Time, obj.VelW)
            legend("$w$");
            ylabel("m/s]")
            ylim([-7 7])
            
            subplot(num_plots_rows,2,11)
            plot(obj.Time, obj.DeltaT)
            legend("$\delta_t$");
            ylim([0 130])
            ylabel("[rev/s]")

            
            subplot(num_plots_rows,2,2)
            plot(obj.Time, rad2deg(obj.Alpha)); 
            legend("$\alpha$");
            ylabel("[deg]")
            ylim([-30 30])
            
            subplot(num_plots_rows,2,4)
            plot(obj.Time, obj.AccX); 
            legend("$a_x$");
            ylabel("[m/s^2]")
 %           ylim([-4 4])
            
            subplot(num_plots_rows,2,6)
            plot(obj.Time, obj.AccZ); 
            legend("$a_z$");
            ylabel("[m/s^2]")
%            ylim([-])
            
            subplot(num_plots_rows,2,8)
            plot(obj.Time, obj.QDot); 
            legend("$\dot{q}$");
            ylabel("[rad/s^2]")
            %ylim([-8 8])
            
            subplot(num_plots_rows,2,10)
            plot(obj.Time, obj.C_X); 
            legend("$c_X$");
            ylabel("")
            
            subplot(num_plots_rows,2,12)
            plot(obj.Time, obj.C_Z); 
            legend("$c_Z$");
            ylabel("")
            
            subplot(num_plots_rows,2,14)
            plot(obj.Time, obj.C_m); 
            legend("$c_m$");
            ylabel("")
            
            
            
            sgtitle("Raw Maneuver Data: " + obj.ManeuverType);
            
            if save_plot
                saveas(fig, plot_location + filename, 'epsc')
            end
        end
        
        function save_plot_longitudinal(obj, plot_location)
            filename = "plot_" + obj.ManeuverType + "_" + obj.Id;
            obj.plot_longitudinal(false, true, filename, plot_location);
        end
        
        function show_plot_longitudinal(obj)
            obj.plot_longitudinal(true, false, '', '');
        end
        
        function save_plot_lateral(obj, plot_location)
            filename = "plot_" + obj.ManeuverType + "_" + obj.Id;
            obj.plot_lateral(false, true, filename, plot_location);
        end
        
        function show_plot_lateral(obj)
            obj.plot_lateral(true, false, '', '');
        end
        
        function obj = calc_force_coeffs(obj)
                [obj.C_X, obj.C_Y, obj.C_Z, obj.C_L, obj.C_D] = calc_force_coeffs(obj.VelU, obj.VelV, obj.VelW, obj.AccX, obj.AccY, obj.AccZ, obj.DeltaT); % For now, lift and drag coeff is not used for anything
        end
        
        function obj = calc_moment_coeffs(obj)
                [obj.C_l, obj.C_m, obj.C_n] = calc_moment_coeffs(obj.AngP, obj.AngQ, obj.AngR, obj.VelU, obj.VelV, obj.VelW, obj.PDot, obj.QDot, obj.RDot); 
        end
        
        function obj = calc_explanatory_vars(obj)
            airframe_static_properties; % Get V_nom, wingspan and MAC

            obj.VelUHat = obj.VelU / V_nom;
            obj.VelVHat = obj.VelV / V_nom;
            obj.VelWHat = obj.VelW / V_nom;
            obj.AngPHat = obj.AngP * (wingspan_m / (2 * V_nom));
            obj.AngQHat = obj.AngQ * (mean_aerodynamic_chord_m / (2 * V_nom));
            obj.AngRHat = obj.AngR * (wingspan_m / (2 * V_nom));
 
        end
        
        function check_kinematic_consistency(obj, show_plot, save_plot, plot_location)
            [t_sim, y_sim] = simulate_kinematics(obj.Time, obj.EulPhi, obj.EulTheta, obj.EulPsi, obj.VelU, obj.VelV, obj.VelW, obj.AngP, obj.AngQ, obj.AngR, obj.AccX, obj.AccY, obj.AccZ);
            phi_sim = y_sim(:,1);
            theta_sim = y_sim(:,2);
            psi_sim = y_sim(:,3);
            u_sim = y_sim(:,4);
            v_sim = y_sim(:,5);
            w_sim = y_sim(:,6);
           
            fig = figure;
            fig.Position = [100 100 1500 1000];
            if ~show_plot
                fig.Visible = 'off';
            end
            num_plots_rows = 3;

            subplot(num_plots_rows,2,1)
            plot(obj.RawData.Time, rad2deg(obj.RawData.EulPhi), '--', t_sim, rad2deg(phi_sim)); 
            legend("$\phi$ (measured)", "$\phi$");
            ylabel("[deg]")
            ylim([-60 60])
            
            subplot(num_plots_rows,2,3)
            plot(obj.RawData.Time, rad2deg(obj.RawData.EulTheta), '--', t_sim, rad2deg(theta_sim)); 
            legend("$\theta$ (measured)", "$\theta$");
            ylabel("[deg]")
            ylim([-30 30])
            
            subplot(num_plots_rows,2,5)
            plot(obj.RawData.Time, rad2deg(obj.RawData.EulPsi), '--', t_sim, rad2deg(psi_sim)); 
            legend("$\psi$ (measured)", "$\psi$");
            ylabel("[deg]")
            psi_mean_deg = mean(rad2deg(obj.EulPsi));
            ylim([psi_mean_deg - 50 psi_mean_deg + 50])
            
            subplot(num_plots_rows,2,2)
            plot(obj.RawData.Time, obj.RawData.VelBodyU, '--', t_sim, u_sim); 
            legend("$u$ (measured)", "$u$");
            ylabel("[m/s]")
            ylim([15 27])
            
            subplot(num_plots_rows,2,4)
            plot(obj.RawData.Time, obj.RawData.VelBodyV, '--', t_sim, v_sim); 
            legend("$v$ (measured)", "$v$");
            ylabel("[m/s]")
            ylim([-7 7])
            
            subplot(num_plots_rows,2,6)
            plot(obj.RawData.Time, obj.RawData.VelBodyW, '--', t_sim, w_sim); 
            legend("$w$ (measured)", "$w$");
            ylabel("[m/s]")
            ylim([-7 7])

            sgtitle("Kinematic Consistency Check: " + obj.ManeuverType + " id: " + obj.Id);
            
            if save_plot
                filename = "kin_consistency_check_" + obj.Id;
                saveas(fig, plot_location + obj.ManeuverType + "_" + filename, 'epsc');
            end
        end
        
        function x_lat_0 = get_lat_state_initial(obj)
            x_lat_0 = [obj.VelV(1) obj.AngP(1) obj.AngR(1) obj.EulPhi(1)];
        end
                
        function x_lon_0 = get_lon_state_initial(obj)
            x_lon_0 = [obj.VelU(1) obj.VelW(1) obj.AngQ(1) obj.EulTheta(1)];
        end
        
        function x_lat = get_lat_state_sequence(obj)
            x_lat = [obj.VelV obj.AngP obj.AngR obj.EulPhi];
        end
        
        function x_lon = get_lon_state_sequence(obj)
            x_lon = [obj.VelU obj.VelW obj.AngQ obj.EulTheta];
        end
        
        function u_lat = get_lat_input_sequence(obj)
            u_lat = [obj.DeltaA obj.DeltaR];
        end
        
        function u_lon = get_lon_input_sequence(obj)
            u_lon = [obj.DeltaE obj.DeltaT];
        end
        
        
        function plot_lateral_validation(obj, t_sim, simulated_responses, model_names, plot_styles, show_plot, save_plot, filename, plot_location)
            fig = figure;
            if ~show_plot
                fig.Visible = 'off';
            end
            fig.Position = [100 100 1500 1000];
            num_plots_rows = 4;
            num_models = numel(t_sim);

            subplot(num_plots_rows,2,1)
            plot(obj.Time, obj.VelV, plot_styles(1)); hold on;
            for i = 1:num_models
                plot(t_sim{i}, simulated_responses{i}(:,1), plot_styles(i+1)); hold on
            end
            legend(model_names);
            ylabel("[m/s]")
            ylim([-13 13])
            title("v")

            subplot(num_plots_rows,2,3)
            plot(obj.Time, rad2deg(obj.AngP), plot_styles(1)); hold on
            for i = 1:num_models
                plot(t_sim{i}, rad2deg(simulated_responses{i}(:,2)), plot_styles(i+1)); hold on
            end
            legend(model_names);
            ylim([-3.5*180/pi 3.5*180/pi]);
            ylabel("[deg/s]")
            title("p")
                        
            subplot(num_plots_rows,2,5)
            plot(obj.Time, rad2deg(obj.AngR), plot_styles(1)); hold on
            for i = 1:num_models
                plot(t_sim{i}, rad2deg(simulated_responses{i}(:,3)), plot_styles(i+1)); hold on
            end
            legend(model_names);
            ylim([-3.5*180/pi 3.5*180/pi]);
            ylabel("[deg/s]")
            title("r")
                        
            subplot(num_plots_rows,2,7)
            plot(obj.Time, rad2deg(obj.EulPhi), plot_styles(1)); hold on
            for i = 1:num_models
                plot(t_sim{i}, rad2deg(simulated_responses{i}(:,4)), plot_styles(i+1)); hold on
            end
            legend(model_names);
            ylabel("[deg]")
            ylim([-60 60])
            title("\phi")
                        
            subplot(num_plots_rows,2,2)
            plot(obj.Time, detrend(rad2deg(obj.DeltaA),0), obj.Time, detrend(rad2deg(obj.DeltaASp),0), '--'); 
            legend("$\delta_a$", "$\delta_a^{sp}$");
            ylabel("[deg]")
            ylim([-28 28])
            title("Aileron")
            
            subplot(num_plots_rows,2,4)
            plot(obj.Time, detrend(rad2deg(obj.DeltaR),0), obj.Time, detrend(rad2deg(obj.DeltaRSp),0), '--'); 
            legend("$\delta_r$", "$\delta_r^{sp}$");
            ylabel("[deg]")
            ylim([-28 28])
            title("Rudder")

            sgtitle("Model validation on " + obj.ManeuverType);
            
            if save_plot
                saveas(fig, plot_location + filename, 'epsc')
            end
        end
        
        function save_plot_lateral_validation(obj, t_sim, simulated_responses, model_names, plot_styles, plot_location)
            filename = "plot_" + obj.ManeuverType + "_" + obj.Id;
            obj.plot_lateral_validation(t_sim, simulated_responses, model_names, plot_styles, false, true, filename, plot_location);
        end
        
        function show_plot_lateral_validation(obj, t_sim, simulated_responses, model_names, plot_styles)
            obj.plot_lateral_validation(t_sim, simulated_responses, model_names, plot_styles, true, false, '', '');
        end
        
        
        function plot_longitudinal_validation(obj, t_sim, simulated_responses, model_names, plot_styles, show_plot, save_plot, filename, plot_location)
            fig = figure;
            if ~show_plot
                fig.Visible = 'off';
            end
            fig.Position = [100 100 1500 1000];
            num_plots_rows = 4;
            num_models = numel(t_sim);

            subplot(num_plots_rows,2,1)
            plot(obj.Time, obj.VelU, plot_styles(1)); hold on;
            for i = 1:num_models
                plot(t_sim{i}, simulated_responses{i}(:,1), plot_styles(i+1)); hold on
            end
            %legend(model_names);
            ylabel("[m/s]")
            ylim([0 28])
            title("u")

            subplot(num_plots_rows,2,3)
            plot(obj.Time, obj.VelW, plot_styles(1)); hold on
            for i = 1:num_models
                plot(t_sim{i}, simulated_responses{i}(:,2), plot_styles(i+1)); hold on
            end
            %legend(model_names);
            ylabel("[m/s]")
            ylim([-10 10])
            title("w")
                        
            subplot(num_plots_rows,2,5)
            plot(obj.Time, rad2deg(obj.AngQ), plot_styles(1)); hold on
            for i = 1:num_models
                plot(t_sim{i}, rad2deg(simulated_responses{i}(:,3)), plot_styles(i+1)); hold on
            end
            %legend(model_names);
            ylim([-200 200]);
            ylabel("[deg/s]")
            title("q")
                        
            subplot(num_plots_rows,2,7)
            plot(obj.Time, rad2deg(obj.EulTheta), plot_styles(1)); hold on
            for i = 1:num_models
                plot(t_sim{i}, rad2deg(simulated_responses{i}(:,4)), plot_styles(i+1)); hold on
            end
            legend(model_names);
            ylabel("[deg]")
            ylim([-50 50])
            title("\theta")
                        
            subplot(num_plots_rows,2,2)
            plot(obj.Time, rad2deg(obj.DeltaE), obj.Time, rad2deg(obj.DeltaESp), '--'); 
            legend("$\delta_e$", "$\delta_e^{sp}$");
            ylabel("[deg]")
            ylim([-28 28])
            title("Elevator")
            
            subplot(num_plots_rows,2,4)
            plot(obj.Time, obj.DeltaT); 
            legend("$\delta_t$");
            ylabel("[rev/s]")
            ylim([0 150])
            title("Throttle")

            sgtitle("Model validation on " + obj.ManeuverType);
            
            if save_plot
                saveas(fig, plot_location + filename, 'epsc')
            end
        end
        
        function save_plot_longitudinal_validation(obj, t_sim, simulated_responses, model_names, plot_styles, plot_location)
            filename = "plot_" + obj.ManeuverType + "_" + obj.Id;
            obj.plot_longitudinal_validation(t_sim, simulated_responses, model_names, plot_styles, false, true, filename, plot_location);
        end
        
        function show_plot_longitudinal_validation(obj, t_sim, simulated_responses, model_names, plot_styles)
            obj.plot_longitudinal_validation(t_sim, simulated_responses, model_names, plot_styles, true, false, '', '');
        end
        
    end
end