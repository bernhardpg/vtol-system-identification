classdef FlightPathData
    properties
        Id
        ManeuverType
        Time
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
        DeltaRSp
        RawData = ManeuverRawData.empty
    end
    methods
        % Construct a FlightPathData object from a ManeuverRawData
        function obj = FlightPathData(id, maneuver_type)
            obj.Id = id;
            obj.ManeuverType = maneuver_type;
            obj.RawData = ManeuverRawData(id, maneuver_type);
        end
        function obj = calc_fpr_from_rawdata(obj, dt_desired, dt_knots)
            obj.Time = (obj.RawData.Time(1):dt_desired:obj.RawData.Time(end))';
            
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
            obj.PDot = calc_spline_derivative(obj.Time, obj.AngP, obj.Time, dt_knots, 0);
            obj.QDot = calc_spline_derivative(obj.Time, obj.AngQ, obj.Time, dt_knots, 0);
            obj.RDot = calc_spline_derivative(obj.Time, obj.AngR, obj.Time, dt_knots, 0);
            
            % Save actuator setpoints by doing linear interpolation to
            % correct time frame
            obj.DeltaASp = interp1(obj.RawData.TimeInput, obj.RawData.DeltaASp, obj.Time);
            obj.DeltaESp = interp1(obj.RawData.TimeInput, obj.RawData.DeltaESp, obj.Time);
            obj.DeltaRSp = interp1(obj.RawData.TimeInput, obj.RawData.DeltaRSp, obj.Time);
            obj.DeltaT = interp1(obj.RawData.TimeInput, obj.RawData.DeltaT, obj.Time);
            
            % Estimate actuator deflections
            obj.DeltaA = simulate_control_surface_dynamics(obj.Time, obj.DeltaASp);
            obj.DeltaE = simulate_control_surface_dynamics(obj.Time, obj.DeltaESp);
            obj.DeltaR = simulate_control_surface_dynamics(obj.Time, obj.DeltaRSp);
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
            plot(obj.Time, rad2deg(obj.DeltaA), obj.Time, rad2deg(obj.DeltaASp), ':'); 
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
            plot(obj.Time, rad2deg(obj.DeltaE), obj.Time, rad2deg(obj.DeltaESp), ':'); 
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
            plot(obj.Time, rad2deg(obj.DeltaR), obj.Time, rad2deg(obj.DeltaRSp), ':'); 
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
                saveas(fig, plot_location + obj.ManeuverType + "_" + filename, 'epsc')
            end
        end
        
        function save_plot(obj, filename, plot_location)
            obj.plot(false, true, filename, plot_location);
        end
        
        function show_plot(obj)
            obj.plot(true, false, '', '');
        end
        
        function V = calc_airspeed(obj)
            V = sqrt(obj.VelU .^ 2 + obj.VelV .^ 2 + obj.VelW .^ 2); 
        end
        
        function check_kinematic_consistency(obj)
            [t_sim, y_sim] = simulate_kinematics(obj.Time, obj.EulPhi, obj.EulTheta, obj.EulPsi, obj.VelU, obj.VelV, obj.VelW, obj.AngP, obj.AngQ, obj.AngR, obj.AccX, obj.AccY, obj.AccZ);
            phi_sim = y_sim(:,1);
            theta_sim = y_sim(:,2);
            psi_sim = y_sim(:,3);
            u_sim = y_sim(:,4);
            v_sim = y_sim(:,5);
            w_sim = y_sim(:,6);
           
            fig.Position = [100 100 1500 1000];
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
        end
    end
end