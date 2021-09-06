classdef ManeuverRawData
    properties
        Id
        ManeuverType
        Time
        QuatNedToBody
        EulPhi
        EulTheta
        EulPsi
        VelN
        VelE
        VelD
        VelBodyU
        VelBodyV
        VelBodyW
        TimeInput
        DeltaASp
        DeltaESp
        DeltaRSp
        DeltaT
    end
    methods
        function obj = ManeuverRawData(id, type)
            obj.Id = id;
            obj.ManeuverType = type;
        end
        function dt_s = calc_time_intervals(obj)
           dt_s = obj.Time(2:end) - obj.Time(1:end-1); 
        end
        function has_dropout = check_for_dropout(obj)
            dt_s = obj.calc_time_intervals();
            if max(dt_s) > 0.1
                has_dropout = true;
            else
                has_dropout = false;
            end
        end
        function plot(obj, show_plot, save_plot, filename, plot_location)
            % Plot
            fig = figure;
            if ~show_plot
                fig.Visible = 'off';
            end
            fig.Position = [100 100 1500 1000];
            num_plots_rows = 6;

            subplot(num_plots_rows,2,1)
            plot(obj.Time, rad2deg(obj.EulPhi)); 
            legend("$\phi$");
            ylabel("[deg]")
            ylim([-60 60])
            
            subplot(num_plots_rows,2,3)
            plot(obj.TimeInput, rad2deg(obj.DeltaASp), 'r--'); 
            legend("$\delta_a$");
            ylabel("[deg]")
            ylim([-28 28])
            
            subplot(num_plots_rows,2,5)
            plot(obj.Time, rad2deg(obj.EulTheta)); 
            legend("$\theta$");
            ylabel("[deg]")
            ylim([-30 30])
            
            subplot(num_plots_rows,2,7)
            plot(obj.TimeInput, rad2deg(obj.DeltaESp), 'r--'); 
            legend("$\delta_e$");
            ylabel("[deg]")
            ylim([-28 28])
            
            subplot(num_plots_rows,2,9)
            plot(obj.Time, rad2deg(obj.EulPsi)); 
            legend("$\psi$");
            ylabel("[deg]")
            psi_mean_deg = mean(rad2deg(obj.EulPsi));
            ylim([psi_mean_deg - 50 psi_mean_deg + 50])
            
            subplot(num_plots_rows,2,11)
            plot(obj.TimeInput, rad2deg(obj.DeltaRSp), 'r--'); 
            legend("$\delta_r$");
            ylabel("[deg]")
            ylim([-28 28])
            
            subplot(num_plots_rows,2,2)
            plot(obj.Time, obj.VelBodyU); 
            legend("$u$");
            ylabel("[m/s]")
            ylim([15 27])
            
            subplot(num_plots_rows,2,4)
            plot(obj.Time, obj.VelBodyV); 
            legend("$v$");
            ylabel("[m/s]")
            ylim([-7 7])
            
            subplot(num_plots_rows,2,6)
            plot(obj.Time, obj.VelBodyW); 
            legend("$w$");
            ylabel("[m/s]")
            ylim([-7 7])

            subplot(num_plots_rows,2,8)
            plot(obj.Time, obj.calc_airspeed()); 
            legend("$V$");
            ylabel("[m/s]")
            ylim([15 27])
            
            subplot(num_plots_rows,2,10)
            plot(obj.TimeInput, obj.DeltaT, 'r--'); 
            legend("$\delta_t$");
            ylabel("[rev/s]")
            ylim([0 130])

            sgtitle("Raw Maneuver Data: " + obj.ManeuverType);
            
            if save_plot
                saveas(fig, plot_location + filename, 'epsc')
            end
        end
        
        function save_plot(obj, plot_location)
            filename = "rawdata_plot_" + obj.ManeuverType + "_" + obj.Id;
            obj.plot(false, true, filename, plot_location);
        end
        
        function show_plot(obj)
            obj.plot(true, false, '', '');
        end
        
        function V = calc_airspeed(obj)
            V = sqrt(obj.VelBodyU .^ 2 + obj.VelBodyV .^ 2 + obj.VelBodyW .^ 2); 
        end
    end
end