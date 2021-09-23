function [y_sim, error_calculations] = evaluate_model(maneuver, model)
    input_sequence = maneuver.get_lon_input_sequence();
    t_data_seq = maneuver.Time;
    y_0 = maneuver.get_lon_state_initial();
    tspan = [maneuver.Time(1) maneuver.Time(end)];
    lat_state_seq = maneuver.get_lat_state_sequence();
    y_recorded = maneuver.get_lon_state_sequence();

    [t_sim, y_sim] = ode45(@(t,y) model.dynamics_lon_model(t, y, t_data_seq, input_sequence, lat_state_seq), tspan, y_0);
    y_sim = interp1(t_sim, y_sim, t_data_seq); % Get to correct time vector
    error_calculations = calc_error_metrics(y_sim, y_recorded ,maneuver.Id);
end