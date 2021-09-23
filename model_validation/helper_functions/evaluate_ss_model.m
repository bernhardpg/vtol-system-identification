function [y_sim, error_calculations] = evaluate_ss_model(maneuver, ss_model, model_type)
    t_sim = maneuver.Time;
    if strcmp(model_type, "longitudinal")
        % Some more work on longitudinal model as it assumes perturbation
        % quantities
        input_sequence = maneuver.get_lon_input_sequence();
        y_0 = maneuver.get_lon_state_initial();
        y_recorded = maneuver.get_lon_state_sequence();
        empty_nonlin_model = NonlinearModel(zeros(5,3), zeros(6,3)); % get trim params
        u_nom = empty_nonlin_model.Params.u_nom;
        w_nom = empty_nonlin_model.Params.w_nom;
        y_nom = [u_nom w_nom 0 0];
        y_0_perturbation = y_0 - y_nom;
        delta_u = input_sequence(:,1);
    
        [y_sim_perturbations, ~] = lsim(ss_model, delta_u, t_sim, y_0_perturbation);
        y_sim = y_sim_perturbations + y_nom;
    elseif strcmp(model_type, "lateral-directional")    
        input_sequence = maneuver.get_lat_input_sequence();
        y_0 = maneuver.get_lat_state_initial();
        y_recorded = maneuver.get_lat_state_sequence();
    
        [y_sim, ~] = lsim(ss_model, input_sequence, t_sim, y_0);
    end

    error_calculations = calc_error_metrics(y_sim, y_recorded, maneuver.Id);
end
