function [y_sim, error_calculations] = evaluate_ss_model(maneuver, ss_model)
    input_sequence = maneuver.get_lon_input_sequence();
    t_sim = maneuver.Time;
    y_0 = maneuver.get_lon_state_initial();
    y_recorded = maneuver.get_lon_state_sequence();
    
    empty_nonlin_model = NonlinearModel(zeros(5,3), zeros(6,3)); % get trim params
    u_nom = empty_nonlin_model.Params.u_nom;
    w_nom = empty_nonlin_model.Params.w_nom;
    delta_e_nom = empty_nonlin_model.Params.delta_e_nom;
    y_nom = [u_nom w_nom 0 0];
    y_0_perturbation = y_0 - y_nom;
    delta_u = input_sequence(:,1);

    [y_sim_perturbations, t_avl_ss_model] = lsim(ss_model, delta_u(:,1), t_sim, y_0_perturbation);
    y_sim = y_sim_perturbations + y_nom;
    error_calculations = calc_error_metrics(y_sim, y_recorded, maneuver.Id);
end
