function [t_sim, y_sim] = simulate_kinematics(t_time_series, phi, theta, psi, u, v, w, p, q, r, a_x, a_y, a_z)
    y0 = [phi(1) theta(1) psi(1) u(1) v(1) w(1)];
    ang_vel = [p q r];
    acc_body = [a_x, a_y, a_z]; 
    tspan = [t_time_series(1) t_time_series(end)];
    airframe_static_properties; % import g
    [t_sim, y_sim] = ode23s(@(t, y) f_dyn(t, y, t_time_series, ang_vel, acc_body, g), tspan, y0);
end

% State = [eul, vel_body]
% Input = [ang_vel, acc_body]
function dy_dt = f_dyn(t_curr, y, t_input_series, ang_vel, acc_body, g)
    % Interpolate inputs to current time
    ang_vel_at_t = interp1(t_input_series, ang_vel, t_curr);
    acc_body_at_t = interp1(t_input_series, acc_body, t_curr);
    
    % Calculate derivatives
    eul = y(1:3);
    vel_body = y(4:6);
    dAngVel_dt = rotational_kinematics(eul, ang_vel_at_t);
    dVelBody_dt = translational_kinematics(eul, vel_body, ang_vel_at_t, acc_body_at_t, g);
    
    dy_dt = [dAngVel_dt
             dVelBody_dt];
end


function dAngVel_dt = rotational_kinematics(eul, ang_vel)
    phi = eul(1);
    theta = eul(2);
    
    p = ang_vel(1);
    q = ang_vel(2);
    r = ang_vel(3);

    phi_dot = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot = (q * sin(phi) + r * cos(phi)) * sec(theta);
    
    dAngVel_dt = [phi_dot
                  theta_dot
                  psi_dot];
end

function dVelBody_dt = translational_kinematics(eul, vel_body, ang_vel, acc_body, g)
    phi = eul(1);
    theta = eul(2);
    
    p = ang_vel(1);
    q = ang_vel(2);
    r = ang_vel(3);
    
    u = vel_body(1);
    v = vel_body(2);
    w = vel_body(3);
    
    a_x = acc_body(1);
    a_y = acc_body(2);
    a_z = acc_body(3);
    
    u_dot = r * v - q * w - g * sin(theta) + a_x;
    v_dot = p * w - r * u + g * cos(theta) * sin(phi) + a_y;
    w_dot = q * u - p * v + g * cos(theta) * cos(phi) + a_z;
    
    dVelBody_dt = [u_dot v_dot w_dot]';
end