% Part weights
battery_6500mah_kg = 0.950;
battery_11000mah_kg = 1.270;
wing_w_servo_kg = 0.450;
top_esc_kg = 0.160;
tail_w_servos_kg = 0.630;
fuselage_w_avionics_and_cf_tubes_kg = 2.310;
vtol_arm_w_wiring_kg = 0.280;
top_motor_w_prop_kg = 0.260;
pusher_motor_w_prop_kg = 0.350;

total_weight_kg = 2 * battery_6500mah_kg + 3 * battery_11000mah_kg ...
    + 2 * wing_w_servo_kg + 4 * top_esc_kg + tail_w_servos_kg ...
    + fuselage_w_avionics_and_cf_tubes_kg + 2 * vtol_arm_w_wiring_kg ...
    + 4 * top_motor_w_prop_kg + pusher_motor_w_prop_kg;

% Mass
mass_kg = 12.140;
cg_position_from_front_mm = 494.31;