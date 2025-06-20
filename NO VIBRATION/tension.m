function T_rope = tension(current_t, current_state_vec, p, actual_ddx, actual_ddy, actual_ddl, f_mx, f_my, f_mz)
    l_i = current_state_vec(5);
    dl_i = current_state_vec(6);
    th_x_i = current_state_vec(7);
    dth_x_i = current_state_vec(8);
    th_y_i = current_state_vec(9);
    dth_y_i = current_state_vec(10);

    sx_i = sin(th_x_i); cx_i = cos(th_x_i);
    sy_i = sin(th_y_i); cy_i = cos(th_y_i);
    payload_force_on_tension = f_mx * sx_i + f_my * sy_i * cx_i - f_mz * cx_i * cy_i;

     T_rope = p.m * actual_ddl ...
           + p.m * actual_ddx * sx_i ...
           + p.m * actual_ddy * sy_i * cx_i ...
           - p.m * l_i * dth_x_i^2 ...
           - p.m * l_i * dth_y_i^2 * cx_i^2 ...
           - p.m * p.g * cx_i * cy_i ...
           + p.D_l * dl_i ...
           - payload_force_on_tension;
end