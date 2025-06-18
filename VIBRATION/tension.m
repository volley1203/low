function T_rope = tension(current_state_vec, p, ddq, control_inputs)
    % 所需的物理参数
    m = p.m;
    g = p.g;
    
    % 
    L       = current_state_vec(5);
    theta_x = current_state_vec(7);
    dtheta_x= current_state_vec(8);
    theta_y = current_state_vec(9);
    dtheta_y= current_state_vec(10);
    
    %提取平台加速度
    ddx = ddq(1);
    ddy = ddq(2);
    ddL = ddq(3);
    
    % 绝对加速度 a_rope
    sx = sin(theta_x); cx = cos(theta_x);
    sy = sin(theta_y); cy = cos(theta_y);
    
    a_rope = ddL + sx*ddx + cx*sy*ddy - L*(dtheta_x^2 + (dtheta_y*cx)^2);
    
    % 计算重力在绳索方向的分量 Fg_rope
    g_component = g * cx * cy;
    
    % 求解最终张力 T_rope
    T_rope = m * (g_component + a_rope);
    
    % 施加物理约束
    if T_rope < 0
        T_rope = 0;
    end

end