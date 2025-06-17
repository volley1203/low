function d_state_vec = nonlinear_dynamics(t, state, p, control_inputs)
    %状态变量
    x = state(1);
    dx = state(2);
    y = state(3);
    dy = state(4);
    l = state(5);
    dl = state(6); 
    th_x = state(7); 
    dth_x = state(8);
    th_y = state(9);
    dth_y = state(10);
    
    v_des_x = control_inputs(1);
    v_des_y = control_inputs(2); 
    v_des_l = control_inputs(3); 
    f_mx = control_inputs(4);
    f_my = control_inputs(5); 
    f_mz = control_inputs(6); 

    %由一阶惯性环节计算加速度
    ddx = (v_des_x - dx) / p.tau_x;  
    ddy = (v_des_y - dy) / p.tau_y; 
    ddl = (v_des_l - dl) / p.tau_l;  
    
    m = p.m; mx = p.m_x; my = p.m_y; g = p.g;
    Dx = p.D_x; Dy = p.D_y; Dl = p.D_l; 

    sx = sin(th_x);cx = cos(th_x);
    sy = sin(th_y);cy = cos(th_y);
    
    %惯性矩阵（M）
    M = zeros(5,5);
    M(1,1) = mx+m; M(1,3) = m*sx; M(1,4) = m*l*cx;
    M(2,2) = my+m; M(2,4) = m*l*sy*cx; M(2,5)=m*l*cy*cx;
    M(3,1) = m*sx; M(3,2) = m*sy*cx; M(3,3) = m;
    M(4,1) = m*l*cx;M(4,2) = -m*l*sx*sy;M(4,4)=m*l^2;
    M(5,2) = m*l*cy*cx;M(5,5)=m*l^2*cx^2;

    %柯氏力矩阵
    C = zeros(5,5);
    C(1,3) = 2*m*cx*dth_x; C(1,4) = 2*m*cx*dl-m*l*sx*dth_x;
    C(2,3) = 2*m*(cy*cx*dth_y-sx*sy*dth_x);
    C(2,4) = -m*(l*sy*cx*dth_x+2*l*sx*cy*dth_y+2*sy*sx*dl);
    C(2,5) = -m*(l*cx*cy*dth_y+2*l*sx*cy*dth_x-2*sy*sx*dl);
    C(3,4) = -m*l*dth_x;C(3,5) = -m*l*cx^2*dth_y;
    C(4,1) = m*dl*cx-2*m*l*sx*dth_x;
    C(4,2) = -2*m*(l*sy*cx*dth_x+l*cy*sx*dth_y+sy*sx*dl);
    C(4,3) = m*(dx*cx+2*l*dth_x-2*dy*sx*sy);
    C(4,4) = 2*m*(l*dl-l*dx*sx-l*dy*sy*cx);
    C(4,5) = m*(l^2*sx*cx*dth_y-2*l*dy*cy*sx);
    C(5,2) = 2*m*(cy*cx*dl-l*sy*cx*dth_y-l*cy*sx*dth_x);
    C(5,3) = 2*m*(dy*cx*cy+l*cx^2*dth_y);
    C(5,4) = -2*m*(l*dy*cy*sx+l^2*cx*sx*dth_y);
    C(5,5) = -2*m*(l*dy*sy*cx+l^2*cx*sx*dth_x-l*cx^2*dl);

    %G
    G = zeros(5,1);
    G(3)=-m*g*cx*cy;
    G(4)=m*g*l*sx*cy;
    G(5)=m*g*l*cx*sy;
    
    %计算ddx与ddy
    M_ang_lhs = M(4:5, 4:5);%提取与角加速度相关的惯性项
    M_ang_rhs_terms_coeff = M(4:5, 1:3);%线性运动对角运动的耦合项

    Q_theta_x_payload = l*(f_mx*cx - f_my*sy*sx + f_mz*sx*cy);
    Q_theta_y_payload = l*cx*(f_my*cy + f_mz*sy);
    Q_payload = [Q_theta_x_payload; Q_theta_y_payload];
    q_dot = [dx; dy; dl; dth_x; dth_y];
    C_term_ang = C(4:5, :) * q_dot;
    %由一阶惯性环节输出的加速度
     known_platform_accelerations = [ddx; ddy; ddl];
     b_vector_ang = Q_payload - (C_term_ang + G(4:5) + ...
                   M_ang_rhs_terms_coeff * known_platform_accelerations);
   angular_accelerations = M_ang_lhs \ b_vector_ang;
    ddth_x = angular_accelerations(1);
    ddth_y = angular_accelerations(2);
    
    %ddX
    d_state_vec = zeros(10,1);
    d_state_vec(1) = dx;
    d_state_vec(2) = ddx; % Commanded
    d_state_vec(3) = dy;
    d_state_vec(4) = ddy; % Commanded
    d_state_vec(5) = dl;
    d_state_vec(6) = ddl; % Commanded
    d_state_vec(7) = dth_x;
    d_state_vec(8) = ddth_x;      % Calculated
    d_state_vec(9) = dth_y;
    d_state_vec(10) = ddth_y;     % Calculated
end