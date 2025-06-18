function d_state_vec = closedloop(t, state, p, control_inputs)
   %  从state 向量中提取所有状态
    % 刚体部分
    dx = state(2); dy = state(4); dl = state(6);
    dth_x = state(8); dth_y = state(10);
    % 柔性部分
    dqx1 = state(12); dqx2 = state(14); dqx3 = state(16);
    dqy1 = state(18); dqy2 = state(20); dqy3 = state(22);

    % 提取控制输入
    v_des_x = control_inputs(1); v_des_y = control_inputs(2);
    v_des_l = control_inputs(3); f_mx = control_inputs(4);
    f_my = control_inputs(5); f_mz = control_inputs(6);

    % 一阶惯性环节
    ddx_known = (v_des_x - dx) / p.tau_x;
    ddy_known = (v_des_y - dy) / p.tau_y;
    ddl_known = (v_des_l - dl) / p.tau_l;
    ddq_known = [ddx_known; ddy_known; ddl_known];

    % 
    q = [state(1); state(3); state(5); state(7); state(9); state(11); state(13); ...
        state(15); state(17); state(19); state(21)];
    dq = [state(2); state(4); state(6); state(8); state(10); state(12); state(14);... 
        state(16); state(18); state(20); state(22)];
    
    % 调用 M, C, G 矩阵函数
    param_vec = [p.m_x; p.m_y; p.m; p.rho; p.g; p.T_tension_val];
    M = M_func(q, param_vec);
    C = C_func(q, dq, param_vec);
    G = G_func(q, param_vec);

    %构建广义力 Q_ext 
    Q_ext = zeros(11, 1);
    sx = sin(q(4)); cx = cos(q(4)); sy = sin(q(5)); cy = cos(q(5)); l = q(3);
    Q_ext(1) = f_mx - p.D_x * dq(1);
    Q_ext(2) = f_my - p.D_y * dq(2);
    Q_ext(3) = (f_mx*sx + f_my*sy*cx - f_mz*cx*cy) - p.D_l * dq(3);
    Q_ext(4) = l*(f_mx*cx - f_my*sy*sx + f_mz*sx*cy);
    Q_ext(5) = l*cx*(f_my*cy + f_mz*sy);
    L0 = p.l0; m_k_modal = 0.5 * p.rho * L0;
    for k = 1:3
        % Modal stiffness (same for x and y)
        K_k = p.T_tension_val * (k * pi)^2 / (2 * L0);
        
        % Calculate separate damping coefficients for x and y directions
        c_kx = 2 * p.zeta_kx(k) * sqrt(K_k * m_k_modal);
        c_ky = 2 * p.zeta_ky(k) * sqrt(K_k * m_k_modal);
        
        % Apply damping force for x-direction modes (qx1, qx2, qx3)
        Q_ext(5+k) = -c_kx * dq(5+k);
        
        % Apply damping force for y-direction modes (qy1, qy2, qy3)
        Q_ext(8+k) = -c_ky * dq(8+k);
    end

    % 计算加速度 
    p_indices = 1:3;   % x, y, l 
    u_indices = 4:11;  % thetas, q_k's 
    
    M_uu = M(u_indices, u_indices);
    M_up = M(u_indices, p_indices);
    
    RHS = Q_ext(u_indices) - G(u_indices) - C(u_indices, :) * dq - M_up * ddq_known;
    ddq_unknown = M_uu \ RHS;

    % 11个广义加速度 ddq
    ddq = zeros(11, 1);
    ddq(p_indices) = ddq_known;
    ddq(u_indices) = ddq_unknown;

    % 
    d_state_vec = zeros(22, 1);
    d_state_vec(1) = dq(1);   % dx
    d_state_vec(2) = ddq(1);  % ddx
    d_state_vec(3) = dq(2);   % dy
    d_state_vec(4) = ddq(2);  % ddy
    d_state_vec(5) = dq(3);   % dL
    d_state_vec(6) = ddq(3);  % ddL
    d_state_vec(7) = dq(4);   % dtheta_x
    d_state_vec(8) = ddq(4);  % ddtheta_x
    d_state_vec(9) = dq(5);   % dtheta_y
    d_state_vec(10) = ddq(5); % ddtheta_y
    % 模态部分
    for k=1:6
        d_state_vec(10 + 2*k - 1) = dq(5+k);
        d_state_vec(10 + 2*k)     = ddq(5+k);
    end
end