function p = parameters()
    p.g = 9.81;
    p.m = 10.0;
    p.m_x = 30.157;
    p.m_y_trolley = 30.157;
    p.m_y_rail = 30.437;
    p.m_y = p.m_y_trolley+p.m_y_rail;
    %阻尼力
    p.D_x = 20.371;
    p.D_y = 23.652;
    p.D_l = 21.528;
    p.l0 = 2.5;
    p.zeta_k = [0.02;0.02;0.02];
    %移动限位
    p.x_min=0.0;
    p.x_max=1.5;%m
    p.y_min=0;
    p.y_max=2;
    p.l_min=0.1;
    p.l_max=2.55;
    %电机时间常数
    p.tau_x = 0.15;
    p.tau_y = 0.1;
    p.tau_l = 0.08;
end