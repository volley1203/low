clear;clc;close all

%% 初始参数
p = parameters();

%% 初始条件
N_dof = 11; % 总自由度
initial_state = zeros(N_dof * 2, 1); % 状态向量为 22x1
initial_state(1) = 0.5; % x
initial_state(3) = 0.5; % y
initial_state(5) = p.l0; % l

%% 仿真参数
t_start = 0;
t_end = 15;
tspan = [t_start t_end];

%% 由于电机输入
%desired_acceleration_profile = @(t) design_acceleration_profile(t);
control_input_profile = @(t) design_velocity_profile(t);

%% 使用ode45求解器进行仿真
tic;

% 设置ode45选项，包括事件检测
options = odeset('Events', @(t,y) limitEvents(t,y,p), 'RelTol', 1e-6, 'AbsTol', 1e-8);

% 初始化仿真状态
current_time = t_start;
current_state = initial_state;
all_time = [];
all_states = [];

while current_time < t_end
    current_tspan = [current_time, t_end];
    
    [t, y, te, ye, ie] = ode15s(@(t,y) nonlinear_dynamics_with_limits(t, y, p, control_input_profile), ...
                               current_tspan, current_state, options);
    
    % 保存结果 (忽略第一个重复点)
    if isempty(all_time)
        all_time = t;
        all_states = y;
    else
        all_time = [all_time; t(2:end)];
        all_states = [all_states; y(2:end,:)];
    end
    
    % 限位事件
    if ~isempty(te)
        fprintf('在时间 t=%.3f 发生限位事件，事件ID: %d\n', te(end), ie(end));
        current_state = ye(end, :)';
        current_time = te(end);
        event_idx = ie(end);
        switch event_idx
            case {1, 2}, current_state(2) = 0; fprintf('  -> X轴速度重置为0\n');
            case {3, 4}, current_state(4) = 0; fprintf('  -> Y轴速度重置为0\n');
            case {5, 6}, current_state(6) = 0; fprintf('  -> 绳长速度重置为0\n');
        end
    else
        break; % 正常结束
    end
end
fprintf('仿真完成，用时 %.2f 秒\n', toc);

%% 计算张力和其他输出
fprintf('计算张力和控制输出...\n');
num_steps = length(all_time);
outputs.tension = zeros(num_steps, 1);
outputs.control_inputs = zeros(num_steps, 6);
outputs.actual_acc = zeros(num_steps, 3);

for i = 1:num_steps
    t_i = all_time(i);
    state_i = all_states(i, :)';
    u_i = control_input_profile(t_i);
    
    % 调用动力学函数以获取实际加速度
    d_state_i = nonlinear_dynamics_with_limits(t_i, state_i, p, control_input_profile);
    
    % 提取平台实际加速度 (ddx, ddy, ddl)
    ddq = d_state_i(2:2:end); % 提取所有广义加速度
    outputs.actual_acc(i,:) = ddq(1:3)';
    
    % 存储控制输入
    outputs.control_inputs(i, :) = u_i';
    
    % 计算张力 
    outputs.tension(i) = tension(state_i, p, ddq, u_i);
end
fprintf('张力计算完成，范围: [%.2f, %.2f] N\n', min(outputs.tension), max(outputs.tension));

%% 三维可视化
animatecart(all_time, all_states(:,1), all_states(:,3), all_states(:,5), ...
           all_states(:,7), all_states(:,9), p);

%% 绘图
fprintf('生成结果图表...\n');

% 图1: 系统状态
figure('Name','系统动态响应','Position',[100 100 800 600]);
newcolor=colororder;
subplot(2,3,1);
plot(all_time, all_states(:,1), '-', 'Color', newcolor(1,:),'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,3), '-', 'Color', newcolor(2,:),'LineWidth', 1.5);
yline(p.x_min, 'b--', '$X_{min}$','Interpreter','latex'); yline(p.x_max, 'b--', '$X_{max}$','Interpreter','latex');
yline(p.y_min, 'r--', '$Y_{min}$','Interpreter','latex'); yline(p.y_max, 'r--', '$Y_{max}$','Interpreter','latex');
title('trolly location','Interpreter','latex'); xlabel('Time (s)','Interpreter','latex'); ylabel('Location (m)','Interpreter','latex');
legend('$X position$', '$Y position$', 'Location', 'best','Interpreter','latex'); grid on;

subplot(2,3,2);
plot(all_time, all_states(:,2), '-', 'Color', newcolor(3,:),'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,4), '-','Color', newcolor(4,:), 'LineWidth', 1.5);
title('trolly speed','Interpreter','latex'); xlabel('Time (s)','Interpreter','latex'); ylabel('Speed (m/s)','Interpreter','latex');
legend('X speed', 'Y speed', 'Location', 'best','Interpreter','latex'); grid on;

subplot(2,3,3);
plot(all_time, all_states(:,5), '-', 'Color', newcolor(5,:),'LineWidth', 1.5);
yline(p.l_min, 'r--', '$L_{min}$','Interpreter','latex'); yline(p.l_max, 'r--', '$L_{max}$','Interpreter','latex');
title('Sling length','Interpreter','latex'); xlabel('Time (s)','Interpreter','latex'); ylabel('Length (m)','Interpreter','latex');
legend('Sling length', 'Location', 'best','Interpreter','latex'); grid on;

subplot(2,3,4);
plot(all_time, all_states(:,7)*180/pi, '-', 'Color', [162, 20, 47]/255, 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,9)*180/pi, '--', 'Color', [217, 83, 25]/255, 'LineWidth', 1.5);
title('Swing angle','Interpreter','latex'); xlabel('Time (s)','Interpreter','latex'); ylabel('Angle (°)','Interpreter','latex');
legend('$\theta_x$', '$\theta_y$', 'Location', 'best','Interpreter','latex'); grid on;

subplot(2,3,5);
plot(all_time, all_states(:,8)*180/pi, '-', 'Color', [0, 191, 255]/255,'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,10)*180/pi, '--','Color', [210, 105, 30]/255, 'LineWidth', 1.5);
title('angular velocity','Interpreter','latex'); xlabel('Time (s)','Interpreter','latex'); ylabel('Angular velocity (°/second)','Interpreter','latex');
legend('$\omega_x$', '$\omega_y$', 'Location', 'best','Interpreter','latex'); grid on;

subplot(2,3,6);
plot(all_time, outputs.tension, '-', 'Color', [255, 69, 0]/255,'LineWidth', 1.5);
title('Sling tension','Interpreter','latex'); xlabel('Time (s)','Interpreter','latex'); ylabel('Tension (N)','Interpreter','latex');
grid on;

% 图2: 控制输入
figure('Name','控制输入','Position',[100 100 800 600]);
newcolor=colororder;
subplot(2,1,1);
plot(all_time, outputs.control_inputs(:,1), '-','Color', newcolor(1,:), 'LineWidth', 1.5); hold on;
plot(all_time, outputs.control_inputs(:,2), '-','Color', newcolor(2,:), 'LineWidth', 1.5);
plot(all_time, outputs.control_inputs(:,3), '-','Color', newcolor(3,:), 'LineWidth', 1.5);
title('Platform acceleration command','Interpreter','latex'); xlabel('Time (s)','Interpreter','latex'); ylabel('Acceleration (m/s²)','Interpreter','latex');
legend('$a_x$', '$a_y$', '$a_l$', 'Location', 'best','Interpreter','latex'); grid on;

subplot(2,1,2);
plot(all_time, outputs.control_inputs(:,4), '-','Color', newcolor(4,:), 'LineWidth', 1.5); hold on;
plot(all_time, outputs.control_inputs(:,5), '-','Color', newcolor(5,:), 'LineWidth', 1.5);
plot(all_time, outputs.control_inputs(:,6), '-','Color', newcolor(6,:), 'LineWidth', 1.5);
title('Mobile robot force input','Interpreter','latex'); xlabel('Time (s)','Interpreter','latex'); ylabel('Force (N)','Interpreter','latex');
legend('$f_{mx}$', '$f_{my}$', '$f_{mz}$', 'Location', 'best','Interpreter','latex'); grid on;

figure('Name','力和实际加速度','Position',[100 100 800 600]);
newcolor=colororder;
plot(all_time, outputs.actual_acc(:,1), '-','Color', newcolor(1,:), 'LineWidth', 1.5); hold on;
plot(all_time, outputs.actual_acc(:,2), '-','Color', newcolor(2,:), 'LineWidth', 1.5);
plot(all_time, outputs.actual_acc(:,3), '-','Color', newcolor(3,:), 'LineWidth', 1.5);
title('Platform actual acceleration','Interpreter','latex'); xlabel('Time (s)','Interpreter','latex'); ylabel('Acceleration (m/s²)','Interpreter','latex');
legend('$a_x$', '$a_y$', '$a_l$', 'Location', 'best','Interpreter','latex'); grid on;

% 图2: 柔性模态坐标
figure('Name', '柔性模态响应', 'Position', [100 100 800 600]);
newcolor=colororder;
% 提取模态坐标，索引为 11, 13, 15, 17, 19, 21
q_flex = all_states(:, 11:2:21);
subplot(2,1,1);
plot(all_time, q_flex(:, 1:3),'-','LineWidth', 1.5);
legend('$q_{x1}$', '$q_{x2}$', '$q_{x3}$','Interpreter','latex');
title('X-direction modal coordinates','Interpreter','latex'); grid on; xlabel('Time (s)','Interpreter','latex'); ylabel('Modal amplitude (m)','Interpreter','latex');
subplot(2,1,2);
plot(all_time, q_flex(:, 4:6),'-','LineWidth', 1.5);
legend('$q_{y1}$', '$q_{y2}$', '$q_{y3}$','Interpreter','latex');
title('Y-direction modal coordinates','Interpreter','latex'); grid on; xlabel('Time (s)','Interpreter','latex'); ylabel('Modal amplitude (m)','Interpreter','latex');

% 图2: 柔性模态坐标
figure('Name', '柔性模态叠加', 'Position', [100 100 800 600]);
newcolor=colororder;
% 提取模态坐标，索引为 11, 13, 15, 17, 19, 21
q_flex = all_states(:, 11:2:21);
q_flex_x_sum = sum(q_flex(:, 1:3), 2);
q_flex_y_sum = sum(q_flex(:, 4:6), 2);
plot(all_time, q_flex_x_sum,'-','LineWidth', 1.5);hold on;
%legend('$q_{x1}$', '$q_{x2}$', '$q_{x3}$','Interpreter','latex');
title('Vibration modal displacement','Interpreter','latex'); grid on; xlabel('Time (s)','Interpreter','latex'); ylabel('displacement (m)','Interpreter','latex');
plot(all_time, q_flex_y_sum,'-','LineWidth', 1.5);
legend('$X$-direction displacement','$Y$-direction displacement','Interpreter','latex');
%title('Y-direction modal coordinates','Interpreter','latex'); grid on; xlabel('Time (s)','Interpreter','latex'); ylabel('displacement (m)','Interpreter','latex');


%% 小车控制输入

function u = design_velocity_profile(t)
    %X方向: 目标速度0.2 m/s, 加速时间1s
    v_des_x = scurve_vel(t, 1.0, 5.0, 1.0, 0.2);
    %Y方向: 目标速度0.15 m/s, 加速时间0.8s
    v_des_y = scurve_vel(t, 1.0, 5.0, 0.8, 0.15);
    %Z方向(绳长): 目标速度-0.1 m/s (收绳), 加速时间1s
    v_des_l = scurve_vel(t, 7.0, 11.0, 1.0, -0.1);

    %施加于负载的主动力/扰动力 
    f_mx = 0; f_my = 0; f_mz = 0;
    if t >= 8.0 && t < 10.0
        f_mx = 5.0;
    end
    if t >= 11.0 && t < 12.0
        f_my = -4.0;
    end

    u = [v_des_x; v_des_y; v_des_l; f_mx; f_my; f_mz];
end

%生成s形速度曲线
function v = scurve_vel(t, t_start, t_end, t_accel, v_max)
    t_decel = t_accel; % 假设减速时间和加速时间相同

    if t < t_start || t > t_end
        v = 0;
        return;
    end

    % 加速段
    if t < (t_start + t_accel)
        % 将时间归一化到 [0, 1] 区间
        x = (t - t_start) / t_accel;
        % 使用 Smoothstep 多项式 (6x^5 - 15x^4 + 10x^3) 生成S形曲线
        scale = 6*x^5 - 15*x^4 + 10*x^3;
        v = v_max * scale;
    % 匀速段
    elseif t < (t_end - t_decel)
        v = v_max;
    % 减速段
    else
        % 将时间归一化到 [0, 1] 区间
        x = (t_end - t) / t_decel;
        % 同样使用 Smoothstep 多项式
        scale = 6*x^5 - 15*x^4 + 10*x^3;
        v = v_max * scale;
    end
end
% function u = design_velocity_profile(t)
%     % 目标: 控制底部小车(负载)做斜向的往复运动
%     % 方法: 同时施加 f_mx 和 f_my 力
% 
%     % 1. 设置顶层小车的期望速度为0，使其保持静止
%     v_des_x = 0;
%     v_des_y = 0;
%     v_des_l = 0;
% 
%     % 2. 设计施加于负载的往复力
%     force_magnitude = 5.0; % N, 可以调整这个力的大小
%     f_mx = 0;
%     f_my = 0;  % 初始化 f_my
%     f_mz = 0;
% 
%     % 在 1s 到 7.5s 之间施加一个正向的对角力 (+x, +y)
%     if t >= 1.0 && t < 7.5
%         f_mx = force_magnitude;
%         f_my = force_magnitude; % CHANGED: 增加y方向的力
%     % 在 7.5s 到 14s 之间施加一个反向的对角力 (-x, -y)
%     elseif t >= 7.5 && t < 14.0
%         f_mx = -force_magnitude;
%         f_my = -force_magnitude; % CHANGED: 增加y方向的力
%     end
% 
%     % 最终的控制输入向量
%     u = [v_des_x; v_des_y; v_des_l; f_mx; f_my; f_mz];
% end

function dydt = nonlinear_dynamics_with_limits(t, y, p, control_profile)
    % 带限位检查的动力学函数
    
    % 获取期望控制输入
    control_inputs = control_profile(t);
    
    % 检查是否在边界处，如果是则修改加速度指令
     x = y(1); y_pos = y(3); l = y(5);
    
    if (x <= p.x_min && control_inputs(1) < 0) || (x >= p.x_max && control_inputs(1) > 0)
        control_inputs(1) = 0;
    end
    if (y_pos <= p.y_min && control_inputs(2) < 0) || (y_pos >= p.y_max && control_inputs(2) > 0)
        control_inputs(2) = 0;
    end
    if (l <= p.l_min && control_inputs(3) < 0) || (l >= p.l_max && control_inputs(3) > 0)
        control_inputs(3) = 0;
    end
    
    dydt = nonlinear_dynamics(t, y, p, control_inputs);
end