clear; clc; close all;

fprintf('=== 力控制演示：通过 f_mx, f_my, f_mz 控制四轮小车 ===\n');

%% 加载参数
p = parameters();

%% 初始条件
initial_state = zeros(10,1);
initial_state(1) = 0.1;  % 初始X位置
initial_state(3) = 0.1;  % 初始Y位置
initial_state(5) = p.l0; % 设置初始绳长

%% 仿真参数
t_start = 0;
t_end = 15;

%% 力控制轮廓设计
force_control_profile = @(t) design_force_control_inputs(t);

%% 使用ode45求解器进行仿真
tic;
options = odeset('Events', @(t,y) limitEvents(t,y,p), 'RelTol', 1e-6, 'AbsTol', 1e-8);

% 初始化仿真状态
current_time = t_start;
current_state = initial_state;
all_time = [];
all_states = [];

while current_time < t_end
    remaining_time = t_end - current_time;
    current_tspan = [current_time, current_time + remaining_time];
    
    [t, y, te, ye, ie] = ode45(@(t,y) nonlinear_dynamics_with_limits(t, y, p, force_control_profile), ...
                               current_tspan, current_state, options);
    
    all_time = [all_time; t];
    all_states = [all_states; y];
    
    if ~isempty(te)
        fprintf('在时间 t=%.3f 发生限位事件，事件ID: %d\n', te(end), ie(end));
        current_state = ye(end, :)';
        current_time = te(end);
        
        event_idx = ie(end);
        switch event_idx
            case {1, 2}
                current_state(2) = 0;
                fprintf('  -> X轴达到限位\n');
            case {3, 4}
                current_state(4) = 0;
                fprintf('  -> Y轴达到限位\n');
            case {5, 6}
                current_state(6) = 0;
                fprintf('  -> 绳长达到限位\n');
        end
        
        current_time = current_time + 1e-6;
    else
        break;
    end
end

fprintf('仿真完成，用时 %.2f 秒\n', toc);

%% 计算力控制输出
fprintf('计算力控制效果...\n');
num_steps = length(all_time);
outputs.control_inputs = zeros(num_steps, 6);
outputs.forces = zeros(num_steps, 3);

for i = 1:num_steps
    u_i = force_control_profile(all_time(i));
    outputs.control_inputs(i, :) = u_i';
    outputs.forces(i, :) = u_i(4:6)'; % f_mx, f_my, f_mz
end

fprintf('力控制分析完成\n');
fprintf('X方向力范围: [%.2f, %.2f] N\n', min(outputs.forces(:,1)), max(outputs.forces(:,1)));
fprintf('Y方向力范围: [%.2f, %.2f] N\n', min(outputs.forces(:,2)), max(outputs.forces(:,2)));
fprintf('Z方向力范围: [%.2f, %.2f] N\n', min(outputs.forces(:,3)), max(outputs.forces(:,3)));

%% 三维可视化
fprintf('\n启动力控制三维可视化...\n');
animatecart(all_time, all_states(:,1), all_states(:,3), all_states(:,5), ...
           all_states(:,7), all_states(:,9), p);

%% 绘制力控制分析图
fprintf('生成力控制分析图表...\n');

% 图1: 系统响应与力控制
figure('Name','力控制系统响应','Position',[100 100 1400 900]);

% 小车位置
subplot(3,3,1);
plot(all_time, all_states(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,3), 'r-', 'LineWidth', 1.5);
title('小车位置响应'); xlabel('时间 (s)'); ylabel('位置 (m)');
legend('X位置', 'Y位置', 'Location', 'best'); grid on;

% 摆动角度
subplot(3,3,2);
plot(all_time, all_states(:,7)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,9)*180/pi, 'r-', 'LineWidth', 1.5);
title('摆动角度响应'); xlabel('时间 (s)'); ylabel('角度 (度)');
legend('θ_x', 'θ_y', 'Location', 'best'); grid on;

% 摆动角速度
subplot(3,3,3);
plot(all_time, all_states(:,8)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,10)*180/pi, 'r-', 'LineWidth', 1.5);
title('摆动角速度'); xlabel('时间 (s)'); ylabel('角速度 (度/s)');
legend('ω_x', 'ω_y', 'Location', 'best'); grid on;

% X方向力控制
subplot(3,3,4);
plot(all_time, outputs.forces(:,1), 'b-', 'LineWidth', 2);
title('X方向力控制 (f_{mx})'); xlabel('时间 (s)'); ylabel('力 (N)');
grid on; ylim([-3, 3]);

% Y方向力控制
subplot(3,3,5);
plot(all_time, outputs.forces(:,2), 'r-', 'LineWidth', 2);
title('Y方向力控制 (f_{my})'); xlabel('时间 (s)'); ylabel('力 (N)');
grid on; ylim([-2.5, 2.5]);

% Z方向力控制
subplot(3,3,6);
plot(all_time, outputs.forces(:,3), 'g-', 'LineWidth', 2);
title('Z方向力控制 (f_{mz})'); xlabel('时间 (s)'); ylabel('力 (N)');
grid on; ylim([-1, 10]);

% 力与角度关系
subplot(3,3,7);
plot(all_time, outputs.forces(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,7)*180/pi*0.5, 'b--', 'LineWidth', 1.5);
title('X力与X摆角关系'); xlabel('时间 (s)'); ylabel('力(N) / 角度×0.5(度)');
legend('f_{mx}', 'θ_x×0.5', 'Location', 'best'); grid on;

subplot(3,3,8);
plot(all_time, outputs.forces(:,2), 'r-', 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,9)*180/pi*0.5, 'r--', 'LineWidth', 1.5);
title('Y力与Y摆角关系'); xlabel('时间 (s)'); ylabel('力(N) / 角度×0.5(度)');
legend('f_{my}', 'θ_y×0.5', 'Location', 'best'); grid on;

% 小车轨迹
subplot(3,3,9);
% 计算小车实际位置
x_vehicle = all_states(:,1) + all_states(:,5) .* sin(all_states(:,7));
y_vehicle = all_states(:,3) + all_states(:,5) .* sin(all_states(:,9)) .* cos(all_states(:,7));
plot(x_vehicle, y_vehicle, 'k-', 'LineWidth', 2); hold on;
plot(x_vehicle(1), y_vehicle(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(x_vehicle(end), y_vehicle(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
title('四轮小车运动轨迹'); xlabel('X位置 (m)'); ylabel('Y位置 (m)');
legend('轨迹', '起点', '终点', 'Location', 'best'); grid on; axis equal;

fprintf('力控制演示完成！\n');

%% 力控制输入设计函数
function control_inputs = design_force_control_inputs(t)
    % 专门设计的力控制输入：[ddx, ddy, ddl, f_mx, f_my, f_mz]
    
    % 平台运动控制（较小的加速度）
    ddx = 0;
    ddy = 0;
    ddl = 0;
    
    % 基础平台运动
    if t >= 1.0 && t < 2.0
        ddx = 0.02; % 小幅X方向加速度
    end
    
    if t >= 1.5 && t < 2.5
        ddy = 0.015; % 小幅Y方向加速度
    end
    
    % 主要通过力控制小车运动
    f_mx = 0;  % X方向力
    f_my = 0;  % Y方向力
    f_mz = 0;  % Z方向力
    
    % X方向力控制序列
    if t >= 2.0 && t < 4.0
        f_mx = 2.0; % 正X方向力，推动小车向右摆
    elseif t >= 5.0 && t < 7.0
        f_mx = -2.0; % 负X方向力，推动小车向左摆
    elseif t >= 8.0 && t < 9.0
        f_mx = 1.0; % 小幅正X方向力
    end
    
    % Y方向力控制序列
    if t >= 3.0 && t < 5.0
        f_my = 1.5; % 正Y方向力，推动小车向前摆
    elseif t >= 6.0 && t < 8.0
        f_my = -1.5; % 负Y方向力，推动小车向后摆
    elseif t >= 9.0 && t < 10.0
        f_my = 0.8; % 小幅正Y方向力
    end
    
    % Z方向力控制（低重力模拟）
    if t >= 4.0 && t < 12.0
        f_mz = 6.0; % 持续向上力，模拟低重力环境
    end
    
    control_inputs = [ddx; ddy; ddl; f_mx; f_my; f_mz];
end

function dydt = nonlinear_dynamics_with_limits(t, y, p, acceleration_profile)
    % 带限位检查的动力学函数
    control_inputs = acceleration_profile(t);
    
    x = y(1); vx = y(2);
    y_pos = y(3); vy = y(4);
    l = y(5); vl = y(6);
    
    % 限位检查
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