clear;clc;close all

%% 初始参数
p = parameters();

%% 初始条件
initial_state = zeros(10,1);
initial_state(1) = 0.5;
initial_state(3) = 0.5;
initial_state(5) = p.l0; % 设置初始绳长

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
    % 计算剩余仿真时间
    remaining_time = t_end - current_time;
    current_tspan = [current_time, current_time + remaining_time];
    
    % 运行ode45直到下一个事件或仿真结束
    [t, y, te, ye, ie] = ode45(@(t,y) nonlinear_dynamics_with_limits(t, y, p, control_input_profile), ...
                               current_tspan, current_state, options);
    
    % 保存结果
    all_time = [all_time; t];
    all_states = [all_states; y];
    
    % 检查是否发生了限位事件
   if ~isempty(te)
        fprintf('在时间 t=%.3f 发生限位事件，事件ID: %d\n', te(end), ie(end));
        
        current_state = ye(end, :)';
        current_time = te(end);
        
        % 硬限位处理：将相应方向的速度设为0
        event_idx = ie(end);
        switch event_idx
            case {1, 2} % X 轴限位
                current_state(2) = 0;
                fprintf('  -> X轴达到限位，X方向速度重置为0\n');
            case {3, 4} % Y 轴限位
                current_state(4) = 0;
                fprintf('  -> Y轴达到限位，Y方向速度重置为0\n');
            case {5, 6} % 绳长限位
                current_state(6) = 0;
                fprintf('  -> 绳长达到限位，绳长变化速度重置为0\n');
        end
        
        % 继续仿真
        current_time = current_time + 1e-6; % 微小时间步进避免重复触发
    else
        % 正常结束
        break;
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
    
    % 获取控制输入
    u_i = control_input_profile(t_i);
    
    % 调用动力学函数以获取实际加速度
    d_state_i = nonlinear_dynamics_with_limits(t_i, state_i, p, control_input_profile);
    
    % 存储实际的平台加速度
    outputs.actual_acc(i,:) = d_state_i([2, 4, 6])';
    
    % 存储控制输入
    outputs.control_inputs(i, :) = u_i';
    
    % 计算张力
    outputs.tension(i) = tension(t_i, state_i, p, ...
                                 outputs.actual_acc(i,1), ...
                                 outputs.actual_acc(i,2), ...
                                 outputs.actual_acc(i,3), ...
                                 u_i(4), u_i(5), u_i(6));
end

fprintf('张力计算完成\n');
fprintf('张力范围: [%.2f, %.2f] N\n', min(outputs.tension), max(outputs.tension));

%% 三维可视化
animatecart(all_time, all_states(:,1), all_states(:,3), all_states(:,5), ...
           all_states(:,7), all_states(:,9), p);

%% 绘图
fprintf('生成结果图表...\n');

% 图1: 系统状态
figure('Name','系统动态响应','Position',[100 100 1200 800]);
subplot(2,3,1);
plot(all_time, all_states(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,3), 'r-', 'LineWidth', 1.5);
yline(p.x_min, 'b--', 'X_{min}'); yline(p.x_max, 'b--', 'X_{max}');
yline(p.y_min, 'r--', 'Y_{min}'); yline(p.y_max, 'r--', 'Y_{max}');
title('小车位置'); xlabel('时间 (s)'); ylabel('位置 (m)');
legend('X位置', 'Y位置', 'Location', 'best'); grid on;

subplot(2,3,2);
plot(all_time, all_states(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,4), 'r-', 'LineWidth', 1.5);
title('小车速度'); xlabel('时间 (s)'); ylabel('速度 (m/s)');
legend('X速度', 'Y速度', 'Location', 'best'); grid on;

subplot(2,3,3);
plot(all_time, all_states(:,5), 'g-', 'LineWidth', 1.5);
yline(p.l_min, 'g--', 'L_{min}'); yline(p.l_max, 'g--', 'L_{max}');
title('绳长'); xlabel('时间 (s)'); ylabel('长度 (m)');
legend('绳长', 'Location', 'best'); grid on;

subplot(2,3,4);
plot(all_time, all_states(:,7)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,9)*180/pi, 'r-', 'LineWidth', 1.5);
title('摆动角度'); xlabel('时间 (s)'); ylabel('角度 (度)');
legend('θ_x', 'θ_y', 'Location', 'best'); grid on;

subplot(2,3,5);
plot(all_time, all_states(:,8)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, all_states(:,10)*180/pi, 'r-', 'LineWidth', 1.5);
title('摆动角速度'); xlabel('时间 (s)'); ylabel('角速度 (度/s)');
legend('ω_x', 'ω_y', 'Location', 'best'); grid on;

subplot(2,3,6);
plot(all_time, outputs.tension, 'k-', 'LineWidth', 1.5);
title('绳索张力'); xlabel('时间 (s)'); ylabel('张力 (N)');
grid on;

% 图2: 控制输入
figure('Name','控制输入','Position',[150 150 800 600]);
subplot(2,1,1);
plot(all_time, outputs.control_inputs(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, outputs.control_inputs(:,2), 'r-', 'LineWidth', 1.5);
plot(all_time, outputs.control_inputs(:,3), 'g-', 'LineWidth', 1.5);
title('平台加速度指令'); xlabel('时间 (s)'); ylabel('加速度 (m/s²)');
legend('ddcx', 'ddcy', 'ddcl', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(all_time, outputs.control_inputs(:,4), 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, outputs.control_inputs(:,5), 'r-', 'LineWidth', 1.5);
plot(all_time, outputs.control_inputs(:,6), 'g-', 'LineWidth', 1.5);
title('主动施加于负载的力'); xlabel('时间 (s)'); ylabel('力 (N)');
legend('f_{mx}', 'f_{my}', 'f_{mz}', 'Location', 'best'); grid on;

figure('Name','力和实际加速度','Position',[150 150 800 600]);
subplot(2,1,1);
plot(all_time, outputs.actual_acc(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, outputs.actual_acc(:,2), 'r-', 'LineWidth', 1.5);
plot(all_time, outputs.actual_acc(:,3), 'g-', 'LineWidth', 1.5);
title('平台实际加速度'); xlabel('时间 (s)'); ylabel('加速度 (m/s²)');
legend('ddx', 'ddy', 'lddl', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(all_time, outputs.control_inputs(:,4), 'b-', 'LineWidth', 1.5); hold on;
plot(all_time, outputs.control_inputs(:,5), 'r-', 'LineWidth', 1.5);
plot(all_time, outputs.control_inputs(:,6), 'g-', 'LineWidth', 1.5);
title('主动施加于负载的力'); xlabel('时间 (s)'); ylabel('力 (N)');
legend('f_{mx}', 'f_{my}', 'f_{mz}', 'Location', 'best'); grid on;

fprintf('仿真和绘图完成！\n');
%% 小车控制输入

function u = design_velocity_profile(t)
    % 设计期望的速度轨迹 (梯形速度曲线)
    % u = [v_des_x; v_des_y; v_des_l; f_mx; f_my; f_mz]
    
    % X方向: 目标速度0.2 m/s, 加速时间1s
    v_des_x = trapezoidal_vel(t, 1.0, 5.0, 1.0, 0.2);
    % Y方向: 目标速度0.15 m/s, 加速时间0.8s
    v_des_y = trapezoidal_vel(t, 1.0, 5.0, 0.8, 0.15);
    % Z方向(绳长): 目标速度-0.1 m/s (收绳), 加速时间1s
    v_des_l = trapezoidal_vel(t, 7.0, 11.0, 1.0, -0.1);

    % 施加于负载的主动力/扰动力 (这部分不变)
    f_mx = 0; f_my = 0; f_mz = 0;
    if t >= 8.0 && t < 10.0
        f_mx = 5.0;
    end
    if t >= 11.0 && t < 12.0
        f_my = -4.0;
    end
    
    u = [v_des_x; v_des_y; v_des_l; f_mx; f_my; f_mz];
end

% 辅助函数：生成梯形速度曲线
function v = trapezoidal_vel(t, t_start, t_end, t_accel, v_max)
    t_decel = t_accel;
    if t < t_start || t > t_end
        v = 0;
    elseif t < (t_start + t_accel) % 加速段
        v = v_max * (t - t_start) / t_accel;
    elseif t < (t_end - t_decel) % 匀速段
        v = v_max;
    else % 减速段
        v = v_max * (t_end - t) / t_decel;
    end
end

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