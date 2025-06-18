clear;clc;close all

%% 定义重力情景
g_moon = 1.62;       % 月球重力 (m/s^2)
g_mars = 3.71;       % 火星重力 (m/s^2)
g_earth = 9.81;      % 地球重力 (m/s^2)
gravity_scenarios = struct('Moon', g_moon, 'Mars', g_mars, 'Earth', g_earth);
scenario_names = fieldnames(gravity_scenarios);

%% 创建一个结构体来存储所有实验的结果
results = struct();

%% 循环执行每个重力情景的仿真
for i = 1:length(scenario_names)
    scenario_name = scenario_names{i};
    g_current = gravity_scenarios.(scenario_name);
    fprintf('====== 开始仿真: %s 重力环境 (g = %.2f m/s^2) ======\n', scenario_name, g_current);
    
    p = parameters();
    p.g = g_current;
    p.T_tension_val = p.m * p.g;
    
    N_dof = 11;
    initial_state = zeros(N_dof * 2, 1);
    initial_state(1) = 1; initial_state(3) = 1; initial_state(5) = p.l0;

    t_start = 0; t_end = 30;
    tspan = [t_start, t_end];
    
    control_input_profile = @(t) design_velocity_profile(t);
    
    options = odeset('Events', @(t,y) limitEvents(t,y,p), 'RelTol', 1e-6, 'AbsTol', 1e-8);
    
    current_time = t_start;
    current_state = initial_state;
    all_time = [];
    all_states = [];
    while current_time < t_end
        current_tspan = [current_time, t_end];
        [t, y, te, ye, ie] = ode45(@(t,y) nonlinear_dynamics_with_limits(t, y, p, control_input_profile), current_tspan, current_state, options);
        if isempty(all_time), all_time = t; all_states = y;
        else, all_time = [all_time; t(2:end)]; all_states = [all_states; y(2:end,:)]; end
        if ~isempty(te)
            fprintf('在时间 t=%.3f 发生限位事件，事件ID: %d\n', te(end), ie(end));
            current_state = ye(end, :)'; current_time = te(end); event_idx = ie(end);
            switch event_idx
                case {1, 2}, current_state(2) = 0;
                case {3, 4}, current_state(4) = 0;
                case {5, 6}, current_state(6) = 0;
            end
        else, break; end
    end
    fprintf('仿真完成\n');
    results.(scenario_name).time = all_time;
    results.(scenario_name).states = all_states;
end

%%
% --- 准备工作：创建公共时间轴用于数据对比 ---
t_common = linspace(tspan(1), tspan(2), 5000)'; % 创建一个高密度的时间轴
p = parameters(); L0 = p.l0;
phi_vals_at_midpoint = [1; 0; -1]; % sin(k*pi/2) for k=1,2,3

% --- 1. 摆动角度 (θ_x) 分析 ---
fprintf('\n\n--- 1. 峰值摆动角度 (θ_x) 分析 ---\n');
% 插值得到在公共时间轴上的数据
angle_earth = interp1(results.Earth.time, abs(results.Earth.states(:,7)*180/pi), t_common);
angle_moon = interp1(results.Moon.time, abs(results.Moon.states(:,7)*180/pi), t_common);
angle_mars = interp1(results.Mars.time, abs(results.Mars.states(:,7)*180/pi), t_common);

% 月球 vs 地球
diff_moon = angle_moon - angle_earth;
[max_diff_moon, idx_moon] = max(diff_moon);
t_diff_moon = t_common(idx_moon);
fprintf('\n[月球 vs 地球]:\n');
fprintf(' -> 最大角度差异发生在 %.2f 秒附近 (区间: %.2fs - %.2fs)。\n', t_diff_moon, t_diff_moon-0.5, t_diff_moon+0.5);
fprintf(' -> 在该时刻, 月球摆角比地球大 %.2f 度。\n', max_diff_moon);
fprintf(' -> 从各自峰值来看, 月球(%.2f度)相比地球(%.2f度)的最大摆角增大了【%.1f%%】。\n', max(angle_moon), max(angle_earth), (max(angle_moon)/max(angle_earth)-1)*100);

% 火星 vs 地球
diff_mars = angle_mars - angle_earth;
[max_diff_mars, idx_mars] = max(diff_mars);
t_diff_mars = t_common(idx_mars);
fprintf('\n[火星 vs 地球]:\n');
fprintf(' -> 最大角度差异发生在 %.2f 秒附近 (区间: %.2fs - %.2fs)。\n', t_diff_mars, t_diff_mars-0.5, t_diff_mars+0.5);
fprintf(' -> 在该时刻, 火星摆角比地球大 %.2f 度。\n', max_diff_mars);
fprintf(' -> 从各自峰值来看, 火星(%.2f度)相比地球(%.2f度)的最大摆角增大了【%.1f%%】。\n', max(angle_mars), max(angle_earth), (max(angle_mars)/max(angle_earth)-1)*100);

% --- 2. X方向柔性位移分析 ---
fprintf('\n\n--- 2. X方向柔性位移分析 ---\n');
disp_x_earth = interp1(results.Earth.time, abs(results.Earth.states(:, 11:2:15) * phi_vals_at_midpoint), t_common);
disp_x_moon = interp1(results.Moon.time, abs(results.Moon.states(:, 11:2:15) * phi_vals_at_midpoint), t_common);
disp_x_mars = interp1(results.Mars.time, abs(results.Mars.states(:, 11:2:15) * phi_vals_at_midpoint), t_common);

% 查找峰值时间和值
[max_disp_x_moon, idx_x_moon] = max(abs(disp_x_moon));
t_peak_x_moon = t_common(idx_x_moon);
[max_disp_x_mars, idx_x_mars] = max(abs(disp_x_mars));
t_peak_x_mars = t_common(idx_x_mars);
[max_disp_x_earth, idx_x_earth] = max(abs(disp_x_earth));
t_peak_x_earth = t_common(idx_x_earth);

fprintf('\n[X方向柔性位移峰值时间]:\n');
fprintf(' -> 月球: 峰值 %.4f m 出现在 %.2f 秒\n', max_disp_x_moon, t_peak_x_moon);
fprintf(' -> 火星: 峰值 %.4f m 出现在 %.2f 秒\n', max_disp_x_mars, t_peak_x_mars);
fprintf(' -> 地球: 峰值 %.4f m 出现在 %.2f 秒\n', max_disp_x_earth, t_peak_x_earth);

fprintf('\n[月球 vs 地球]:\n');
fprintf(' -> 从各自峰值来看, 月球(%.4fm)相比地球(%.4fm)的最大位移增大了【%.1f%%】。\n', max_disp_x_moon, max_disp_x_earth, (max_disp_x_moon/max_disp_x_earth-1)*100);
fprintf('\n[火星 vs 地球]:\n');
fprintf(' -> 从各自峰值来看, 火星(%.4fm)相比地球(%.4fm)的最大位移增大了【%.1f%%】。\n', max_disp_x_mars, max_disp_x_earth, (max_disp_x_mars/max_disp_x_earth-1)*100);

% --- 3. Y方向柔性位移分析 ---
fprintf('\n\n--- 3. Y方向柔性位移分析 ---\n');
disp_y_earth = interp1(results.Earth.time, abs(results.Earth.states(:, 17:2:21) * phi_vals_at_midpoint), t_common);
disp_y_moon = interp1(results.Moon.time, abs(results.Moon.states(:, 17:2:21) * phi_vals_at_midpoint), t_common);
disp_y_mars = interp1(results.Mars.time, abs(results.Mars.states(:, 17:2:21) * phi_vals_at_midpoint), t_common);

% 查找峰值时间和值
[max_disp_y_moon, idx_y_moon] = max(abs(disp_y_moon));
t_peak_y_moon = t_common(idx_y_moon);
[max_disp_y_mars, idx_y_mars] = max(abs(disp_y_mars));
t_peak_y_mars = t_common(idx_y_mars);
[max_disp_y_earth, idx_y_earth] = max(abs(disp_y_earth));
t_peak_y_earth = t_common(idx_y_earth);

fprintf('\n[Y方向柔性位移峰值时间]:\n');
fprintf(' -> 月球: 峰值 %.4f m 出现在 %.2f 秒\n', max_disp_y_moon, t_peak_y_moon);
fprintf(' -> 火星: 峰值 %.4f m 出现在 %.2f 秒\n', max_disp_y_mars, t_peak_y_mars);
fprintf(' -> 地球: 峰值 %.4f m 出现在 %.2f 秒\n', max_disp_y_earth, t_peak_y_earth);

fprintf('\n[月球 vs 地球]:\n');
fprintf(' -> 从各自峰值来看, 月球(%.4fm)相比地球(%.4fm)的最大位移增大了【%.1f%%】。\n', max_disp_y_moon, max_disp_y_earth, (max_disp_y_moon/max_disp_y_earth-1)*100);
fprintf('\n[火星 vs 地球]:\n');
fprintf(' -> 从各自峰值来看, 火星(%.4fm)相比地球(%.4fm)的最大位移增大了【%.1f%%】。\n', max_disp_y_mars, max_disp_y_earth, (max_disp_y_mars/max_disp_y_earth-1)*100);

%% 
% 图1: 摆动角度比较 (月球 vs. 火星 vs. 地球)
figure('Name','摆动角度比较 (月球 vs. 火星 vs. 地球)','Position',[100 100 900 600]);
hold on;
newcolor=colororder;
plot(results.Moon.time, results.Moon.states(:,7)*180/pi, '-', 'Color', newcolor(4,:), 'LineWidth', 2);
plot(results.Moon.time, results.Moon.states(:,7)*180/pi, '--', 'Color', newcolor(6,:), 'LineWidth',2);
plot(results.Mars.time, results.Mars.states(:,7)*180/pi, '-','Color', newcolor(3,:), 'LineWidth', 2);
plot(results.Mars.time, results.Mars.states(:,9)*180/pi, ':', 'Color', newcolor(2,:),'LineWidth', 2);
plot(results.Earth.time, results.Earth.states(:,7)*180/pi, '-','Color', newcolor(1,:), 'LineWidth', 2); 
plot(results.Earth.time, results.Earth.states(:,9)*180/pi, '-.', 'Color', newcolor(7,:),'LineWidth', 2); 

% plot(results.Mars.time, results.Mars.states(:,7)*180/pi, '--','Color', [255, 127, 80]/255, 'LineWidth', 1.5);
% plot(results.Mars.time, results.Mars.states(:,9)*180/pi, '--', 'Color', [250, 69, 0]/255,'LineWidth', 1.5);
% plot(results.Earth.time, results.Earth.states(:,7)*180/pi, ':','Color', [100, 149, 237]/255, 'LineWidth', 2); 
% plot(results.Earth.time, results.Earth.states(:,9)*180/pi, ':', 'Color', [0, 0, 255]/255,'LineWidth', 2); 
hold off;
title('Swing angle under different gravitational forces',Interpreter='latex');
xlabel('Time (s)');
ylabel('Angle (°)');
legend('$\theta_x (Moon:0.167g)$', '$\theta_y (Moon:0.167g)$', '$\theta_x (Mars:0.38g)$', '$\theta_y (Mars:0.38g)$', '$\theta_x (Earth:g)$', '$\theta_y (Earth:g)$', 'Location', 'best','Orientation', 'vertical','Interpreter','latex');
grid on;

% 图2: 总柔性位移比较 (月球 vs. 火星 vs. 地球)
figure('Name', '柔性位移响应比较 (中点)', 'Position', [150 150 1200 800]);
line_styles = {'-', '--', ':'};
% X方向总位移
ax_x = subplot(2,1,1);
hold(ax_x, 'on');
for i = 1:length(scenario_names)
    total_disp_x = results.(scenario_names{i}).states(:, 11:2:15) * phi_vals_at_midpoint;
    plot(ax_x, results.(scenario_names{i}).time, total_disp_x, line_styles{i}, 'LineWidth', 2, 'DisplayName', scenario_names{i});
end
hold(ax_x, 'off');
% title('Flexible displacement in the $X$ direction (relative to the midpoint of the suspension rope)',Interpreter='latex');
%xlabel('Time (s)');
xlabel('(a)');
ylabel('Displacement (m)');
legend(ax_x, 'Location', 'best');
grid on;

% Y方向总位移
ax_y = subplot(2,1,2);
hold(ax_y, 'on');
for i = 1:length(scenario_names)
    total_disp_y = results.(scenario_names{i}).states(:, 17:2:21) * phi_vals_at_midpoint;
    plot(ax_y, results.(scenario_names{i}).time, total_disp_y, line_styles{i}, 'LineWidth', 2, 'DisplayName', scenario_names{i});
end
hold(ax_y, 'off');
% title('Flexible displacement in the $Y$ direction (relative to the midpoint of the suspension rope)',Interpreter='latex');
% xlabel('Time (s)');
xlabel('(b)');
ylabel('Displacement (m)');
legend(ax_y, 'Location', 'best');
grid on;

fprintf('\n====== 正在生成力曲线图表... ======\n');
figure('Name', 'Force Profile', 'Position', [200 200 800 400]);
hold on;
for i = 1:length(scenario_names)
    plot(results.(scenario_names{i}).time, results.(scenario_names{i}).states(:, 4), 'b-', 'LineWidth', 1.5, 'DisplayName', [scenario_names{i}, ' f_{mx}']); % 假设第4列是f_mx
    plot(results.(scenario_names{i}).time, results.(scenario_names{i}).states(:, 5), 'r-', 'LineWidth', 1.5, 'DisplayName', [scenario_names{i}, ' f_{my}']);
    plot(results.(scenario_names{i}).time, results.(scenario_names{i}).states(:, 6), 'g-', 'LineWidth', 1.5, 'DisplayName', [scenario_names{i}, ' f_{mz}']);
end
hold off;
title('施加于负载的力曲线');
xlabel('时间 (s)');
ylabel('力 (N)');
legend('Location', 'best');
grid on;

fprintf('\n====== 所有任务完成! ======\n');

%% Functions
function u = design_velocity_profile(t)
    % 目标: 控制底部负载机器人沿 x 和 y 方向同时执行单摆的往复运动
    % 参数: 负载质量 m = 10 kg (from parameters.m), 时间范围 0-32 秒

    % 1. 设置顶层小车的期望速度为0，使其保持静止
    v_des_x = 0;
    v_des_y = 0;
    v_des_l = 0;

    force_magnitude = 1.0; % 基础力幅度 (N), 控制摆动幅度
    f_mx = 0;
    f_my = 0;
    f_mz = 0;

    if t >= 0 && t < 32
        % 使用正弦函数驱动 x 和 y 方向的单摆运动，周期 16 秒
        t_shift = t; % 从 t=0 开始
        omega = 2 * pi / 16; % 周期 16 秒
        f_mx = force_magnitude * sin(omega * t_shift); % x 方向力
        f_my = force_magnitude * sin(omega * t_shift); % y 方向力，同步变化
    end

    % 最终的控制输入向量
    u = [v_des_x; v_des_y; v_des_l; f_mx; f_my; f_mz];
end

function dydt = nonlinear_dynamics_with_limits(t, y, p, control_profile)
    control_inputs = control_profile(t);
    x = y(1); y_pos = y(3); l = y(5);
    if (x <= p.x_min && control_inputs(1) < 0) || (x >= p.x_max && control_inputs(1) > 0), control_inputs(1) = 0; end
    if (y_pos <= p.y_min && control_inputs(2) < 0) || (y_pos >= p.y_max && control_inputs(2) > 0), control_inputs(2) = 0; end
    if (l <= p.l_min && control_inputs(3) < 0) || (l >= p.l_max && control_inputs(3) > 0), control_inputs(3) = 0; end
    dydt = nonlinear_dynamics(t, y, p, control_inputs);
end