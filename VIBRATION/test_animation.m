clear; clc; close all;

fprintf('测试真实桥式吊车3D可视化...\n');

%% 加载参数
p = parameters();

%% 生成测试数据
t_end = 10;
dt = 0.1;
t = 0:dt:t_end;
n_steps = length(t);

% 生成简单的测试轨迹
x_cart = 0.5 + 0.3*sin(0.5*t);           % X方向正弦运动
y_cart = 1.0 + 0.4*cos(0.3*t);           % Y方向余弦运动
l_rope = p.l0 + 0.2*sin(0.8*t);          % 绳长变化
theta_x = 0.1*sin(0.7*t);                % X方向摆角
theta_y = 0.08*cos(0.6*t);               % Y方向摆角

fprintf('生成测试数据完成，共 %d 个时间点\n', n_steps);
fprintf('时间范围: %.1f - %.1f 秒\n', t(1), t(end));
fprintf('小车运动范围: X=[%.2f, %.2f], Y=[%.2f, %.2f]\n', ...
        min(x_cart), max(x_cart), min(y_cart), max(y_cart));
fprintf('绳长变化范围: [%.2f, %.2f] 米\n', min(l_rope), max(l_rope));
fprintf('最大摆角: θ_x=%.1f°, θ_y=%.1f°\n', ...
        max(abs(theta_x))*180/pi, max(abs(theta_y))*180/pi);

%% 启动3D动画
fprintf('\n启动3D动画...\n');
animatecart(t, x_cart, y_cart, l_rope, theta_x, theta_y, p);

fprintf('3D动画测试完成！\n'); 