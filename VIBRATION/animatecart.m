function animatecart(t, x_cart_sim, y_cart_sim, l_rope_sim, theta_x_sim, theta_y_sim, p_params)

    fprintf('启动真实桥式吊车3D动画...\n');

    % --- 0. 计算负载的绝对位置 ---
    x_payload_sim = x_cart_sim + l_rope_sim .* sin(theta_x_sim);
    y_payload_sim = y_cart_sim + l_rope_sim .* sin(theta_y_sim) .* cos(theta_x_sim);
    z_payload_sim = -l_rope_sim .* cos(theta_x_sim) .* cos(theta_y_sim); % 相对于悬挂点

    % --- 1. 真实平台参数设置 ---
    % 平台实际尺寸
    platform_x_span = 2.0;    % X轴跨度 2米
    platform_y_span = 2.5;    % Y轴跨度 2.5米
    platform_height = 2.5;    % Z轴高度 2.5米
    
    % 结构参数
    beam_width = 0.05;         % 梁的宽度
    support_height = 0.3;      % 支撑柱高度
    ground_level = 0;          % 地面高度
    crane_height = platform_height; % 吊车高度
    
    % 小车和负载参数
    cart_size = 0.08;          % 小车尺寸
    payload_size = 0.06;       % 负载尺寸
    
    % 四轮小车参数
    vehicle_length = 0.24;     % 小车长度
    vehicle_width = 0.26;      % 小车宽度
    vehicle_height = 0.1;     % 小车高度
    wheel_radius = 0.03;      % 车轮半径
    wheel_width = 0.02;        % 车轮宽度
    
    % --- 2. 初始化3D图窗 ---
    figure_handle = figure('Position', [100, 100, 1200, 900]);
    clf(figure_handle);
    set(figure_handle, 'Name', '真实桥式吊车低重力平台仿真', 'NumberTitle', 'off', 'Renderer', 'opengl');
    hold on;
    axis equal;
    grid on;
    view(45, 30); % 更好的观察角度
    
    % --- 3. 绘制地面 ---
    ground_x = [-0.5, platform_x_span+0.5];
    ground_y = [-0.5, platform_y_span+0.5];
    [X_ground, Y_ground] = meshgrid(ground_x, ground_y);
    Z_ground = ground_level * ones(size(X_ground));
    surf(X_ground, Y_ground, Z_ground, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    % --- 4. 绘制桥式吊车主体结构 ---
    
    % 4.1 主梁（X方向，跨度2米）
    main_beam_x = [0, platform_x_span, platform_x_span, 0, 0];
    main_beam_y1 = [0, 0, beam_width, beam_width, 0]; % Y=0处的主梁
    main_beam_y2 = [platform_y_span-beam_width, platform_y_span-beam_width, platform_y_span, platform_y_span, platform_y_span-beam_width]; % Y=2.5处的主梁
    main_beam_z = [crane_height, crane_height, crane_height, crane_height, crane_height];
    
    % 绘制两根主梁
    plot3(main_beam_x, main_beam_y1, main_beam_z, 'k-', 'LineWidth', 4);
    plot3(main_beam_x, main_beam_y2, main_beam_z, 'k-', 'LineWidth', 4);
    
    % 4.2 端梁（Y方向，跨度2.5米）
    end_beam_y = [0, platform_y_span, platform_y_span, 0, 0];
    end_beam_x1 = [0, 0, beam_width, beam_width, 0]; % X=0处的端梁
    end_beam_x2 = [platform_x_span-beam_width, platform_x_span-beam_width, platform_x_span, platform_x_span, platform_x_span-beam_width]; % X=2处的端梁
    end_beam_z = [crane_height, crane_height, crane_height, crane_height, crane_height];
    
    % 绘制两根端梁
    plot3(end_beam_x1, end_beam_y, end_beam_z, 'k-', 'LineWidth', 4);
    plot3(end_beam_x2, end_beam_y, end_beam_z, 'k-', 'LineWidth', 4);
    
    % 4.3 支撑立柱
    support_positions = [0, 0; platform_x_span, 0; 0, platform_y_span; platform_x_span, platform_y_span];
    for i = 1:size(support_positions, 1)
        x_pos = support_positions(i, 1);
        y_pos = support_positions(i, 2);
        plot3([x_pos, x_pos], [y_pos, y_pos], [ground_level, crane_height], 'k-', 'LineWidth', 6);
        
        % 支撑底座
        base_size = 0.1;
        base_x = [x_pos-base_size, x_pos+base_size, x_pos+base_size, x_pos-base_size, x_pos-base_size];
        base_y = [y_pos-base_size, y_pos-base_size, y_pos+base_size, y_pos+base_size, y_pos-base_size];
        base_z = [ground_level, ground_level, ground_level, ground_level, ground_level];
        plot3(base_x, base_y, base_z, 'k-', 'LineWidth', 2);
    end
    
    % 4.4 轨道（小车运行轨道）
    rail_y1 = beam_width/2;
    rail_y2 = platform_y_span - beam_width/2;
    plot3([0, platform_x_span], [rail_y1, rail_y1], [crane_height+0.01, crane_height+0.01], 'r-', 'LineWidth', 3);
    plot3([0, platform_x_span], [rail_y2, rail_y2], [crane_height+0.01, crane_height+0.01], 'r-', 'LineWidth', 3);
    
    % --- 5. 初始化动态组件 ---
    
    % 5.1 大车（桥架，沿Y方向移动）
    bridge_y = y_cart_sim(1);
    h_bridge = plot3([0, platform_x_span], [bridge_y, bridge_y], [crane_height+0.02, crane_height+0.02], 'm-', 'LineWidth', 5);
    
    % 5.2 小车（沿X方向移动）
    cart_x = x_cart_sim(1);
    cart_y = y_cart_sim(1);
    cart_z = crane_height + 0.03;
    
    % 绘制小车本体（立方体）
    cart_vertices = [
        cart_x-cart_size, cart_y-cart_size, cart_z-cart_size/2;
        cart_x+cart_size, cart_y-cart_size, cart_z-cart_size/2;
        cart_x+cart_size, cart_y+cart_size, cart_z-cart_size/2;
        cart_x-cart_size, cart_y+cart_size, cart_z-cart_size/2;
        cart_x-cart_size, cart_y-cart_size, cart_z+cart_size/2;
        cart_x+cart_size, cart_y-cart_size, cart_z+cart_size/2;
        cart_x+cart_size, cart_y+cart_size, cart_z+cart_size/2;
        cart_x-cart_size, cart_y+cart_size, cart_z+cart_size/2;
    ];
    
    cart_faces = [
        1,2,3,4; 5,6,7,8; 1,2,6,5; 2,3,7,6; 3,4,8,7; 4,1,5,8
    ];
    
    h_cart = patch('Vertices', cart_vertices, 'Faces', cart_faces, ...
                   'FaceColor', 'red', 'FaceAlpha', 0.8, 'EdgeColor', 'black');
    
    % 5.3 绳索
    rope_start_x = x_cart_sim(1);
    rope_start_y = y_cart_sim(1);
    rope_start_z = cart_z - cart_size/2;
    rope_end_x = x_payload_sim(1);
    rope_end_y = y_payload_sim(1);
    rope_end_z = rope_start_z + z_payload_sim(1);
    
    h_rope = plot3([rope_start_x, rope_end_x], [rope_start_y, rope_end_y], ...
                   [rope_start_z, rope_end_z], 'g-', 'LineWidth', 2);
    
    % 5.4 负载（四轮小车）
    vehicle_x = x_payload_sim(1);
    vehicle_y = y_payload_sim(1);
    vehicle_z = rope_end_z;
    
    % 创建四轮小车的图形对象
    [h_vehicle_body, h_wheels] = create_four_wheel_vehicle(vehicle_x, vehicle_y, vehicle_z, ...
                                                          vehicle_length, vehicle_width, vehicle_height, ...
                                                          wheel_radius, wheel_width);
    
    % --- 6. 设置坐标轴和标签 ---
    padding = 0.3;
    xlim([-padding, platform_x_span + padding]);
    ylim([-padding, platform_y_span + padding]);
    zlim([ground_level - padding, crane_height + padding]);
    
    xlabel('X轴位置 (m)', 'FontSize', 12);
    ylabel('Y轴位置 (m)', 'FontSize', 12);
    zlabel('Z轴高度 (m)', 'FontSize', 12);
    
    % 添加详细的标题和信息
    title_handle = title(sprintf('桥式吊车低重力平台仿真\\n时间: %.2f s | 小车位置: (%.2f, %.2f) m | 绳长: %.2f m', ...
                                t(1), x_cart_sim(1), y_cart_sim(1), l_rope_sim(1)), 'FontSize', 14);
    
    % 添加图例
    legend_elements = [
        plot3(NaN, NaN, NaN, 'k-', 'LineWidth', 4), ...
        plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 3), ...
        plot3(NaN, NaN, NaN, 'm-', 'LineWidth', 5), ...
        h_cart, ...
        plot3(NaN, NaN, NaN, 'g-', 'LineWidth', 2), ...
        h_vehicle_body
    ];
    legend(legend_elements, {'钢结构', '轨道', '大车', '小车', '绳索', '四轮车'}, ...
           'Location', 'northeast', 'FontSize', 10);
    
    % --- 7. 动画循环 ---
    animation_speed_factor = 1;
    skip_frames = max(1, round(length(t)/300)); % 限制帧数以提高性能
    
    fprintf('开始动画播放，共 %d 帧...\n', ceil(length(t)/skip_frames));
    
    for i = 1:skip_frames:length(t)
        if ~ishandle(figure_handle)
            fprintf('动画窗口被用户关闭\n');
            break;
        end
        
        % 更新大车位置
        set(h_bridge, 'YData', [y_cart_sim(i), y_cart_sim(i)]);
        
        % 更新小车位置
        cart_x = x_cart_sim(i);
        cart_y = y_cart_sim(i);
        new_cart_vertices = [
            cart_x-cart_size, cart_y-cart_size, cart_z-cart_size/2;
            cart_x+cart_size, cart_y-cart_size, cart_z-cart_size/2;
            cart_x+cart_size, cart_y+cart_size, cart_z-cart_size/2;
            cart_x-cart_size, cart_y+cart_size, cart_z-cart_size/2;
            cart_x-cart_size, cart_y-cart_size, cart_z+cart_size/2;
            cart_x+cart_size, cart_y-cart_size, cart_z+cart_size/2;
            cart_x+cart_size, cart_y+cart_size, cart_z+cart_size/2;
            cart_x-cart_size, cart_y+cart_size, cart_z+cart_size/2;
        ];
        set(h_cart, 'Vertices', new_cart_vertices);
        
        % 更新绳索
        rope_start_x = x_cart_sim(i);
        rope_start_y = y_cart_sim(i);
        rope_end_x = x_payload_sim(i);
        rope_end_y = y_payload_sim(i);
        rope_end_z = cart_z - cart_size/2 + z_payload_sim(i);
        
        set(h_rope, 'XData', [rope_start_x, rope_end_x], ...
                    'YData', [rope_start_y, rope_end_y], ...
                    'ZData', [cart_z - cart_size/2, rope_end_z]);
        
        % 更新四轮小车位置
        vehicle_x = x_payload_sim(i);
        vehicle_y = y_payload_sim(i);
        vehicle_z = rope_end_z;
        
        update_four_wheel_vehicle(h_vehicle_body, h_wheels, vehicle_x, vehicle_y, vehicle_z, ...
                                 vehicle_length, vehicle_width, vehicle_height, ...
                                 wheel_radius, wheel_width);
        
        % 更新标题
        set(title_handle, 'String', sprintf('桥式吊车低重力平台仿真\\n时间: %.2f s | 小车位置: (%.2f, %.2f) m | 绳长: %.2f m | 摆角: (%.1f°, %.1f°)', ...
                                           t(i), x_cart_sim(i), y_cart_sim(i), l_rope_sim(i), ...
                                           theta_x_sim(i)*180/pi, theta_y_sim(i)*180/pi));
        
        drawnow;
        
        % 控制动画速度
        if i < length(t)
            actual_pause_time = (t(min(i+skip_frames, length(t))) - t(i)) / animation_speed_factor;
            pause(max(0.01, actual_pause_time)); % 最小暂停时间
        end
        
        % 显示进度
        if mod(i, round(length(t)/10)) == 1
            fprintf('动画进度: %.0f%%\n', i/length(t)*100);
        end
    end
    
    fprintf('3D动画播放完成！\n');
    hold off;
end

function [h_body, h_wheels] = create_four_wheel_vehicle(x, y, z, length, width, height, wheel_radius, wheel_width)
    % 创建四轮小车的3D模型
    
    % 车身（长方体）
    body_vertices = [
        x-length/2, y-width/2, z-height/2;          % 1: 底面左前
        x+length/2, y-width/2, z-height/2;          % 2: 底面右前
        x+length/2, y+width/2, z-height/2;          % 3: 底面右后
        x-length/2, y+width/2, z-height/2;          % 4: 底面左后
        x-length/2, y-width/2, z+height/2;          % 5: 顶面左前
        x+length/2, y-width/2, z+height/2;          % 6: 顶面右前
        x+length/2, y+width/2, z+height/2;          % 7: 顶面右后
        x-length/2, y+width/2, z+height/2;          % 8: 顶面左后
    ];
    
    body_faces = [
        1,2,3,4;    % 底面
        5,6,7,8;    % 顶面
        1,2,6,5;    % 前面
        2,3,7,6;    % 右面
        3,4,8,7;    % 后面
        4,1,5,8;    % 左面
    ];
    
    h_body = patch('Vertices', body_vertices, 'Faces', body_faces, ...
                   'FaceColor', 'blue', 'FaceAlpha', 0.8, 'EdgeColor', 'black', 'LineWidth', 1);
    
    % 四个车轮位置
    wheel_positions = [
        x-length/2+0.02, y-width/2-wheel_width/2, z-height/2;    % 左前轮
        x+length/2-0.02, y-width/2-wheel_width/2, z-height/2;    % 右前轮
        x-length/2+0.02, y+width/2+wheel_width/2, z-height/2;    % 左后轮
        x+length/2-0.02, y+width/2+wheel_width/2, z-height/2;    % 右后轮
    ];
    
    % 创建四个车轮
    h_wheels = [];
    for i = 1:4
        wheel_x = wheel_positions(i, 1);
        wheel_y = wheel_positions(i, 2);
        wheel_z = wheel_positions(i, 3);
        
        % 创建圆柱体车轮
        [cyl_x, cyl_y, cyl_z] = cylinder(wheel_radius, 16);
        cyl_z = cyl_z * wheel_width - wheel_width/2;
        
        % 旋转车轮（使其沿Y轴方向）
        cyl_temp = cyl_y;
        cyl_y = cyl_z + wheel_y;
        cyl_z = cyl_temp + wheel_z;
        
        cyl_x = cyl_x + wheel_x;
        
        h_wheel = surf(cyl_x, cyl_y, cyl_z, 'FaceColor', 'black', ...
                      'FaceAlpha', 0.9, 'EdgeColor', 'none');
        h_wheels = [h_wheels, h_wheel];
    end
end

function update_four_wheel_vehicle(h_body, h_wheels, x, y, z, length, width, height, wheel_radius, wheel_width)
    % 更新四轮小车的位置
    
    % 更新车身
    body_vertices = [
        x-length/2, y-width/2, z-height/2;          % 1: 底面左前
        x+length/2, y-width/2, z-height/2;          % 2: 底面右前
        x+length/2, y+width/2, z-height/2;          % 3: 底面右后
        x-length/2, y+width/2, z-height/2;          % 4: 底面左后
        x-length/2, y-width/2, z+height/2;          % 5: 顶面左前
        x+length/2, y-width/2, z+height/2;          % 6: 顶面右前
        x+length/2, y+width/2, z+height/2;          % 7: 顶面右后
        x-length/2, y+width/2, z+height/2;          % 8: 顶面左后
    ];
    
    set(h_body, 'Vertices', body_vertices);
    
    % 四个车轮位置
    wheel_positions = [
        x-length/2+0.02, y-width/2-wheel_width/2, z-height/2;    % 左前轮
        x+length/2-0.02, y-width/2-wheel_width/2, z-height/2;    % 右前轮
        x-length/2+0.02, y+width/2+wheel_width/2, z-height/2;    % 左后轮
        x+length/2-0.02, y+width/2+wheel_width/2, z-height/2;    % 右后轮
    ];
    
    % 更新四个车轮
    for i = 1:4
        wheel_x = wheel_positions(i, 1);
        wheel_y = wheel_positions(i, 2);
        wheel_z = wheel_positions(i, 3);
        
        % 创建圆柱体车轮
        [cyl_x, cyl_y, cyl_z] = cylinder(wheel_radius, 16);
        cyl_z = cyl_z * wheel_width - wheel_width/2;
        
        % 旋转车轮（使其沿Y轴方向）
        cyl_temp = cyl_y;
        cyl_y = cyl_z + wheel_y;
        cyl_z = cyl_temp + wheel_z;
        
        cyl_x = cyl_x + wheel_x;
        
        set(h_wheels(i), 'XData', cyl_x, 'YData', cyl_y, 'ZData', cyl_z);
    end
end