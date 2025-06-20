function animatecart(t, x_cart_sim, y_cart_sim, l_rope_sim, theta_x_sim, theta_y_sim, p_params)

    fprintf('启动真实桥式吊车3D动画（机械狗版本）...\n');

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
    
    % 机械狗参数
    dog_body_length = 0.20;    % 机械狗身体长度
    dog_body_width = 0.12;     % 机械狗身体宽度
    dog_body_height = 0.08;    % 机械狗身体高度
    dog_leg_length = 0.08;     % 机械狗腿长
    dog_leg_radius = 0.015;    % 机械狗腿粗细
    
    % --- 2. 初始化3D图窗 ---
    figure_handle = figure('Position', [100, 100, 1200, 900]);
    clf(figure_handle);
    set(figure_handle, 'Name', '真实桥式吊车低重力平台仿真（机械狗版本）', 'NumberTitle', 'off', 'Renderer', 'opengl');
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
    
    % 5.4 负载（机械狗）
    dog_x = x_payload_sim(1);
    dog_y = y_payload_sim(1);
    dog_z = rope_end_z;
    
    % 创建机械狗的图形对象
    [h_dog_body, h_dog_legs, h_dog_head] = create_robot_dog(dog_x, dog_y, dog_z, ...
                                                          dog_body_length, dog_body_width, dog_body_height, ...
                                                          dog_leg_length, dog_leg_radius);
    
    % 初始化力矢量箭头（用于显示f_mx, f_my, f_mz）
    h_force_arrows = [];
    force_colors = {'red', 'green', 'blue'};
    for k = 1:3
        h_force_arrows(k) = quiver3(dog_x, dog_y, dog_z, 0, 0, 0, 'Color', force_colors{k}, ...
                                   'LineWidth', 3, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
    end
    
    % --- 6. 设置坐标轴和标签 ---
    padding = 0.3;
    xlim([-padding, platform_x_span + padding]);
    ylim([-padding, platform_y_span + padding]);
    zlim([ground_level - padding, crane_height + padding]);
    
    xlabel('X轴位置 (m)', 'FontSize', 12);
    ylabel('Y轴位置 (m)', 'FontSize', 12);
    zlabel('Z轴高度 (m)', 'FontSize', 12);
    
    % 添加详细的标题和信息
    title_handle = title(sprintf('桥式吊车低重力平台仿真（机械狗版本）\\n时间: %.2f s | 小车位置: (%.2f, %.2f) m | 绳长: %.2f m', ...
                                t(1), x_cart_sim(1), y_cart_sim(1), l_rope_sim(1)), 'FontSize', 14);
    
    % 添加图例
    legend_elements = [
        plot3(NaN, NaN, NaN, 'k-', 'LineWidth', 4), ...
        plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 3), ...
        plot3(NaN, NaN, NaN, 'm-', 'LineWidth', 5), ...
        h_cart, ...
        plot3(NaN, NaN, NaN, 'g-', 'LineWidth', 2), ...
        h_dog_body
    ];
    legend(legend_elements, {'钢结构', '轨道', '大车', '小车', '绳索', '机械狗'}, ...
           'Location', 'northeast', 'FontSize', 10);
    
    % --- 7. 动画循环 ---
    animation_speed_factor = 1;
    skip_frames = max(1, round(length(t)/300)); % 限制帧数以提高性能
    
    fprintf('开始动画播放，共 %d 帧...\n', ceil(length(t)/skip_frames));
    
    % 全局变量访问控制输出
    global controller_outputs_log;
    
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
        
        % 更新机械狗位置
        dog_x = x_payload_sim(i);
        dog_y = y_payload_sim(i);
        dog_z = rope_end_z;
        
        % 获取当前时刻的力控制输入
        current_forces = get_force_at_time(t(i), controller_outputs_log);
        f_mx = current_forces(1); f_my = current_forces(2); f_mz = current_forces(3);
        
        update_robot_dog(h_dog_body, h_dog_legs, h_dog_head, dog_x, dog_y, dog_z, ...
                        dog_body_length, dog_body_width, dog_body_height, ...
                        dog_leg_length, dog_leg_radius, f_mx, f_my, t(i));
        
        % 力矢量显示（f_mx, f_my, f_mz已在上面获取）
        
        % 更新力矢量箭头
        force_scale = 0.01; % 调整箭头长度的比例因子
        
        if length(h_force_arrows) >= 3
            % X方向力 (红色)
            set(h_force_arrows(1), 'XData', dog_x, 'YData', dog_y, 'ZData', dog_z + dog_body_height/2, ...
                                  'UData', f_mx * force_scale, 'VData', 0, 'WData', 0);
            % Y方向力 (绿色)
            set(h_force_arrows(2), 'XData', dog_x, 'YData', dog_y, 'ZData', dog_z + dog_body_height/2, ...
                                  'UData', 0, 'VData', f_my * force_scale, 'WData', 0);
            % Z方向力 (蓝色)
            set(h_force_arrows(3), 'XData', dog_x, 'YData', dog_y, 'ZData', dog_z + dog_body_height/2, ...
                                  'UData', 0, 'VData', 0, 'WData', f_mz * force_scale);
        end
        
        % 更新标题
        set(title_handle, 'String', sprintf('桥式吊车低重力平台仿真（机械狗版本）\\n时间: %.2f s | 小车位置: (%.2f, %.2f) m | 绳长: %.2f m | 摆角: (%.1f°, %.1f°)\\n力: F_x=%.1fN, F_y=%.1fN, F_z=%.1fN', ...
                                           t(i), x_cart_sim(i), y_cart_sim(i), l_rope_sim(i), ...
                                           theta_x_sim(i)*180/pi, theta_y_sim(i)*180/pi, ...
                                           f_mx, f_my, f_mz));
        
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

function [h_body, h_legs, h_head] = create_robot_dog(x, y, z, body_length, body_width, body_height, leg_length, leg_radius)
    % 创建机械狗的3D模型
    
    % 1. 机械狗身体（椭圆体）
    body_vertices = [
        x-body_length/2, y-body_width/2, z-body_height/2;          % 1: 底面左前
        x+body_length/2, y-body_width/2, z-body_height/2;          % 2: 底面右前
        x+body_length/2, y+body_width/2, z-body_height/2;          % 3: 底面右后
        x-body_length/2, y+body_width/2, z-body_height/2;          % 4: 底面左后
        x-body_length/2, y-body_width/2, z+body_height/2;          % 5: 顶面左前
        x+body_length/2, y-body_width/2, z+body_height/2;          % 6: 顶面右前
        x+body_length/2, y+body_width/2, z+body_height/2;          % 7: 顶面右后
        x-body_length/2, y+body_width/2, z+body_height/2;          % 8: 顶面左后
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
                   'FaceColor', [0.7 0.5 0.3], 'FaceAlpha', 0.9, 'EdgeColor', 'black', 'LineWidth', 1);
    
    % 2. 机械狗的头部
    head_size = body_height * 0.6;
    head_x = x + body_length/2 + head_size/2;
    head_y = y;
    head_z = z + body_height/4;
    
    head_vertices = [
        head_x-head_size/2, head_y-head_size/2, head_z-head_size/2;
        head_x+head_size/2, head_y-head_size/2, head_z-head_size/2;
        head_x+head_size/2, head_y+head_size/2, head_z-head_size/2;
        head_x-head_size/2, head_y+head_size/2, head_z-head_size/2;
        head_x-head_size/2, head_y-head_size/2, head_z+head_size/2;
        head_x+head_size/2, head_y-head_size/2, head_z+head_size/2;
        head_x+head_size/2, head_y+head_size/2, head_z+head_size/2;
        head_x-head_size/2, head_y+head_size/2, head_z+head_size/2;
    ];
    
    head_faces = [
        1,2,3,4; 5,6,7,8; 1,2,6,5; 2,3,7,6; 3,4,8,7; 4,1,5,8
    ];
    
    h_head = patch('Vertices', head_vertices, 'Faces', head_faces, ...
                   'FaceColor', [0.6 0.4 0.2], 'FaceAlpha', 0.9, 'EdgeColor', 'black', 'LineWidth', 1);
    
    % 3. 四条腿位置
    leg_positions = [
        x-body_length/3, y-body_width/2, z-body_height/2;    % 左前腿
        x+body_length/3, y-body_width/2, z-body_height/2;    % 右前腿
        x-body_length/3, y+body_width/2, z-body_height/2;    % 左后腿
        x+body_length/3, y+body_width/2, z-body_height/2;    % 右后腿
    ];
    
    % 创建四条腿
    h_legs = [];
    for i = 1:4
        leg_x = leg_positions(i, 1);
        leg_y = leg_positions(i, 2);
        leg_z_top = leg_positions(i, 3);
        leg_z_bottom = leg_z_top - leg_length;
        
        % 创建圆柱体腿
        [cyl_x, cyl_y, cyl_z] = cylinder(leg_radius, 8);
        cyl_z = cyl_z * leg_length + leg_z_bottom;
        cyl_x = cyl_x + leg_x;
        cyl_y = cyl_y + leg_y;
        
        h_leg = surf(cyl_x, cyl_y, cyl_z, 'FaceColor', [0.4 0.4 0.4], ...
                    'FaceAlpha', 0.9, 'EdgeColor', 'none');
        h_legs = [h_legs, h_leg];
        
        % 添加脚掌
        foot_size = leg_radius * 1.5;
        [sphere_x, sphere_y, sphere_z] = sphere(8);
        sphere_x = sphere_x * foot_size + leg_x;
        sphere_y = sphere_y * foot_size + leg_y;
        sphere_z = sphere_z * foot_size + leg_z_bottom;
        
        h_foot = surf(sphere_x, sphere_y, sphere_z, 'FaceColor', [0.2 0.2 0.2], ...
                     'FaceAlpha', 0.9, 'EdgeColor', 'none');
        h_legs = [h_legs, h_foot];
    end
end

function update_robot_dog(h_body, h_legs, h_head, x, y, z, body_length, body_width, body_height, leg_length, leg_radius, f_mx, f_my, current_time)
    % 更新机械狗的位置和朝向
    
    % 计算机械狗应该面向的角度（基于施加的力）
    force_magnitude = sqrt(f_mx^2 + f_my^2);
    if force_magnitude > 0.1  % 只有当力足够大时才调整朝向
        % 计算力的方向角度
        force_angle = atan2(f_my, f_mx);
    else
        force_angle = 0; % 默认朝向X正方向
    end
    
    % 计算旋转后的身体顶点
    cos_angle = cos(force_angle);
    sin_angle = sin(force_angle);
    
    % 定义身体在局部坐标系中的顶点（相对于中心）
    local_body_vertices = [
        -body_length/2, -body_width/2, -body_height/2;          % 1: 底面左前
        +body_length/2, -body_width/2, -body_height/2;          % 2: 底面右前
        +body_length/2, +body_width/2, -body_height/2;          % 3: 底面右后
        -body_length/2, +body_width/2, -body_height/2;          % 4: 底面左后
        -body_length/2, -body_width/2, +body_height/2;          % 5: 顶面左前
        +body_length/2, -body_width/2, +body_height/2;          % 6: 顶面右前
        +body_length/2, +body_width/2, +body_height/2;          % 7: 顶面右后
        -body_length/2, +body_width/2, +body_height/2;          % 8: 顶面左后
    ];
    
    % 应用旋转变换
    body_vertices = zeros(size(local_body_vertices));
    for i = 1:8
        local_x = local_body_vertices(i, 1);
        local_y = local_body_vertices(i, 2);
        
        % 绕Z轴旋转
        rotated_x = local_x * cos_angle - local_y * sin_angle;
        rotated_y = local_x * sin_angle + local_y * cos_angle;
        
        body_vertices(i, :) = [x + rotated_x, y + rotated_y, z + local_body_vertices(i, 3)];
    end
    
    set(h_body, 'Vertices', body_vertices);
    
    % 更新头部（头部朝向力的方向）
    head_size = body_height * 0.6;
    head_offset = body_length/2 + head_size/2;
    head_x = x + head_offset * cos_angle;
    head_y = y + head_offset * sin_angle;
    head_z = z + body_height/4;
    
    % 定义头部在局部坐标系中的顶点
    local_head_vertices = [
        -head_size/2, -head_size/2, -head_size/2;
        +head_size/2, -head_size/2, -head_size/2;
        +head_size/2, +head_size/2, -head_size/2;
        -head_size/2, +head_size/2, -head_size/2;
        -head_size/2, -head_size/2, +head_size/2;
        +head_size/2, -head_size/2, +head_size/2;
        +head_size/2, +head_size/2, +head_size/2;
        -head_size/2, +head_size/2, +head_size/2;
    ];
    
    % 应用旋转变换到头部
    head_vertices = zeros(size(local_head_vertices));
    for i = 1:8
        local_x = local_head_vertices(i, 1);
        local_y = local_head_vertices(i, 2);
        
        rotated_x = local_x * cos_angle - local_y * sin_angle;
        rotated_y = local_x * sin_angle + local_y * cos_angle;
        
        head_vertices(i, :) = [head_x + rotated_x, head_y + rotated_y, head_z + local_head_vertices(i, 3)];
    end
    
    set(h_head, 'Vertices', head_vertices);
    
    % 四条腿位置（考虑旋转）
    local_leg_positions = [
        -body_length/3, -body_width/2, -body_height/2;    % 左前腿
        +body_length/3, -body_width/2, -body_height/2;    % 右前腿
        -body_length/3, +body_width/2, -body_height/2;    % 左后腿
        +body_length/3, +body_width/2, -body_height/2;    % 右后腿
    ];
    
    % 添加行走动画效果
    walk_cycle = mod(current_time * 3, 2*pi); % 行走周期
    leg_lift = 0.02 * abs(force_magnitude) * sin(walk_cycle + [0, pi, pi, 0]); % 腿部上下运动
    
    % 更新四条腿
    for i = 1:4
        % 计算旋转后的腿部位置
        local_x = local_leg_positions(i, 1);
        local_y = local_leg_positions(i, 2);
        
        rotated_x = local_x * cos_angle - local_y * sin_angle;
        rotated_y = local_x * sin_angle + local_y * cos_angle;
        
        leg_x = x + rotated_x;
        leg_y = y + rotated_y;
        leg_z_top = z + local_leg_positions(i, 3);
        leg_z_bottom = leg_z_top - leg_length + leg_lift(i); % 添加行走动画
        
        % 更新腿部圆柱体
        leg_idx = (i-1)*2 + 1;
        if leg_idx <= length(h_legs)
            [cyl_x, cyl_y, cyl_z] = cylinder(leg_radius, 8);
            cyl_z = cyl_z * (leg_length - leg_lift(i)) + leg_z_bottom;
            cyl_x = cyl_x + leg_x;
            cyl_y = cyl_y + leg_y;
            
            set(h_legs(leg_idx), 'XData', cyl_x, 'YData', cyl_y, 'ZData', cyl_z);
            
            % 更新脚掌
            foot_idx = leg_idx + 1;
            if foot_idx <= length(h_legs)
                foot_size = leg_radius * 1.5;
                [sphere_x, sphere_y, sphere_z] = sphere(8);
                sphere_x = sphere_x * foot_size + leg_x;
                sphere_y = sphere_y * foot_size + leg_y;
                sphere_z = sphere_z * foot_size + leg_z_bottom;
                
                set(h_legs(foot_idx), 'XData', sphere_x, 'YData', sphere_y, 'ZData', sphere_z);
            end
        end
    end
end

function forces = get_force_at_time(current_time, controller_log)
    % 从控制器日志中获取指定时刻的力值
    forces = [0, 0, 0]; % 默认值
    
    if isempty(controller_log)
        return;
    end
    
    % 去重并排序
    [~, unique_indices] = unique(controller_log(:,1), 'stable');
    unique_log = controller_log(unique_indices, :);
    
    if size(unique_log, 2) >= 6
        log_time = unique_log(:,1);
        force_data = unique_log(:,4:6); % f_mx, f_my, f_mz 位于第4-6列
        
        if length(log_time) > 1
            % 线性插值获取当前时刻的力值
            forces = interp1(log_time, force_data, current_time, 'linear', 'extrap');
        else
            forces = force_data(1, :);
        end
    end
end