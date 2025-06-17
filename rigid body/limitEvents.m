function [value,terminal,direction] = limitEvents(t,current_state,p)
    % 提取状态变量
    x = current_state(1);
    y = current_state(3);
    l = current_state(5);
    
    % 事件函数：当value=0时触发事件
    % 正值表示在边界内，负值表示超出边界
    value = [x - p.x_max;    % 1: 撞到 x_max (当x >= x_max时触发)
             p.x_min - x;    % 2: 撞到 x_min (当x <= x_min时触发)
             y - p.y_max;    % 3: 撞到 y_max (当y >= y_max时触发)
             p.y_min - y;    % 4: 撞到 y_min (当y <= y_min时触发)
             l - p.l_max;    % 5: 撞到 l_max (当l >= l_max时触发)
             p.l_min - l];   % 6: 撞到 l_min (当l <= l_min时触发)
    
    % 终止仿真：1表示事件发生时停止积分，0表示不停止
    terminal = [1;1;1;1;1;1];
    
    % 方向：1表示从正到负穿越零点时触发，-1表示从负到正，0表示任意方向
    direction = [-1;-1;-1;-1;-1;-1]; % 都设为-1，表示从正值变为负值时触发
end