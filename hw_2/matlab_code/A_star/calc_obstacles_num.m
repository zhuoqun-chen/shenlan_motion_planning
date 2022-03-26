function nearby_obstacle_set = calc_obstacles_num(xNode, yNode, ORIGINAL_OBSTACLE_MAP)
    num = size(ORIGINAL_OBSTACLE_MAP, 1);
    nearby_obstacle_set = [];
    nearby_obstacle_count = 0;
    for i = 1: num
        obstacle_x = ORIGINAL_OBSTACLE_MAP(i, 1);
        obstacle_y = ORIGINAL_OBSTACLE_MAP(i, 2);
        str = ['当前障碍物点: (' num2str(obstacle_x) ',' num2str(obstacle_y) ')'];disp(str);
        if distance(obstacle_x, obstacle_y, xNode, yNode) <= sqrt(2)
            nearby_obstacle_count = nearby_obstacle_count+1;
            nearby_obstacle_set(nearby_obstacle_count, :) = [obstacle_x, obstacle_y];
        end
    end
    nearby_obstacle_set
    str = ['当前中心节点(' num2str(xNode) ',' num2str(yNode) ')周围的障碍物点有 ' num2str(nearby_obstacle_count) '个'];disp(str);
end