function flag = check_feasible(point_x, point_y, xNode, yNode, ORIGINAL_OBSTACLE_MAP)
    
    str = ['检查节点(' num2str(point_x) ',' num2str(point_y) ')是否是当前中心节点(' num2str(xNode) ',' num2str(yNode) ')的可行点'];disp(str);
    flag = 1;
    vec_to_central_x = point_x-xNode;
    vec_to_central_y = point_y-yNode;
    num = size(ORIGINAL_OBSTACLE_MAP, 1);
    nearby_obstacle_set = [];
    nearby_obstacle_count = 0;
    for i = 1: num
       
        obstacle_x = ORIGINAL_OBSTACLE_MAP(i, 1);
        obstacle_y = ORIGINAL_OBSTACLE_MAP(i, 2);
        str = ['当前障碍物点: (' num2str(obstacle_x) ',' num2str(obstacle_y) ')'];disp(str);
        vec_temp_x = obstacle_x-xNode;
        vec_temp_y = obstacle_y-yNode;
        vec_temp = [vec_temp_x, vec_temp_y];
        % if (  ((obstacle_x-xNode)*(obstacle_y-yNode) == 0)   &&   (abs(obstacle_x-xNode)*abs(obstacle_y-yNode) <= 1)  )
        % if ( obstacle_x-xNode==0 && (obstacle_y-yNode == 1 || obstacle_y-yNode == -1) ) || ( obstacle_y-yNode==0 && (obstacle_x-xNode == 1 || obstacle_x-xNode == -1) )
        % if sum(vec_temp == [-1 0]) == 2 || sum(vec_temp == [1 0]) == 2 || sum(vec_temp == [0 -1]) == 2 || sum(vec_temp == [0 1]) == 2
        if distance(obstacle_x, obstacle_y, xNode, yNode) <= sqrt(2)
            nearby_obstacle_count = nearby_obstacle_count+1;
            nearby_obstacle_set(nearby_obstacle_count, :) = [obstacle_x, obstacle_y];
        end
    end
    nearby_obstacle_set
    str = ['当前中心节点(' num2str(xNode) ',' num2str(yNode) ')周围的障碍物点有 ' num2str(nearby_obstacle_count) '个'];disp(str);

    if nearby_obstacle_count == 1
        flag = 1;
    

    else

       diff_vec_mul_val = 1;
       for idx = 1: nearby_obstacle_count
           vec_obstable_central_x = nearby_obstacle_set(idx, 1)-xNode;
           vec_obstable_central_y = nearby_obstacle_set(idx, 2)-yNode;
           diff_vec_mul_val = diff_vec_mul_val * vector_angle(vec_to_central_x, vec_to_central_y, vec_obstable_central_x, vec_obstable_central_y);
       end

       same_vec_mul_val = 1;
       for i = 1: nearby_obstacle_count
           vec_obstable_central_x_i = nearby_obstacle_set(i, 1)-xNode;
           vec_obstable_central_y_i = nearby_obstacle_set(i, 2)-yNode;
           for j = i+1: nearby_obstacle_count
               vec_obstable_central_x_j = nearby_obstacle_set(j, 1)-xNode;
               vec_obstable_central_y_j = nearby_obstacle_set(j, 2)-yNode;
               same_vec_mul_val = same_vec_mul_val * vector_angle(vec_obstable_central_x_i, vec_obstable_central_y_i, vec_obstable_central_x_j, vec_obstable_central_y_j);
           end
        end

        if diff_vec_mul_val*same_vec_mul_val == -1
            flag = 1;
        else
            flag = 0;
        end

    end
end

%% flag = 1 it's feasible
%% flag = 0 it's forbidden