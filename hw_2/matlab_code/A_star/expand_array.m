function exp_array=expand_array(node_x,node_y,gn,xTarget,yTarget,CLOSED,MAX_X,MAX_Y, obs_map)
    %Function to return an expanded array
    %This function takes a node and returns the expanded list
    %of successors,with the calculated fn values.
    %The criteria being none of the successors are on the CLOSED list.
    %
    %Copyright 2009-2010 The MathWorks, Inc.
    
    %EXPANDED ARRAY FORMAT
    %--------------------------------
    %|X val |Y val ||h(n) |g(n)|f(n)|
    %--------------------------------
    
    exp_array=[];
    exp_count=1;
    c2=size(CLOSED,1);%Number of elements in CLOSED including the zeros


    nearby_obstacle_set = calc_obstacles_num(node_x, node_y, obs_map)

    for k= 1:-1:-1
        for j= 1:-1:-1
            if (k~=j || k~=0)  %The node itself is not its successor
                s_x = node_x+k;
                s_y = node_y+j;
                if( (s_x >0 && s_x <=MAX_X) && (s_y >0 && s_y <=MAX_Y))%node within array bound
                    flag=1;
                    % if( (s_x == node_x+1) && (s_y == node_y+1) )
                    %     flag=0;
                    % end
                    for c1=1:c2
                        if(s_x == CLOSED(c1,1) && s_y == CLOSED(c1,2))
                            flag=0;
                        end
                    end%End of for loop to check if a successor is on closed list.

                    % original_obstacle_map = CLOSED(1:size(CLOSED,1)-2, :)
                    % flag = check_feasible(s_x, s_y, node_x, node_y, obs_map)

                    % nearby_obstacle_count = size(nearby_obstacle_set, 1);
                    % if nearby_obstacle_count ~= 1
                    %     vec_to_central_x = s_x-node_x;
                    %     vec_to_central_y = s_y-node_y;

                    %     diff_vec_mul_val = 1;
                    %     for idx = 1: nearby_obstacle_count
                    %         vec_obstable_central_x = nearby_obstacle_set(idx, 1)-node_x;
                    %         vec_obstable_central_y = nearby_obstacle_set(idx, 2)-node_y;
                    %         diff_vec_mul_val = diff_vec_mul_val * vector_angle(vec_to_central_x, vec_to_central_y, vec_obstable_central_x, vec_obstable_central_y);
                    %     end

                    %     same_vec_mul_val = 1;
                    %     for i = 1: nearby_obstacle_count
                    %         vec_obstable_central_x_i = nearby_obstacle_set(i, 1)-node_x;
                    %         vec_obstable_central_y_i = nearby_obstacle_set(i, 2)-node_y;
                    %         for j = i+1: nearby_obstacle_count
                    %             vec_obstable_central_x_j = nearby_obstacle_set(j, 1)-node_x;
                    %             vec_obstable_central_y_j = nearby_obstacle_set(j, 2)-node_y;
                    %             same_vec_mul_val = same_vec_mul_val * vector_angle(vec_obstable_central_x_i, vec_obstable_central_y_i, vec_obstable_central_x_j, vec_obstable_central_y_j);
                    %         end
                    %     end
                    %     if diff_vec_mul_val*same_vec_mul_val == 1
                    %         str=['点' num2str(s_x), num2str(s_y) '是不可行的'];disp(str);
                    %         flag = 0;
                    %     end
                    % end

                    % for idx = 1: size(obs_map, 1)
                    %     if(s_x ~= obs_map(idx,1) || s_y ~= obs_map(idx,2))
                    %         str=['当前点坐标' num2str(s_x), num2str(s_y)];disp(str);
                    %         flag = check_feasible(s_x, s_y, node_x, node_y, obs_map)
                    %     end
                    % end

                    if (flag == 1)
                        exp_array(exp_count,1) = s_x;
                        exp_array(exp_count,2) = s_y;
                        exp_array(exp_count,3) = distance(xTarget,yTarget,s_x,s_y);%distance between node and goal,hn
                        exp_array(exp_count,4) = gn+distance(node_x,node_y,s_x,s_y);%cost of travelling to node��gn
                        exp_array(exp_count,5) = exp_array(exp_count,3)+exp_array(exp_count,4);%fn
                        exp_count=exp_count+1;
                    
                    end%Populate the exp_array list!!!
                end% End of node within array bound
            end%End of if node is not its own successor loop
        end%End of j for loop
    end%End of k for loop    