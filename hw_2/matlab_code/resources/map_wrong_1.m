function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;
    MAP
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    OPEN(OPEN_COUNT,1)=0; %zqchen:弹出第一个点，这个点就是起始点，接下来去找它的child nodes
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;%zqchen:把弹出的第一个点，即起始点加入到closed list中，之前的closed list中的每一行都是障碍物点的x, y坐标
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    DEBUG_COUNT = 1;
    while(1) %you have to dicide the Conditions for while loop exit 
        
        
        disp(["开始第" DEBUG_COUNT "轮次扩展节点"]);
        DEBUG_COUNT = DEBUG_COUNT+1;
        %%Find current unexpanded neighbors
        
        %EXPANDED ARRAY FORMAT
        %--------------------------------
        %|X val |Y val ||h(n) |g(n)|f(n)|
        %--------------------------------
        exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y)
        for row_idx = 1: size(exp_array,1)
            child_xval = exp_array(row_idx, 1);
            child_yval = exp_array(row_idx, 2);
            h = exp_array(row_idx, 3);
            g = exp_array(row_idx, 4);
            f = exp_array(row_idx, 5);
            %zqchen: In math, it means "g(m) = infinite, and it has not been visited before"
            if(MAP(child_xval,child_yval) == 2 || MAP(child_xval,child_yval) == 0)
                fprintf("节点( %d, %d )是先前未访问过的点", floor(child_xval), floor(child_yval))
                %OPEN LIST FORMAT
                %--------------------------------------------------------------------------
                %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
                %-------------------------------------------------------------------------
                OPEN_COUNT = OPEN_COUNT+1;
                %zqchen: It means "Push this node into the queue"
                OPEN(OPEN_COUNT, :) = insert_open(child_xval,child_yval, xNode,yNode, h, g, f)
                str = ['将 MAP(' num2str([floor(child_xval), floor(child_yval)]) ')压入 OPEN']; disp(str);
                %zqchen: This means "This node has been visited"
                MAP(child_xval,child_yval) = -1
            else
                %get the index of the location of this node in the list
                str = ['节点(' num2str([floor(child_xval), floor(child_yval)]) ')先前被访问过,有更好的路径']; disp(str);
                n_index = node_index(OPEN,child_xval,child_yval)
                OPEN(n_index, 4) = xNode;
                OPEN(n_index, 5) = yNode;
                OPEN(n_index, 7) = g
                str=['OPEN 已经被更新了\n'];disp(str);
            end
        end

        i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
        if(i_min == -1)
            break
        end
        if(i_min ~= -1)
            i_min_x = OPEN(i_min, 2);
            i_min_y = OPEN(i_min, 3);
            if ( i_min_x==xTarget && i_min_y==yTarget )
                NoPath = 0;
                break
            else
                str = ['将要弹出的节点在 OPEN 中的索引: ' num2str(i_min) ', 坐标为: ' num2str(floor(i_min_x)) ',' num2str(floor(i_min_y))]; disp(str);
                OPEN(i_min,1)=0;
                CLOSED_COUNT=CLOSED_COUNT+1;
                CLOSED(CLOSED_COUNT,1)=i_min_x;
                CLOSED(CLOSED_COUNT,2)=i_min_y;
                xNode = i_min_x
                yNode = i_min_y
            end
        end
        % for x = xNode-1: 1: xNode+1:
        %     for y = yNode-1: 1: yNode+1:
        %         if( (x ~= xNode && y ~= yNode) && (x ~= 0) && (y ~= 0) ):
        %             if MAP(x,y) ~= -1:
        %                 for 


     %
     %finish the while loop
     %
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
    
    % path = [];
    path = []
    nodes_arr = []
    while(xTarget~=xStart && yTarget~=yStart)
        COUNT = size(nodes_arr, 1) + 1
        nodes_arr(COUNT, 1) = xTarget
        nodes_arr(COUNT, 2) = yTarget
        idx = node_index(OPEN, xTarget, yTarget)
        xTarget = OPEN(idx, 4)
        yTarget = OPEN(idx, 5)
    end
    % COUNT = size(nodes_arr, 1) + 1
    % npdes_arr(COUNT, 1) = xStart
    % nodes_arr(COUNT, 2) = yStart
    % path = nodes_arr
    COUNT = size(nodes_arr, 1) + 1
    nodes_arr(COUNT, 1) = 1
    nodes_arr(COUNT, 2) = 1
    path = nodes_arr
end
