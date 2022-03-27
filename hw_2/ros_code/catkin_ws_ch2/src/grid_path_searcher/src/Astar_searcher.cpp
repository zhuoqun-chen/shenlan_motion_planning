#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0); //global_xyz_l 对应于 _map_lower （-25, -25, 0）
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0); //global_xyz_u 对应于 _map_upper （25，25，5）  global_xyz_upper
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */

    /*******************************************************************************************/
    //对于周围一圈的点，只要没有被占据，就都可以是当前节点的邻居节点，然后在该函数执行完后，再判断这些邻居是否合法
    Vector3i current_idx = currentPtr->index;
    // current_coord = currentPtr->coord;
    Vector3d current_coord = gridIndex2coord(current_idx);

    int current_idx_x = current_idx(0);
    int current_idx_y = current_idx(1);
    int current_idx_z = current_idx(2);

    // ROS_INFO("Please search current node:(%d, %d, %d) neighbor_list", current_idx_x,current_idx_y,current_idx_z);

    for(int i = current_idx_x-1; i <= current_idx_x+1; i++)
        for(int j = current_idx_y-1; j <= current_idx_y+1; j++)
            for(int k = current_idx_z-1; k <= current_idx_z+1; k++){
                //if the temp grid is free
                if(    (isFree(i, j, k) == true)  &&  
                   (current_idx_x != i || current_idx_y != j ||  current_idx_z != k)   )
                {
                    GridNodePtr neighborPtr = GridNodeMap[i][j][k];

                    neighborPtr->index(0) = i;
                    neighborPtr->index(1) = j;
                    neighborPtr->index(2) = k;
                    neighborPtr->coord = gridIndex2coord(neighborPtr->index);

                    neighborPtrSets.push_back(neighborPtr);
                    Vector3d temp_coord = neighborPtr->coord;
                    //calculate the edge cost Cnm
                    double distance = (temp_coord-current_coord).norm();
                    edgeCostSets.push_back(distance);
                    // ROS_WARN("Calc distance between current node and neighbor node: %f", distance);
                }
            }

    /*******************************************************************************************/

}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    /*******************************************************************************************/
#define _use_Euclidean 0
#define _use_Euclidean_tie_breaker 1

hello

asdsadsad 

#define _use_Diagonal_tie_breaker 1

#if _use_Euclidean
{
    //方案1：Euclidean Heuristic
    Vector3i idx_1 = node1->index;
    Vector3d coord_1 = gridIndex2coord(idx_1);

    Vector3i idx_2 = node2->index;
    Vector3d coord_2 = gridIndex2coord(idx_2);

    Vector3d diff_coord = coord_1-coord_2;
    double distance = diff_coord.norm();

    #if !_use_Euclidean_tie_breaker
        return distance;
    #elif _use_Euclidean_tie_breaker
        return distance * (1.0 + 0.001);// 加入 p=0.001 的 Tie Breaker
    #endif/*_use_Euclidean_tie_breaker*/
}

#else/*_use_Diagonal*/
{
    //方案2：3D Diagonal Heuristic
    //(Ref: [python - Calculating 'Diagonal Distance' in 3 dimensions for A* path-finding heuristic - Stack Overflow]
    //(https://stackoverflow.com/questions/53116475/calculating-diagonal-distance-in-3-dimensions-for-a-path-finding-heuristic))
    Vector3i idx_1 = node1->index;
    Vector3d coord_1 = gridIndex2coord(idx_1);

    Vector3i idx_2 = node2->index;
    Vector3d coord_2 = gridIndex2coord(idx_2);

    Vector3d diff_abs = (coord_1 - coord_2).cwiseAbs();
    double diff_abs_x = diff_abs(0);
    double diff_abs_y = diff_abs(1);
    double diff_abs_z = diff_abs(2);
    double dmin = diff_abs.minCoeff();
    double dmax = diff_abs.maxCoeff();
    double dmid = diff_abs_x + diff_abs_y + diff_abs_z - dmin - dmax;
    double D3 = std::sqrt(3.0);
    double D2 = std::sqrt(2.0);
    double D1 = 1.0;
    double h_value = (D3 - D2) * dmin + (D2 - D1) * dmid + D1 * dmax;

    #if !_use_Diagonal_tie_breaker
        return h_value;
    #elif _use_Diagonal_tie_breaker
        // 加入 p=0.001 的 Tie Breaker
        // return h_value * (1.0 + 0.005);

        //加入希望尽可能走直线的度量：尽可能和起点到终点的连线的参考向量的夹角尽可能小，也就是内积尽可能大，夹角越小的额外的代价也越小，因此用减法。
        
        //2022-03-27：发现这两句有问题啊！start_coord不是真实坐标是（25, 25, 5）而是索引是（25, 25, 5）啊！改一下看看有问题没！
        // Vector3d start_coord;
        // start_coord << 25, 25, 5;

        Vector3i start_idx;
        start_idx << 25, 25, 5;
        Vector3d start_coord = gridIndex2coord(start_idx);
        
        Vector3d ref_vec = start_coord - coord_2; //start_pt - end_pt
        Vector3d current_vec = coord_1 - coord_2;
        double dot_product = ref_vec.dot(current_vec);
        return h_value + 0.001*dot_product;
    #endif/*_use_Diagonal_tie_breaker*/
}
#endif/*_use_Euclidean*/
    /*******************************************************************************************/
    // return 0;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //为什么把Vector3d的点先转化为Vector3i，然后又把Vector3i给转化为Vector3d？？？？？？？？？？？？？？？？？？？？？？？？？
    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */

    /*******************************************************************************************/
    //应该完成把其它地图点的g值设为inf并且初始化 closedSet 为空，
    //这个 closedSet 在主函数中会被 visVisitedNode() 可视化
    //让 currentPtr 指向 startPtr，表明初始状态下 startPtr 在 openSet 中的f(n)值最小
    //closedSet应该使用什么数据类型呢？到底是使用Vector3d呢还是GridNodePtr呢？
    vector<GridNodePtr> closedSet;

    //有个疑问，最开始的startPtr初始化了id和coord，为什么没有初始化index和dir呢？
    //嗷嗷，我sb了！GridNodePtr startPtr = new GridNode(start_idx, start_pt)指定了index
    //GridNode中的这个dir是干什么用的呢，是用于到时候判断哪些节点是可扩展用的吗？
    currentPtr = startPtr;
    currentPtr->nodeMapIt = openSet.begin(); //每次遍历时的currentPtr指向的节点应该记录下它在openSet当中的位置

    ROS_INFO("start node:(%d, %d, %d)", startPtr->index(0),startPtr->index(1),startPtr->index(2));
    ROS_INFO("end node:(%d, %d, %d)", goalIdx(0),goalIdx(1),goalIdx(2));
    ROS_WARN("The closedSet has been initiated!");
    ROS_WARN("Currently the startPtr in the openSet has the smallest f(n)!");

    int DEBUG_LOOP_COUNT=0;


    /*******************************************************************************************/

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */

        /*******************************************************************************************/
        //multimap将{key,value}当做元素，允许重复元素。multimap根据key的排序准则⾃动将元素排序，因此使⽤时只需考虑插⼊和删除操作即可。
        
        //注意应该始终使得currentPtr的nodeMapIt指向：
        //openSet中key最小的一条记录，它大部分情况下就是begin()迭代器，
        //但是也不一定，比如终点所在的一条记录有可能被插入到openSet最小key的最后一条，这时
        //就要对最小key的所有记录遍历一遍，看看是否有终点GridNode，如果有那么currentPtr
        //的nodeMapIt就应该指向这条非常独特的记录。

        DEBUG_LOOP_COUNT+=1;
        ROS_INFO("WHILE COUNT = %d", DEBUG_LOOP_COUNT);

        typedef multimap<double, GridNodePtr>::iterator nodeIter;
        typedef pair<nodeIter, nodeIter> nodeIter_pair;

        double min_fScore = openSet.begin()->first;
        nodeIter_pair node_iter_pair_ = openSet.equal_range(min_fScore);

        int find_goalIdx_flag = 0;
        for(nodeIter it = node_iter_pair_.first; it != node_iter_pair_.second; it++)
        {
            if( it->second->index == goalIdx)
            {
                currentPtr = it->second;
                find_goalIdx_flag = 1;
                break;
            }
        }



        if(find_goalIdx_flag == 0)
            currentPtr = openSet.begin()->second;

        ROS_WARN("The current find_goalIdx_flag = %d", find_goalIdx_flag);

        openSet.erase(currentPtr->nodeMapIt);
        ROS_WARN("One record has been erased from openSet, now has %d keys in it.", openSet.size());
        currentPtr->id = -1; //标记当前GridNode已经被expanded，接下来放入closed_list中
        closedSet.push_back(currentPtr);//把当前被expanded的节点放入closed_list中
        /*******************************************************************************************/

        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost is %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */

            
            /*******************************************************************************************/
            neighborPtr = neighborPtrSets[i];
            // ROS_INFO("CHECK Neighbor: %d  *** id = %d", i, neighborPtr->id);
            /*******************************************************************************************/

            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */

                /*******************************************************************************************/
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
                neighborPtr->id = 1;
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) );
                /*******************************************************************************************/
                continue;
            }
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */

                /*******************************************************************************************/
                if( neighborPtr->gScore > currentPtr->gScore + edgeCostSets[i])
                {
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                    //此时说明从currentPtr到达该neighborPtr是更优的选择，那么就要改变cameFrom指针，让它指向currentPtr
                    neighborPtr->cameFrom = currentPtr; 
                }
                /*******************************************************************************************/

                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */

    /*******************************************************************************************/
    while(terminatePtr->cameFrom != NULL)
    {
        gridPath.push_back(terminatePtr);
        terminatePtr = terminatePtr->cameFrom;
    }
    gridPath.push_back(terminatePtr);//此时终点变为了起点，也要压进去
    /*******************************************************************************************/

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}