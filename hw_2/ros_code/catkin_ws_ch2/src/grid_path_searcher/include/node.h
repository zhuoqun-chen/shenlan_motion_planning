#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord; 
    Eigen::Vector3i dir;   // direction of expanding 请问这个Vector3i是什么意思？？？？
    Eigen::Vector3i index; // 请问这个Vector3i是什么意思？？？？
	
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = 0;
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();

		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};


#endif
