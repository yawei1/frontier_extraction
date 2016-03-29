#ifndef SR_NODE_EXAMPLE_CORE_H
#define SR_NODE_EXAMPLE_CORE_H

// ROS includes.
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <utility>
#include <queue>
#include <fstream>



class FrontierExtractor
{
  public:
    // Constructor
    FrontierExtractor(ros::NodeHandle nh);

    // Destructor
    ~FrontierExtractor();

    // Callback function for map subscriber
    void costMapCallback(const nav_msgs::OccupancyGrid& costmap);

    // Get centroids of frontiers
    void getCentroids(costmap_2d::Costmap2D *costmap_);

    // Frontier extraction
    void frontierExtract(costmap_2d::Costmap2D *costmap_);

    // Classify the frontier
    bool isFrontier(int point);

    // get frontiers region
    void getRegion(int cell, std::vector<int> & region);

    // Adjacents fuction
    int getAdjacents(int point, int direction);

    // Check if two cells are nbors
    bool isAdjacent(int cell1, int cell2);

    // Check the size of unknown area near the boundary point
    bool isSufficient(int point);

    // Frontier publisher
    void pubFrontierMap(const nav_msgs::OccupancyGrid& costmap);

    // Centroid publisher
    void pubFrontierCentroid(const nav_msgs::OccupancyGrid & costmap);

    // map constants
    int UNKNOWN;
    int OBSTACLE;
    int FREE_SPACE;
    uint map_height;
    uint map_width;
    nav_msgs::OccupancyGrid frontierMap;
    nav_msgs::OccupancyGrid frontierCentroid;
    ros::Publisher pub_frontierMap;
    ros::Publisher pub_frontierCentroid;
    std::vector<int8_t> occupancy_grid_array;
    std::vector<int> frontiers;
    std::vector<int> centroid_cells;
    std::vector<std::pair<uint, uint> > centroids;
};
 
#endif 
// SR_NODE_EXAMPLE_CORE_H