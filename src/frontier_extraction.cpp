#include <ros/ros.h>
#include "frontier_extractor.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>


int main(int argc, char *argv[])
{
	// initialization
	ros::init(argc, argv, "frontier_extraction");

	ros::NodeHandle nHandle;

	FrontierExtractor *frontier_extractor = new FrontierExtractor(nHandle);

	// subscribe to global costmap 
	ros::Subscriber sub = nHandle.subscribe("map", 10, &FrontierExtractor::costMapCallback, frontier_extractor);

	// // waypoints publisher
	// ros::Publisher pubWaypoint;
    
 //    pubWaypoint = nHandle.advertise<geometry_msgs::Twist>("map_goal", 10);

	// return the control to ROS
	ros::spin();

	return 0;
}