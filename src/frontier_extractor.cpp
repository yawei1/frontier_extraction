#include "frontier_extractor.h"

FrontierExtractor::FrontierExtractor(ros::NodeHandle nh)
{
	UNKNOWN = -1;
	OBSTACLE = 100;
	FREE_SPACE = 0;
	pub_frontierMap = nh.advertise<nav_msgs::OccupancyGrid>("frontier_cells", 50, true);
	pub_frontierCentroid = nh.advertise<nav_msgs::OccupancyGrid>("centroid_cells", 50, true);
}


FrontierExtractor::~FrontierExtractor()
{
}


void FrontierExtractor::costMapCallback(const nav_msgs::OccupancyGrid& costmap)
{
	ROS_INFO("Map height is %d", costmap.info.height);
	ROS_INFO("Map width is %d", costmap.info.width);
	map_height = costmap.info.height;
	map_width = costmap.info.width;
	double resolution = costmap.info.resolution;
	double origin_x = costmap.info.origin.position.x;
	double origin_y = costmap.info.origin.position.y;

	// create a costmap 2d object
	costmap_2d::Costmap2D costmap_(map_height, map_width, resolution, origin_x, origin_y);
	
	ROS_INFO("Costmap created.");
	occupancy_grid_array = costmap.data;
	frontierExtract(&costmap_);
	pubFrontierMap(costmap);
	getCentroids(&costmap_);
	pubFrontierCentroid(costmap);
}


void FrontierExtractor::frontierExtract(costmap_2d::Costmap2D *costmap_)
{
	ROS_INFO("Begin Frontier Extraction...");
	uint array_length = map_height * map_width;
	for (int i = 0; i < array_length; i++) {
		if (occupancy_grid_array[i] == FREE_SPACE && isFrontier(i)) {
			// add to frontiers	
			frontiers.push_back(i);
		}
	}
	ROS_INFO("Frontiers size %d", (int)frontiers.size());
	// std::ofstream frontiers_file ("frontiers_file.csv");
	// for (int i = 0; i < frontiers.size(); i++) {
	// // 	ROS_INFO("Frontiers: %d", frontiers[i]);
	// 	frontiers_file << frontiers[i] << ",";
	// 	ROS_INFO("writing...");
	// }
	// frontiers_file.close();
}


void FrontierExtractor::getCentroids(costmap_2d::Costmap2D *costmap_)
{	
	ROS_INFO("Now getting centroids...");

	// store all nbors of a cell
	std::vector<int> region;	

	while (!frontiers.empty()) {
		std::pair<uint, uint> coordinate;
		int cell = frontiers.front();
		frontiers.erase(frontiers.begin());

		getRegion(cell, region);

		// if region too small
		if (region.size() < 10) {
			region.clear();
			continue;
		}

		uint centroid_x = 0;
		uint centroid_y = 0;
		uint  mx, my;
		for (int index = 0; index < region.size(); index++) {
			costmap_->indexToCells(region[index], mx, my);
			// mx = region[index] / map_width;
			// my = region[index] % map_width;
			centroid_x += mx;
			centroid_y += my;
		}
		centroid_x /= (uint)region.size();
		centroid_y /= (uint)region.size();
		centroid_cells.push_back(centroid_y * map_width + centroid_x);
		coordinate.first = centroid_x;
		coordinate.second = centroid_y;
		region.clear();
		centroids.push_back(coordinate);
	}
	// for (int i = 0; i < centroids.size(); i++) {
	// 	ROS_INFO("Centroids x and y, %d, %d", centroids[i].first, centroids[i].second);
	// }
	// ROS_INFO("Centroids number %d", (int)centroids.size());
}


void FrontierExtractor::getRegion(int cell, std::vector<int> & region)
{
	std::queue<int> cell_queue;
	cell_queue.push(cell);
	int current;
	while(!cell_queue.empty()) {
		std::priority_queue<int> del_cells;
		current = cell_queue.front();
		cell_queue.pop();
		region.push_back(current);
		for (int i = 0; i < frontiers.size(); i++) {
			if (isAdjacent(current, frontiers[i])) {
				cell_queue.push(frontiers[i]);
				del_cells.push(i);
			}
		}
		while (!del_cells.empty()) {
			frontiers.erase(frontiers.begin()+del_cells.top());
			del_cells.pop();
		}
	}
}


bool FrontierExtractor::isAdjacent(int cell1, int cell2)
{
	if (cell1 == cell2) {
		ROS_ERROR("Two cells are the same.");
	}
	int adjacent;
	for (int i = 0; i < 8; i++) {
		adjacent = getAdjacents(cell1 ,i);
		if (adjacent == cell2) {
			return true;
		}
	}
	return false;
}


bool FrontierExtractor::isFrontier(int point)
{
	int adjacent[8];
	for (int i = 0; i < 8; i++) {
		adjacent[i] = getAdjacents(point, i);
	}
	for (int i = 0; i < 8; i++) {
		if (adjacent[i] != -1 && occupancy_grid_array[adjacent[i]] == UNKNOWN) {
			if (isSufficient(adjacent[i])) {
				return true;
			}
		}
	}
	return false;
}


bool FrontierExtractor::isSufficient(int point) 
{
	int adjacent[8];
	int count = 0;
	for (int i = 0; i < 8; i++) {
		adjacent[i] = getAdjacents(point, i);
	}
	for (int i = 0; i < 8; i++) {
		if (adjacent[i] != -1 && occupancy_grid_array[adjacent[i]] == UNKNOWN) {
			count++;
		}
	}
	if (count > 2) {
		return true;
	} else {
		return false;
	}
}


int FrontierExtractor::getAdjacents(int point, int direction)
{
	if (direction == 0) { // left
		if (point % map_width != 0) {
			return point - 1;
		}
	} else if (direction == 1) { // upleft
		if (point % map_width != 0 && point >= map_width) {
			return point - 1 - map_width;
		}
	} else if (direction == 2) { // up
		if (point >= map_width) {
			return point - map_width;
		}
	} else if (direction == 3) { // upright
		if (point >= map_width && ((point + 1) % map_width != 0)) {
			return point - map_width + 1;
		}
	} else if (direction == 4) { // right
		if ((point + 1) % map_width != 0) {
			return point + 1;
		}
	} else if (direction == 5) { // downright
		if ((point + 1) % map_width != 0 && (point / map_width) < (map_height - 1)) {
			return point + map_width + 1;
		}
	} else if (direction == 6) { // down
		if ((point / map_width) < (map_height - 1)) {
			return point + map_width;
		}
	} else if (direction == 7) { // downleft
		if ((point / map_width) < (map_height - 1) && (point % map_width != 0)) {
			return point + map_width - 1;
		}
	}
	return -1;
}

void FrontierExtractor::pubFrontierMap(const nav_msgs::OccupancyGrid & costmap)
{
	ROS_INFO("Publish frontier cells...");
	frontierMap.header.seq = costmap.header.seq;
    frontierMap.header.stamp = costmap.header.stamp;
    frontierMap.header.frame_id = costmap.header.frame_id;
    
    frontierMap.info.map_load_time = costmap.info.map_load_time;
    frontierMap.info.resolution = costmap.info.resolution;
    frontierMap.info.width = costmap.info.width;
    frontierMap.info.height = costmap.info.height;
    
    frontierMap.info.origin.position.x = costmap.info.origin.position.x;
    frontierMap.info.origin.position.y = costmap.info.origin.position.y;
    frontierMap.info.origin.position.z = costmap.info.origin.position.z;
    frontierMap.info.origin.orientation.x = costmap.info.origin.orientation.x;
    frontierMap.info.origin.orientation.y = costmap.info.origin.orientation.y;
    frontierMap.info.origin.orientation.z = costmap.info.origin.orientation.z;
    frontierMap.info.origin.orientation.w = costmap.info.origin.orientation.w;

    frontierMap.data = costmap.data;
    for (int i = 0; i < costmap.data.size(); i++) {
    	frontierMap.data[i] = -1;
    }

    for (int i = 0; i < frontiers.size(); i++) {
    	frontierMap.data[frontiers[i]] = 0;
    }
    pub_frontierMap.publish(frontierMap);
}

void FrontierExtractor::pubFrontierCentroid(const nav_msgs::OccupancyGrid & costmap)
{
	ROS_INFO("Publishing frontier centroids...");
	frontierCentroid.header.seq = costmap.header.seq;
    frontierCentroid.header.stamp = costmap.header.stamp;
    frontierCentroid.header.frame_id = costmap.header.frame_id;
    
    frontierCentroid.info.map_load_time = costmap.info.map_load_time;
    frontierCentroid.info.resolution = costmap.info.resolution;
    frontierCentroid.info.width = costmap.info.width;
    frontierCentroid.info.height = costmap.info.height;
    
    frontierCentroid.info.origin.position.x = costmap.info.origin.position.x;
    frontierCentroid.info.origin.position.y = costmap.info.origin.position.y;
    frontierCentroid.info.origin.position.z = costmap.info.origin.position.z;
    frontierCentroid.info.origin.orientation.x = costmap.info.origin.orientation.x;
    frontierCentroid.info.origin.orientation.y = costmap.info.origin.orientation.y;
    frontierCentroid.info.origin.orientation.z = costmap.info.origin.orientation.z;
    frontierCentroid.info.origin.orientation.w = costmap.info.origin.orientation.w;

    frontierCentroid.data = costmap.data;
    for (int i = 0; i < costmap.data.size(); i++) {
    	frontierCentroid.data[i] = -1;
    }

    for (int i = 0; i < centroid_cells.size(); i++) {
    	frontierCentroid.data[centroid_cells[i]] = 0;
    }
    pub_frontierCentroid.publish(frontierCentroid);
}