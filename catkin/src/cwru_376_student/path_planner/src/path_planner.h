// path_planner.h
#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_planner/path_segment.h> // the service message class
#include <math.h>
#include <vector>

// PathSegment stores a linear segment of the path
class PathSegment {
public:
	// heading in radians, distance in meters
	const double heading, distance;
	
	// simple constructor, just assign the values
	PathSegment(double h, double d):
		heading(h),
		distance(d) { }
};
 
class PathPlanner {
private:
	ros::NodeHandle *nh_p;
	// the x and y coordinates for the path
	std::vector<double> path_x,
						path_y;
	int path_index;
	//PathSegment *segments; // the segments associated with this path
	//int segment_count; // the number of segments (should be the length of the array)
	
	// Odometry
	ros::Subscriber odom_subscriber; // subscriber to odometry
	geometry_msgs::PoseStamped current_pose; // track our current position
	bool odom_is_initialized; // to verify that odom is working
	void odomCallback(const nav_msgs::Odometry& odom_rcvd); // odometry callback
	
	// Path service
	ros::ServiceServer service; // the ROS service server object for listening/sending
	// callback for when a node wants a path segment
	bool serviceCallback(path_planner::path_segmentRequest& request, path_planner::path_segmentResponse& response);

	// map-to-odom transform
	bool tf_is_initialized;
	tf::TransformListener *tf_p;
	tf::StampedTransform map_to_odom;
	
public:
	PathPlanner(ros::NodeHandle& nh);
	// load the path data
	bool loadPath();
	// calculate the next path segment for the sequence
	PathSegment* nextSegment();
};

#endif
