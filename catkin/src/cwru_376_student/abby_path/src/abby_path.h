// path_planner.h
#ifndef _ABBY_PATH_H_
#define _ABBY_PATH_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <abby_path/path_segment.h> // the service message class
#include <interactive_markers/interactive_marker_server.h>
#include <cwru_srv/simple_int_service_message.h>
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
 
class AbbyPathPlanner {
private:
	ros::NodeHandle *nh_p;
	
	// Odometry
	ros::Subscriber odom_subscriber; // subscriber to odometry
	geometry_msgs::PoseStamped current_pose; // track our current position
	bool odom_is_initialized; // to verify that odom is working
	void odomCallback(const nav_msgs::Odometry& odom_rcvd); // odometry callback

	// interactive marker
	int dest_trigger;
	// geometry_msgs::Point dest_point;
	interactive_markers::InteractiveMarkerServer im_server;
	visualization_msgs::InteractiveMarker im;
	visualization_msgs::InteractiveMarkerControl im_control,
		im_translate_x,
		im_translate_y;
	void initializeInteractiveMarker();
	static void markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &pt);
	
	// Path service
	ros::Publisher segment_pub;
	ros::ServiceServer service, // the ROS service server object for listening/sending
		cmd_service;
	// callback for when a node wants a path segment
	bool serviceCallback(abby_path::path_segmentRequest& request, abby_path::path_segmentResponse& response);
	bool commandCallback(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response);

	// map-to-odom transform
	bool tf_is_initialized;
	tf::TransformListener *tf_p;
	tf::StampedTransform baseLink_wrt_map,
		map_to_odom;
	
public:
	AbbyPathPlanner(ros::NodeHandle& nh);
	// calculate the next path segment for the sequence
	PathSegment* nextSegment();
};

#endif
