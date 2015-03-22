#include "path_planner.h"

// Define the list of path segments, in order.
// The array index becomes the ID of the segment.

PathPlanner::PathPlanner(ros::NodeHandle& nh): nh_p(&nh) {
	// create the publisher
	segment_pub = nh.advertise<geometry_msgs::PointStamped>("path_planner", 1);
	// create and broadcast the service
	service = nh.advertiseService("path_planner_service", &PathPlanner::serviceCallback, this);
	// subscribe to odom
	std::string odom_topic;
	if (!nh.getParam("/odom_topic", odom_topic)) {
		ROS_WARN("path_planner needs ROS param /odom_topic");
		return;
	}
	odom_subscriber = nh.subscribe(odom_topic, 1, &PathPlanner::odomCallback, this);

	// wait for valid transform data
	tf_is_initialized = false;
	tf_p = new tf::TransformListener(nh);
	while (!tf_is_initialized) {
		try {
			tf_p->lookupTransform("map", "base_link", ros::Time(0), baseLink_wrt_map);
			tf_is_initialized = true;
		}
		catch (tf::TransformException& exception) {
			ROS_ERROR("%s", exception.what());
			tf_is_initialized = false;
			ros::spinOnce();
			ros::Duration(0.5).sleep();
		}
	}
	ROS_INFO("tf is ready");

	ROS_INFO("Ready to fulfill path segment requests");
}

// update member fields with odom data
void PathPlanner::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
	current_pose.pose = odom_rcvd.pose.pose;
	current_pose.header = odom_rcvd.header;
	// we have data, so set to true
	odom_is_initialized = true;

	if (tf_is_initialized) {
		geometry_msgs::PoseStamped transformed_pose;
		tf_p->transformPose("map", current_pose, transformed_pose);
		ROS_INFO("Current map position: (%f, %f)", transformed_pose.pose.position.x, transformed_pose.pose.position.y);
	}
}

// ROS service callback used to fetch length/heading coordinates from this node
bool PathPlanner::serviceCallback(path_planner::path_segmentRequest& request,
								  path_planner::path_segmentResponse& response) {
	ROS_INFO("Retrieving a path segment from the path_planner service");
	/* Need to update how IDs work
	// ensure the request ID is within the bounds
	if (request.id < 0 || request.id >= segment_count) {
		ROS_WARN("Segment ID out of range.");
		return false;
	}
	*/
	// populate the response object and return
	PathSegment *seg = nextSegment();
	if (seg){
		response.heading = seg->heading;
		response.distance = seg->distance;
		delete seg;
		return true;
	}
	else {
		// we have no segment to return
		return false;
	}
}

// load the path data from the param server
bool PathPlanner::loadPath() {
	if (nh_p->getParam("/path_planner/x_coordinates", path_x) &&
		nh_p->getParam("/path_planner/y_coordinates", path_y)) {
		// set to the beginning
		path_index = 0;
		return true;
	}
	else { // failed to get the path coordinates
		// invalidate path_index
		path_index = -1;
		ROS_WARN("Path data could not be loaded from the parameter server.");
		return false;
	}
}

// based on current odometry and current index within the path,
// calculate what our next path segment should be
PathSegment* PathPlanner::nextSegment(){
	// if we're out of points, return null
	if (path_index < 0 ||
		path_index > path_x.size() ||
		path_index > path_y.size()) {
		ROS_WARN("No more path segments!");
		return NULL;
	}
	// transform from map to odom space using most recent data
	geometry_msgs::PointStamped map, odom_point;
		map.header.frame_id = "map";
		map.header.stamp = ros::Time::now();
		map.point.x = path_x[path_index];
		map.point.y = path_y[path_index];
		map.point.z = 0.0;
	try {
		tf_p->transformPoint("odom", map, odom_point);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR("%s", exception.what());
	}
	// get the transformed destination coordinates
	double dest_x = odom_point.point.x,
		   dest_y = odom_point.point.y;
	ROS_INFO("current wrt odom: (%f, %f); destination wrt map: (%f, %f); destination wrt odom: (%f, %f)", current_pose.pose.position.x, current_pose.pose.position.y,
		map.point.x, map.point.y, 
		dest_x, dest_y);
	// find the distance between our current position and our destination
	double dx = dest_x - current_pose.pose.position.x,
		   dy = dest_y - current_pose.pose.position.y;
	double length = sqrt(dx*dx + dy*dy);
	// find the angle between our current orientation and our destination
	/*
	For a quaternion | qz + qw | = 1 (qx and qy = 0 for planar rotation)
		angle = 2*acos(qw) = 2*asin(qz)
	We project this angle to the plane,
		px = cos(angle) = cos(2*acos(qw)) = 2*qw^2 - 1
		py = sin(angle) = sin(2*asin(qz)) = 2*qz*qw
	To find the angle between unit vectors,
		angle_between = acos(A . B)
	Here, A = <px, py>
	and   B = <dx, dy>/length (dx and dy from above)
		angle_between = acos((px*dx + py*dy)/length)
	Finally, we need to find the sign of angle_between.
	This is given by
		sign(px*dy - py*dx)
	*/
	double qz = current_pose.pose.orientation.z,
		   qw = current_pose.pose.orientation.w,
		   px = 2*qw*qw - 1,
		   py = 2*qz*qw;
	if (length < 0.1) { // don't bother. Just go to the next path index.
		path_index++;
		return new PathSegment(0, 0);
	}

	double heading = (length==0.0? 0.0: acos((px*dx + py*dy)/length)); // don't divide by zero
	
	ROS_INFO("Pre-segment h=%f, l=%f", heading, length);
	if (heading < 0.15) { // if our angle is approximately zero, then we can go straight
		heading = 0.0;
		// we should have reached our destination once this segment is done. Go to the next point.
		path_index++;
	}
	else {
		// determine which direction to head
		if ((px*dy - py*dx) < 0) {
			heading = -heading;
		}
		// turn to that direction, but don't move linearly
		length = 0.0;
	}

	ROS_INFO("Path segment h=%f, l=%f", heading, length);
	
	// publish the path segment
	geometry_msgs::PointStamped seg_stamped;
	seg_stamped.header.stamp = ros::Time::now();
	seg_stamped.point.x = length;
	seg_stamped.point.y = heading;
	seg_stamped.point.z = 0.0;
	segment_pub.publish(seg_stamped);
	
	return new PathSegment(heading, length);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_planner"); // name this node
    ros::NodeHandle nh;
	
	// create a path planner, which will do all of the work from here on
	PathPlanner path_planner(nh);
	
	path_planner.loadPath();
	
	// do nothing; delegate further processing to callbacks
    ros::spin();
	
    return 0;
}
