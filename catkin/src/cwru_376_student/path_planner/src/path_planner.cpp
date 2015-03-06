#include "path_planner.h"

// Define the list of path segments, in order.
// The array index becomes the ID of the segment.
/*
PathSegment path_segments_[] = {
	//PathSegment(0.0, 4.0),
	// PathSegment(-M_PI/2.0, 0.0),
	// PathSegment(M_PI, 0.0),
	// PathSegment(-M_PI/2.0, 0.0)
	PathSegment(0.0, 4.4),
	PathSegment(-M_PI/2.0, 0.0),
	PathSegment(0.0, 12.2),
	PathSegment(-M_PI*0.53, 0.0),
	PathSegment(0.0, 6.0)
};
*/

PathPlanner::PathPlanner(ros::NodeHandle& nh): nh_p(&nh) {
	// specify the path segments
	// segments = path_segments_;
	// the number of segments
	// segment_count = 5;
	// create and broadcast the service
	service = nh.advertiseService("path_planner_service", &PathPlanner::serviceCallback, this);
	// subscribe to odom
	std::string odom_topic;
	if (!nh.getParam("/odom_topic", odom_topic)) {
		ROS_WARN("path_planner needs ROS param /odom_topic");
		return;
	}
	odom_subscriber = nh.subscribe(odom_topic, 1, &PathPlanner::odomCallback, this);
	
	ROS_INFO("Ready to fulfill path segment requests");
}

// update member fields with odom data
void PathPlanner::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
	current_pose.pose = odom_rcvd.pose.pose;
	current_pose.header = odom_rcvd.header;
	// we have data, so set to true
	odom_is_initialized = true;
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
		return NULL;
	}
	// get the destination coordinates
	double dest_x = path_x[path_index],
		   dest_y = path_y[path_index];
	
	// TODO: transform the map coordinates into odom space using most recent data
	ROS_INFO("current position: (%f, %f, %f); destination: (%f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, atan2(current_pose.pose.orientation.w, current_pose.pose.orientation.z), dest_x, dest_y);
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
	double heading = (length==0.0? 0.0: acos((px*dx + py*dy)/length)); // don't divide by zero
	
	ROS_INFO("Pre-segment h=%f, l=%f", heading, length);
	if (heading < 0.05) { // if our angle is approximately zero, then we can go straight
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
