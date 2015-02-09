#include <ros/ros.h>
#include <path_planner/path_segment.h> // the service message class
#include <math.h>

// PathSegment stores a linear segment of the path
class PathSegment {
public:
	// heading in radians, distance in meters
	const float heading, distance;
	
	// simple constructor, just assign the values
	PathSegment(float h, float d):
		heading(h),
		distance(d) { }
};

// Define the list of path segments, in order.
// The array index becomes the ID of the segment.
PathSegment path_segments_[] = {
	PathSegment(0.0, 4.8),
	PathSegment(-M_PI/2.0, 0.0),
	PathSegment(0.0, 12.2),
	PathSegment(-M_PI/2.0, 0.0),
	PathSegment(0.0, 5.0)
};

// PathPlanner singleton 
class PathPlanner {
private:
	static PathPlanner *instance; // singleton instance variable
	static PathSegment *segments; // the segments associated with this path
	static int segment_count; // the number of segments (should be the length of the array)
	
	ros::ServiceServer service; // the ROS service server object for listening/sending
public:
	PathPlanner(ros::NodeHandle& nh);
	// callback for when a node wants a path segment
	static bool serviceCallback(path_planner::path_segmentRequest& request,
								path_planner::path_segmentResponse& response);
};

PathPlanner::PathPlanner(ros::NodeHandle& nh) {
	// set the singleton instance variable
	if (!instance)
		instance = this;
	// specify the path segments
	segments = path_segments_;
	// the number of segments
	segment_count = 5;
	// create and broadcast the service
	service = nh.advertiseService("path_planner_service", serviceCallback);
	ROS_INFO("Ready to fulfill path segment requests");
};

// initialize the static fields
PathPlanner *PathPlanner::instance = NULL;
PathSegment *PathPlanner::segments;
int PathPlanner::segment_count;

bool PathPlanner::serviceCallback(path_planner::path_segmentRequest& request,
								  path_planner::path_segmentResponse& response) {
	ROS_INFO("Retrieving a path segment from the path_planner service");
	// ensure the request ID is within the bounds
	if (request.id < 0 || request.id >= segment_count) {
		ROS_WARN("Segment ID out of range.");
		return false;
	}
	// populate the response object and return
	PathSegment *seg = &(segments[request.id]);
	response.heading = seg->heading;
	response.distance = seg->distance;
	return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_planner"); // name this node
    ros::NodeHandle nh;
	
	// create a path planner, which will do all of the work from here on
	PathPlanner path_planner(nh);
	
	// do nothing; delegate further processing to callbacks
    ros::spin();
	
    return 0;
}
