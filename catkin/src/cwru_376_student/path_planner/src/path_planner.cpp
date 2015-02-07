#include <ros/ros.h>
#include <path_planner/path_segment.h>
#include <math.h>

class PathSegment {
public:
	const float heading, distance;
	
	PathSegment(float h, float d):
		heading(h),
		distance(d) { };
};

class PathPlanner {
private:
	static PathPlanner *instance;
    static PathSegment[] segments;
	static int segment_count;
public:
	PathPlanner(ros::NodeHandle& nh);
	// callback for when a node wants a path segment
	static bool serviceCallback(path_planner::path_segmentRequest& request,
								path_planner::path_segmentResponse& response);
};

PathPlanner::PathPlanner(ros::NodeHandle& nh) {
	// set the singleton instance variable
	instance = this;
	// specify the path segments
	segments = {
		PathSegment(0.0, 4.8),
		PathSegment(-M_PI/2.0, 0.0),
		PathSegment(0.0, 12.2),
		PathSegment(-M_PI/2.0, 0.0),
		PathSegment(0.0, 5.0)
	};
	// the number of segments
	segment_count = 5;
};

PathPlanner *PathPlanner::instance;

bool PathPlanner::serviceCallback(path_planner::path_segmentRequest& request,
								  path_planner::path_segmentResponse& response) {
	ROS_INFO("Retrieving a path segment from the path_planner service");
	// ensure the request ID is within the bounds
	if (request.id < 0 || request.id >= segment_count) {
		ROS_WARN("Segment ID out of range.");
		return false;
	}
	// populate the response object and return
	PathSegment *seg = segments[request.id];
	response.heading = seg->heading;
	response.distance = seg->distance;
	return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_planner"); // name this node
    ros::NodeHandle nh;
	
	PathPlanner path_planner(nh);
	
	// do nothing; delegate further processing to callbacks
    ros::spin();
	
    return 0;
}
