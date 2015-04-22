#include "abby_path.h"

// Define the list of path segments, in order.
// The array index becomes the ID of the segment.

AbbyPathPlanner::AbbyPathPlanner(ros::NodeHandle& nh):
	nh_p(&nh),
	im_server("path_marker") {
	// create the publisher
	segment_pub = nh.advertise<geometry_msgs::PointStamped>("path_planner", 1);
	// create and broadcast the service
	service = nh.advertiseService("path_planner_service", &AbbyPathPlanner::serviceCallback, this);
	cmd_service = nh.advertiseService("abby_path_command", &AbbyPathPlanner::commandCallback, this);
	// subscribe to odom
	std::string odom_topic;
	if (!nh.getParam("/odom_topic", odom_topic)) {
		ROS_WARN("path_planner needs ROS param /odom_topic");
		return;
	}
	odom_subscriber = nh.subscribe(odom_topic, 1, &AbbyPathPlanner::odomCallback, this);

	// wait for valid transform data
	/*
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
	*/

	initializeInteractiveMarker();

	ROS_INFO("Ready to fulfill path segment requests");
}


void AbbyPathPlanner::initializeInteractiveMarker() {
    // later, change the reference frame to the "map" frame
    im.header.frame_id = "/base_link"; // the reference frame for pose coordinates
    im.name = "desired_position"; //name the marker
    im.description = "Abby Path Interactive Marker";
     
    geometry_msgs::Point temp_point_start;
    /** specify/push-in the origin for this marker */
    temp_point_start.x = 0.38; 
    temp_point_start.y = 0.0;
    temp_point_start.z = 0.67;

    // create an arrow marker; do this 3 times to create a triad (frame)
    visualization_msgs::Marker arrow_marker_x; //this one for the x axis
    geometry_msgs::Point temp_point;

    arrow_marker_x.type = visualization_msgs::Marker::ARROW;
    // specify/push-in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_x.points.push_back(temp_point);
    // Specify and push in the end point for the arrow 
    temp_point = temp_point_start;
    temp_point.x = 0.2; // arrow is this long in x direction
    arrow_marker_x.points.push_back(temp_point);

    // make the arrow very thin
    arrow_marker_x.scale.x = 0.01;
    arrow_marker_x.scale.y = 0.01;
    arrow_marker_x.scale.z = 0.01;

    arrow_marker_x.color.r = 1.0; // red, for the x axis
    arrow_marker_x.color.g = 0.0;
    arrow_marker_x.color.b = 0.0;
    arrow_marker_x.color.a = 1.0;

    // do this again for the y axis:
    visualization_msgs::Marker arrow_marker_y;
    arrow_marker_y.type = visualization_msgs::Marker::ARROW; 
    // Push in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_y.points.push_back(temp_point);
    // Push in the end point for the arrow 
    temp_point.y = 0.2; // points in the y direction
    arrow_marker_y.points.push_back(temp_point);

    arrow_marker_y.scale.x = 0.01;
    arrow_marker_y.scale.y = 0.01;
    arrow_marker_y.scale.z = 0.01;

    arrow_marker_y.color.r = 0.0;
    arrow_marker_y.color.g = 1.0; // color it green, for y axis
    arrow_marker_y.color.b = 0.0;
    arrow_marker_y.color.a = 1.0;

    // create a control that contains the markers
    im_control.always_visible = true;
    
    im_control.markers.push_back(arrow_marker_x);
    im_control.markers.push_back(arrow_marker_y);
    
    // add the control to the interactive marker
    im.controls.push_back(im_control);

    // create a control that will move the marker
    im_translate_x.name = "move_x";
    im_translate_x.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    /** Create the Y-Axis Control*/
    im_translate_y.name = "move_y";
    im_translate_y.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    im_translate_y.orientation.x = 0; //point this in the y direction
    im_translate_y.orientation.y = 0;
    im_translate_y.orientation.z = 1;
    im_translate_y.orientation.w = 1;

    // add the controls to the interactive marker
    im.controls.push_back(im_translate_x);    
    im.controls.push_back(im_translate_y);
    
    /** Scale Down: this makes all of the arrows/disks for the user controls smaller than the default size */
    im.scale = 0.2;
    
    //let's pre-position the marker, else it will show up at the frame origin by default
    im.pose.position.x = 0.0;
    im.pose.position.y = 0.0;
    im.pose.position.z = 0.0;
    
    // add the interactive marker to our collection 
    // and tell the server to call markerCallback() when feedback arrives for it
    im_server.insert(im, &markerCallback);

    // 'commit' changes and send to all clients
    im_server.applyChanges();
}

// update member fields with odom data
void AbbyPathPlanner::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
	current_pose.pose = odom_rcvd.pose.pose;
	current_pose.header = odom_rcvd.header;
	// we have data, so set to true
	odom_is_initialized = true;

	/*if (tf_is_initialized) {
		geometry_msgs::PoseStamped transformed_pose;
		tf_p->transformPose("map", current_pose, transformed_pose);
		ROS_INFO("Current map position: (%f, %f)", transformed_pose.pose.position.x, transformed_pose.pose.position.y);
	}*/
}

void AbbyPathPlanner::markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& pt) {
	dest_point.x = pt->pose.position.x;
	dest_point.y = pt->pose.position.y;
}

// ROS service callback used to fetch length/heading coordinates from this node
bool AbbyPathPlanner::serviceCallback(path_planner::path_segmentRequest& request,
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

bool AbbyPathPlanner::commandCallback(cwru_srv::simple_int_service_messageRequest& request,
					cwru_srv::simple_int_service_messageResponse& response) {
	dest_trigger = request.req;
	response.resp = true;
	return true;
}

// based on current odometry and current index within the path,
// calculate what our next path segment should be
PathSegment* AbbyPathPlanner::nextSegment(){
	// if we're out of points, return null
	if (dest_trigger == 0) {
		ROS_WARN("No more path segments!");
		return NULL;
	}
	// transform from map to odom space using most recent data
	/*	
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
	*/
	// get the transformed destination coordinates
	double dest_x = dest_point.x,
		   dest_y = dest_point.y;
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
		return NULL;
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
	AbbyPathPlanner abby_path(nh);
	
	// do nothing; delegate further processing to callbacks
    ros::spin();
	
    return 0;
}
