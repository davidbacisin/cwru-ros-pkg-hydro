#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <cwru_srv/simple_int_service_message.h> // for the process_mode service
#include <cwru_srv/simple_bool_service_message.h> // for the move_trigger service

// overall mode of the node
enum MasterMode {
	IDLE = 0,	
	FIND_CAN,
	APPROACH_POSITION,
	OPEN_HAND,
	CARTESIAN_DESCENT,
	CLOSE_HAND,
	CARTESIAN_ASCENT
};
MasterMode mode = IDLE;
bool masterPlannerCallback(cwru_srv::simple_int_service_messageRequest& req, cwru_srv::simple_int_service_messageResponse& resp) {
	mode = (MasterMode) req.req;
	resp.resp = 1;
	return true;
}

// can data and callback
struct {
	bool is_found;
	geometry_msgs::PointStamped top;
	ros::Subscriber subscriber;	
} can;
void canTopCallback(const geometry_msgs::PointStamped& pt) {
	can.top = pt;
	can.is_found = true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "master_planner"); // name this node
	ros::NodeHandle nh;
	
	// initializations
	ros::ServiceServer master_service = nh.advertiseService("master_planner", masterPlannerCallback);
	
	can.subscriber = nh.subscribe("/can_top_position", 1, canTopCallback);

	ROS_INFO("master_planner is ready");
	while (ros::ok()) {
		switch (mode) {
			case FIND_CAN:{
				can.is_found = false;

				// call object_finder service
				cwru_srv::simple_int_service_messageRequest req;
				cwru_srv::simple_int_service_messageResponse resp;
				
				req.req = 1; // process_mode = 1 means find a can
				if (!ros::service::call("process_mode", req, resp)) {
					ROS_WARN("process_mode service not successful. Is object_finder running?");
					mode = IDLE;
					break;
				}
				// wait for response
				while (!can.is_found) {
					ros::spinOnce();	
					ros::Duration(0.5).sleep();
				}
				// great! we have the can position
				ROS_INFO("Can top found at (%f, %f, %f)", can.top.point.x, can.top.point.y, can.top.point.z);
				// go to next step				
				mode = APPROACH_POSITION;
				break;
			}
			case APPROACH_POSITION:
				// send arm to the approach position
				
				// wait until it's there
				
				break;
			case OPEN_HAND:
				break;
			case CARTESIAN_DESCENT:
				break;
			case CLOSE_HAND:
				break;
			case CARTESIAN_ASCENT:
				break;
			case IDLE:
			default:
				break; // do nothing	
		}

		// spin and sleep
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	return 0;
}
