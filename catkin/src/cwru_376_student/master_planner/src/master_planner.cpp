#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>

#include <cwru_srv/simple_int_service_message.h> // for the process_mode service
#include <cwru_srv/simple_bool_service_message.h> // for the move_trigger service

#include <Eigen/Eigen>

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

// arm data and callback
struct {
	bool is_moving;
	sensor_msgs::JointState joint_states;
	ros::Publisher point_publisher;
	ros::Subscriber joint_subscriber;
} arm;
void jointStateCallback(const sensor_msgs::JointStatePtr& jt) {
	// first check if the new joint states are different from the previous joint states, which means the arm is moving
	// then copy over
	arm.is_moving = false;
	if (arm.joint_states.position.size()) {
		for (int i = 0; i < jt->position.size(); i++) {
			arm.is_moving = arm.is_moving || (fabs(arm.joint_states.position[i] - jt->position[i]) < 0.01);
		}
	}
	arm.joint_states = *jt;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "master_planner"); // name this node
	ros::NodeHandle nh;
	
	// initializations
	ros::ServiceServer master_service = nh.advertiseService("master_planner", masterPlannerCallback);
	can.subscriber = nh.subscribe("/can_top_position", 1, canTopCallback);
	arm.point_publisher = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("example_marker/feedback", 1);
	arm.is_moving = false;

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
				while (!can.is_found && ros::ok()) {
					ros::spinOnce();	
					ros::Duration(0.5).sleep();
				}
				// great! we have the can position
				ROS_INFO("Can top found at (%f, %f, %f)", can.top.point.x, can.top.point.y, can.top.point.z);
				// go to next step				
				mode = APPROACH_POSITION;
				break;
			}
			case APPROACH_POSITION: {
				// send arm to the approach position
				visualization_msgs::InteractiveMarkerFeedback dest;
				dest.header.frame_id = "base_link";
				dest.header.stamp = ros::Time::now();
				dest.client_id = "master_planner_marker";
				dest.marker_name = "des_hand_pose";
				dest.control_name = "";
				dest.event_type = dest.POSE_UPDATE;
				dest.pose.position.x = can.top.point.x;
				dest.pose.position.y = can.top.point.y;
				dest.pose.position.z = can.top.point.z + 0.18; // be a bit above the can
				Eigen::Matrix3d rotation; // calculate the rotation of the hand
				rotation << 0.0, 0.0, 1.0,
					    0.0, 1.0, 0.0,
					    -1.0, 0.0, 0.0;
				Eigen::Quaterniond hand_angle(rotation);
				hand_angle.normalize();
				ROS_INFO("angle %f, %f, %f, %f", hand_angle.x(), hand_angle.y(), hand_angle.z(), hand_angle.w());
				dest.pose.orientation.x = hand_angle.x();
				dest.pose.orientation.y = hand_angle.y() - 1e-3; // shift by a tiny bit, otherwise the ik solver gets stuck
				dest.pose.orientation.z = hand_angle.z();
				dest.pose.orientation.w = hand_angle.w() + 1e-3;
				
				arm.is_moving = true;
				arm.point_publisher.publish(dest);

				// call the service
				cwru_srv::simple_bool_service_messageRequest req;
				cwru_srv::simple_bool_service_messageResponse resp;
		
				req.req = true;
				if (!ros::service::call("move_trigger", req, resp)) {
					ROS_WARN("move_trigger service not successful. Is beta_irb120_im_interface running?");
					mode = IDLE;
					break;
				}

				// wait until it's there
				while (arm.is_moving && ros::ok()) {
					ros::spinOnce();
					ros::Duration(0.5).sleep();
				}
				ROS_INFO("Arm is in approach position");
			
				// go to next step
				mode = OPEN_HAND;
				break;
			}
			case OPEN_HAND:
				// go to next step
				mode = CARTESIAN_DESCENT;
				break;
			case CARTESIAN_DESCENT:
				// go to next step
				mode = CARTESIAN_DESCENT;
				break;
			case CLOSE_HAND:
				// go to next step
				mode = CARTESIAN_ASCENT;
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
