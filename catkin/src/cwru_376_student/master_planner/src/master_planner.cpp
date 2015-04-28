#include <ros/ros.h>

enum MasterMode {
	IDLE = 0,	
	FIND_CAN,
	APPROACH_POSITION,
	OPEN_HAND,
	CARTESIAN_DESCENT,
	CLOSE_HAND,
	CARTESIAN_ASCENT
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "master_planner"); // name this node
	ros::NodeHandle nh;

	MasterMode mode;

	while (ros::ok()) {
		switch (mode) {
			case FIND_CAN:
				// call object_finder service and wait for response
				break;
			case APPROACH_POSITON:
				// send arm to the approach position and wait until it's there
				break;
			case OPEN_HAND:
				break;
			case IDLE:
			default:
				break; // do nothing	
		}
	}

	// do nothing; delegate further processing to callbacks
	ros::spin();

	return 0;
}
