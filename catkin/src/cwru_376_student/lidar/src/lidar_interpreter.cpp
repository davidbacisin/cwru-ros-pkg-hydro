#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

class LidarInterpreter {
private:
	static LidarInterpreter *instance;
    // subscribe to the lidar topic
    ros::Subscriber lidar_subscriber;
	// publish a float on lidar_nearest
    ros::Publisher lidar_nearest;
	// basic range information from the LIDAR
	float angle_min,
		  angle_max,
		  angle_increment,
		  range_min,
		  range_max;
		  
	// utility method for clamping ping data to the measurable ranges.
	// In the clamped param, returns 0 if no clamping, 1 if clamped on the far end, or -1 if clamped on the near end.
	static float clampPing(int index, const sensor_msgs::LaserScan& laser_scan, int& clamped);
	// utility method for reducing noise in the LIDAR range data
	static float smoothPing(int index, const sensor_msgs::LaserScan& laser_scan);
	
public:
    LidarInterpreter(ros::NodeHandle& nh, char *lidar_topic);
	// callback for when the LIDAR has data
	static void laserCallback(const sensor_msgs::LaserScan& laser_scan);
};

LidarInterpreter *LidarInterpreter::instance;

LidarInterpreter::LidarInterpreter(ros::NodeHandle& nh, char *lidar_topic) {
	// set the instance variable
	instance = this;
	// subscribe to the LIDAR
	lidar_subscriber = nh.subscribe(lidar_topic, 1, laserCallback);
	// broadcast on the lidar_nearest topic
	lidar_nearest = nh.advertise<std_msgs::Float32>("lidar_nearest", 1);
	// initialize the LIDAR range values
	angle_min = 0.0;
	angle_max = 0.0;
	angle_increment = -1.0;
	range_min = 0.0;
	range_max = 0.0;
}

void LidarInterpreter::laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	// have we initialized the LIDAR range data yet?
	if (instance->angle_increment <= 0.0) {
		instance->angle_min = laser_scan.angle_min;
        instance->angle_max = laser_scan.angle_max;
        instance->angle_increment = laser_scan.angle_increment;
        instance->range_min = laser_scan.range_min;
        instance->range_max = laser_scan.range_max;
	}
	
	// filter the ping data
	// clamped pings should not be considered for nearest object
	int ping_clamped = 0;
	// keep updated with the nearest object
	float nearest_object = instance->range_max,
		current_ping = 0.0;
	sensor_msgs::LaserScan laser_scan_cp = laser_scan;
	// start with ping 2 and end with the third-to-last ping so that we don't cause a segmentation fault in smoothPing()
	for (int i=2; i+2 < sizeof(laser_scan.ranges); i++){
		// clamp the ping
		current_ping = clampPing(i, laser_scan_cp, ping_clamped);
		laser_scan_cp.ranges[i] = current_ping;
		// smooth out noise
		current_ping = smoothPing(i, laser_scan_cp);
		// see if the ping had the shortest distance
		if (ping_clamped==0 &&
			laser_scan.ranges[i] < nearest_object){
			nearest_object = laser_scan.ranges[i];
		}
	}
	
	// publish the distance of the nearest object
	std_msgs::Float32 lidar_dist_msg;
	lidar_dist_msg.data = nearest_object;
	instance->lidar_nearest.publish(lidar_dist_msg);
}

float LidarInterpreter::clampPing(int index, const sensor_msgs::LaserScan& laser_scan, int& clamped) {
	if (laser_scan.ranges[index] <= instance->range_min) {
		clamped = -1;
		return instance->range_min;
	}
	else if (laser_scan.ranges[index] >= instance->range_max) {
		clamped = 1;
		return instance->range_max;
	}
	else {
		clamped = 0;
		return laser_scan.ranges[index];
	}
}

float LidarInterpreter::smoothPing(int index, const sensor_msgs::LaserScan& laser_scan) {
	// Perform a weighted average of the ping values
	int weights[] = { 1, 4, 6, 4, 1 },
		sum_weights = 16;
	// accumulate the weighted sum
	float aggregator = 0.0;
	for (int i=0; i < 5; i++){
		aggregator += weights[i] * laser_scan.ranges[index + i - 2];
	}
	// divide by the sum of the weights to determine the smoothed value
	return aggregator / sum_weights;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_interpreter"); // name this node
    ros::NodeHandle nh;
	// argv[1] should be the name of the topic on which to get the LIDAR data
	if (argc < 1) {
		ROS_INFO("LidarInterpreter needs the name of the LIDAR topic as the first argument");
	}
	else {
		LidarInterpreter lidar_interpreter(nh, argv[1]);
		
		// do nothing; delegate further processing to callbacks
		ros::spin();
	}
    return 0;
}
