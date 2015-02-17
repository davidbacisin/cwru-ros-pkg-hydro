#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <string>

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
	// we will calculate the number of pings per scan
	int ping_count,
		ping_start_index,
		ping_end_index;
		  
	// utility method for clamping ping data to the measurable ranges.
	// Returns 0 if no clamping, 1 if clamped on the far end, or -1 if clamped on the near end.
    static int clampPing(int index, sensor_msgs::LaserScan& laser_scan);
	// utility method for reducing noise in the LIDAR range data
    static void smoothPing(int index, sensor_msgs::LaserScan& laser_scan);
	
public:
    LidarInterpreter(ros::NodeHandle& nh, std::string lidar_topic);
	// callback for when the LIDAR has data
	static void laserCallback(const sensor_msgs::LaserScan& laser_scan);
};

LidarInterpreter *LidarInterpreter::instance;

LidarInterpreter::LidarInterpreter(ros::NodeHandle& nh, std::string lidar_topic) {
	// set the instance variable
	instance = this;
	// subscribe to the LIDAR
	lidar_subscriber = nh.subscribe(lidar_topic, 1, laserCallback);
	// broadcast on the lidar_nearest topic
	lidar_nearest = nh.advertise<std_msgs::Float32>("lidar_nearest", 1);
	// initialize the LIDAR range values
	angle_min = 0.0;
	angle_max = 0.0;
	angle_increment = 0.0;
	range_min = 0.0;
	range_max = 0.0;
	// initialize ping_count
	ping_count = 0;
}

void LidarInterpreter::laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	// have we initialized the LIDAR range data yet?
	if (instance->ping_count <= 0) {
		instance->angle_min = laser_scan.angle_min;
        instance->angle_max = laser_scan.angle_max;
        instance->angle_increment = laser_scan.angle_increment;
        instance->range_min = laser_scan.range_min;
        instance->range_max = laser_scan.range_max;
		instance->ping_count = (int) ((fabs(instance->angle_max - instance->angle_min))/instance->angle_increment);
		instance->ping_start_index = (int) ((fabs(-M_PI/4.0 - instance->angle_min))/instance->angle_increment);
		instance->ping_end_index = (int) ((fabs(M_PI/4.0 - instance->angle_min))/instance->angle_increment);
        ROS_INFO("There are %i pings in the laser scan. We are testing pings %i through %i.",
                 instance->ping_count,
                 instance->ping_start_index,
                 instance->ping_end_index);
	}
	
	// filter the ping data
	// clamped pings should not be considered for nearest object
	int ping_clamped = 0;
	// keep updated with the nearest object
    float nearest_object = instance->range_max;
    sensor_msgs::LaserScan laser_scan_copy = laser_scan;
    // clamp all the pings before running the smoothing algorithm
    for (int i=0; i < instance->ping_count; i++){
		// clamp the ping
		ping_clamped = clampPing(i, laser_scan_copy);
        // ROS_INFO("Original ping: %f; Adjusted: %f", laser_scan.ranges[i], laser_scan_copy.ranges[i]);
    }
    // start with at least ping 2 and end with no further than the third-to-last ping so that we don't cause a segmentation fault inside smoothPing
    for (int i=instance->ping_start_index; i < instance->ping_end_index; i++){
        // smooth out noise
        smoothPing(i, laser_scan_copy);
        // see if the ping had the shortest distance
        if (ping_clamped <= 0 &&
			laser_scan_copy.ranges[i] < nearest_object){
			nearest_object = laser_scan_copy.ranges[i];
		}
	}
	
	// publish the distance of the nearest object
	std_msgs::Float32 lidar_dist_msg;
	lidar_dist_msg.data = nearest_object;
	instance->lidar_nearest.publish(lidar_dist_msg);
}

int LidarInterpreter::clampPing(int index, sensor_msgs::LaserScan& laser_scan) {
	if (laser_scan.ranges[index] <= instance->range_min) {
		laser_scan.ranges[index] = instance->range_min;
		return -1;
	}
	else if (laser_scan.ranges[index] >= instance->range_max) {
		laser_scan.ranges[index] = instance->range_max;
		return 1;
	}
	else {
		// no need to set the value, just return 0
		return 0;
	}
}

void LidarInterpreter::smoothPing(int index, sensor_msgs::LaserScan& laser_scan) {
	// Perform a weighted average of the ping values
	int weights[] = { 1, 4, 6, 4, 1 },
		sum_weights = 16;
	// accumulate the weighted sum
    float aggregator = 0.0;
	for (int i=0; i < 5; i++){
		aggregator += weights[i] * laser_scan.ranges[index + i - 2];
	}
	// divide by the sum of the weights to determine the smoothed value
    laser_scan.ranges[index] = aggregator / sum_weights;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_interpreter"); // name this node
    ros::NodeHandle nh;

	std::string lidar_topic;
	if (nh.getParam("/lidar_interpreter/lidar_topic", lidar_topic)){
		LidarInterpreter lidar_interpreter(nh, lidar_topic);
		ros::spin();
	}
	else {
		ROS_WARN("lidar_interpreter needs the param lidar_topic");
	}
	/*
	// argv[1] should be the name of the topic on which to get the LIDAR data
    if (argc < 2) {
		ROS_INFO("LidarInterpreter needs the name of the LIDAR topic as the first argument");
	}
	else {
		LidarInterpreter lidar_interpreter(nh, argv[1]);
		
		// do nothing; delegate further processing to callbacks
		ros::spin();
	}
	*/
    return 0;
}
