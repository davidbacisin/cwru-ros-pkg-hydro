#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;

class LidarInterpreter {
private:
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
	// Returns 0 if no clamping, 1 if clamped on the far end, or -1 if clamped on the near end.
	static int clampPing(int index, const sensor_msgs::LaserScan& laser_scan);
	// utility method for reducing noise in the LIDAR range data
	static void smoothPing(int index, const sensor_msgs::LaserScan& laser_scan);
	
public:
	LidarInterpreter(ros::NodeHandle& nh);
	// callback for when the LIDAR has data
	static void laserCallback(const sensor_msgs::LaserScan& laser_scan);
};

LidarInterpreter::LidarInterpreter(ros::NodeHandle& nh) {
	// subscribe to the LIDAR
	lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
	// broadcast on the lidar_nearest topic
	lidar_nearest = nh.advertise<std_msgs::Float32>("lidar_nearest", 1);
	// initialize the LIDAR range values
	angle_min = 0.0;
	angle_max = 0.0;
	angle_increment = -1.0;
	range_min = 0.0;
	range_max = 0.0;
}

static void LidarInterpreter::laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	// have we initialized the LIDAR range data yet?
	if (angle_increment <= 0.0) {
		angle_min = laser_scan.angle_min;
        angle_max = laser_scan.angle_max;
        angle_increment = laser_scan.angle_increment;
        range_min = laser_scan.range_min;
        range_max = laser_scan.range_max;
	}
	
	// filter the ping data
	// clamped pings should not be considered for nearest object
	bool ping_was_clamped = false;
	// keep updated with the nearest object
	float nearest_object = range_max;
	// start with ping 2 and end with the third-to-last ping so that we don't cause a segmentation fault in smoothPing()
	for (int i=2; i+2 < sizeof(laser_scan.ranges); i++){
		// clamp the ping
		ping_was_clamped = (clampPing(i, laser_scan) != 0);
		// smooth out noise
		smoothPing(i, laser_scan);
		// see if the ping had the shortest distance
		if (!ping_was_clamped &&
			laser_scan.ranges[i] < nearest_object){
			nearest_object = laser_scan.ranges[i];
		}
	}
	
	// publish the distance of the nearest object
	std_msgs::Float32 lidar_dist_msg;
	lidar_dist_msg.data = nearest_object;
	lidar_nearest.publish(lidar_dist_msg);
}

int LidarInterpreter::clampPing(int index, const sensor_msgs::LaserScan& laser_scan) {
	if (laser_scan.ranges[index] <= range_min) {
		laser_scan.ranges[index] = range_min;
		return -1;
	}
	else if (laser_scan.ranges[index] >= range_max) {
		laser_scan.ranges[index] = range_max;
		return 1;
	}
	else {
		return 0;
	}
}

void LidarInterpreter::smoothPing(int index, const sensor_msgs::LaserScan& laser_scan) {
	// Perform a weighted average of the ping values
	int weights[5] = [ 1, 4, 6, 4, 1 ],
		sum_weights = 16;
	// accumulate the weighted sum
	float aggregator = 0.0;
	for (int i=0; i < 5; i++){
		aggregator += weights[i] * laser_scan.ranges[index + i - 2];
	}
	// divide by the sum of the weights to determine the smoothed value
	laser_scan.ranges[index] = aggregator / sum_weights;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_interpreter"); // name this node
    ros::NodeHandle nh;
	
	LidarInterpreter lidar_interpreter(nh);
	
	// do nothing; delegate further processing to callbacks
    ros::spin();
	
    return 0;
}