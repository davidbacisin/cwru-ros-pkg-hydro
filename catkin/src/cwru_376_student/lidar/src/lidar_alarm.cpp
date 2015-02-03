#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

class LidarAlarm {
private:
	static LidarAlarm *instance;
    // subscribe to the lidar_nearest topic
    ros::Subscriber lidar_nearest_subscriber;
	// publish a Boolean on lidar_alarm
    ros::Publisher lidar_alarm;
	// track the current state of the alarm
	bool isAlarmed;
	// the closest something can be before the alarm is raised (meters)
	float minimum_safe_distance;
public:
	LidarAlarm(ros::NodeHandle& nh);
	// callback for when the lidar_nearest has data
	static void liderNearestCallback(const std_msgs::Float32& nearest);
};

LidarAlarm::LidarAlarm(ros::NodeHandle& nh) {
	// set the singleton instance variable
	instance = this;
	// subscribe to lidar_nearest
	lidar_nearest_subscriber = nh.subscribe("lidar_nearest", 1, liderNearestCallback);
	// broadcast on the lidar_alarm topic
	lidar_alarm = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
	// by default, alarm is off
	isAlarmed = false;
	// set minimum_safe_distance
	minimum_safe_distance = 0.5; // meters
};

LidarAlarm *LidarAlarm::instance;

void LidarAlarm::liderNearestCallback(const std_msgs::Float32& nearest) {
	instance->isAlarmed = (nearest.data < instance->minimum_safe_distance);
	// publish the alarm
	std_msgs::Bool lidar_alarmed_msg;
	lidar_alarmed_msg.data = instance->isAlarmed;
	instance->lidar_alarm.publish(lidar_alarmed_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); // name this node
    ros::NodeHandle nh;
	
	LidarAlarm lidar_alarm(nh);
	
	// do nothing; delegate further processing to callbacks
    ros::spin();
	
    return 0;
}