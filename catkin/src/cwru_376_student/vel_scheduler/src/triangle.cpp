#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

double DT = 1.0/50.0;

int main(int argc, char **argv) {
	ros::init(argc, argv, "triangle_vel");
	ros::NodeHandle nh; // get a ros nodehandle
	//create a publisher object that can talk to ROS and issue twist messages on named topic;
	// note: this is customized for stdr robot; would need to change the topic to talk to jinx, etc.

	ros::Publisher vel_cmd_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Publisher vel_cmd_stamped_publisher = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel_stamped", 1);

	// initialize the timer
	ros::Rate rtimer(1.0 / DT);


	// ramp up, immediately ramp down
	double vel = 0.0,
		inc = 0.01,
		direction = 1.0;
	geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands
	geometry_msgs::TwistStamped cmd_vel_stamped;

	cmd_vel.linear.x = 0.0; // initialize these values to zero
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;
	cmd_vel_stamped.twist = cmd_vel;
	while (ros::ok()){
		cmd_vel.linear.x = vel;
		vel_cmd_publisher.publish(cmd_vel);
		cmd_vel_stamped.twist = cmd_vel;
		cmd_vel_stamped.header.stamp = ros::Time::now();
		vel_cmd_stamped_publisher.publish(cmd_vel_stamped);
		// increment or decrement?
		if (vel >= 1.0){ // switch direction of increment
			direction = -1.0;
		}
		else if (vel <= 0.0){
			direction = 1.0;
		}
		vel += direction * inc;	
		rtimer.sleep();	
	}
	return 0;
}   
