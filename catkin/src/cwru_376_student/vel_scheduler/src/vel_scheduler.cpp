//intent of this program: modulate the velocity command to comply with a speed limit, v_max,
// acceleration limits, +/-a_max, and come to a halt gracefully at the end of
// an intended line segment

//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <path_planner/path_segment.h> // the service message class for getting the next path segments
#include <math.h>
#include <string>

// Some limits on speed and acceleration. These are loaded from the ROS param server.
double v_max, // 1m/sec is a slow walk
	v_max_original, // lidar will change v_max during execution, so save a backup of our original v_max
	a_max, // 1m/sec^2 is 0.1 g's
	omega_max, // 1 rad/sec about 6 seconds to rotate 1 full rev
	alpha_max; // 0.5 rad/sec^2 takes 2 sec to get from rest to full omega
double DT = 1.0/50.0; // choose an update rate of 50Hz
double radian_to_go_error = 0.1;

// For communication with odometry callbacks
bool odom_initialized = false;
double odom_vel_ = 0.0, // measured/published system speed
	odom_omega_ = 0.0, // measured/published system yaw rate (spin)
	odom_x_= 0.0,
	odom_y_ = 0.0,
	odom_quat_z_ = 0.0,
	odom_quat_w_ = 0.0;
ros::Time t_last_callback_;
double dt_odom_= 0.0; // track the time between odom callbacks

// For communication with lidar, user brake, and estop
bool lidar_initialized = false; // to check if our data is valid
std_msgs::Bool lidar_alarm_msg;
std_msgs::Float32 lidar_nearest;
std_msgs::Bool estop_on;
std_msgs::Bool halt_status;

// Values for controlling how lidar affects velocity
double lidar_safe_distance, // this will be loaded from the ROS param server
	dist_safe,
	dist_danger,
	dist_critical,
	v_safe = 0.1;

ros::Rate *rtimer; // frequency corresponding to chosen sample period DT; the main loop will run this fast

// get info from the ROS param server. Populate most of our global variables.
void getParams(ros::NodeHandle& nh){
	// get the velocity/acceleration max values as specified by ROS params
	// third param of nh.param is the default value
	nh.param("/vel_scheduler/max_linear_velocity", v_max, 1.0);
	v_max_original = v_max;
	nh.param("/vel_scheduler/max_linear_acceleration", a_max, 1.0);
	nh.param("/vel_scheduler/max_angular_velocity", omega_max, 1.0);
	nh.param("/vel_scheduler/max_angular_acceleration", alpha_max, 0.5);
    nh.param("/lidar_alarm/lidar_safe_distance", lidar_safe_distance, 0.5);
	// set safe distances
	dist_safe = lidar_safe_distance + 2.0;
	dist_danger = lidar_safe_distance + 0.25;
	dist_critical = lidar_safe_distance;
}

// receive the pose and velocity estimates from the simulator or the physical robot
// copy the relevant values to global variables
// Note: stdr updates odom only at 10Hz; Jinx is ~50Hz
void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    // compute the delta-time between successive callbacks:
    dt_odom_ = (ros::Time::now() - t_last_callback_).toSec();
    t_last_callback_ = ros::Time::now(); // let's remember the current time, and use it next iteration

	// clamp the max value of dt_odom_ for computations elsewhere
    if (dt_odom_ > 0.15) { // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
        dt_odom_ = 0.1;
        ROS_WARN("large dt; dt = %lf", dt_odom_); // let's complain whenever this happens
    }
    
    // copy some of the components of the received message into global vars, for use by "main()"
    // we care about speed and spin, as well as position estimates x,y and heading
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_z_ = odom_rcvd.pose.pose.orientation.z;
    odom_quat_w_ = odom_rcvd.pose.pose.orientation.w;

	// we have data, so set to true
	odom_initialized = true;

	// debug output
    ROS_INFO("odom CB: x = %f, y= %f, quat_z = %f, quat_w = %f, v = %f, omega = %f", odom_x_, odom_y_, odom_quat_z_, odom_quat_w_, odom_vel_, odom_omega_);
}

// callback to check for a user brake
void haltCallback(const std_msgs::Bool& h_rcvd){    
    // the output below could get annoying; may comment this out, but useful initially for debugging
    ROS_INFO("halt status: %i", h_rcvd.data);
    // Update the global variable with the halt status
    halt_status.data = h_rcvd.data;
}

// receive the info from lidar alarm
void lidarAlarmCallback(const std_msgs::Bool& la_rcvd){    
    ROS_INFO("received lidar alarm value is: %i", la_rcvd.data);
    // post the received data in a global var for access by main prog
    lidar_alarm_msg.data = la_rcvd.data;
}

// receive the info from lidar nearest
void lidarNearestCallback(const std_msgs::Float32& ln_rcvd){    
    // check for data on topic "lidar_dist" 
    ROS_INFO("received lidar nearest value is: %f", ln_rcvd.data);
    // post the received data in a global var for access by main prog
    lidar_nearest.data = ln_rcvd.data;
    lidar_initialized = true;
}

// receive the info on e-stop status
void eStopStatusCallback(const std_msgs::Bool& ess_rcvd){  
    // check for data on topic ""lidar_alarm"" 
    ROS_INFO("received estop status value is: %i", ess_rcvd.data);
    // post the received data in a global var for access by main prog
    estop_on.data = !ess_rcvd.data; // boolean inverse just so that estop makes more intuitive sense
}

// calculate the percentage of our max speed that we _want_ to go, based off of the distance remaining
double getRampingFactor(double remaining, double vel, double acc){
	double ramping_factor = 0.0,
		time_to_decel = vel/acc, // basic physics
		dist_decel = 0.5 * acc * (time_to_decel * time_to_decel);
	if (remaining <= 0.0) { // at goal, or overshot; stop!
		ramping_factor = 0.0;
	}
	else if (halt_status.data == true) { // if user brake, then stop!
		ramping_factor = 0.0;
	}
	else if (remaining <= dist_decel) { // possibly should be braking to a halt
		// dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
		// so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
		// and factor = sqrt(2*dist*a)/v_max
		if (vel == 0.0) { // avoid dividing by zero
			ramping_factor = 0.0;
		}
		else {
			ramping_factor = sqrt(2 * remaining * acc) / vel;
		}
		ROS_INFO("braking zone: ramping_factor = %f", ramping_factor);
	}
	else { // otherwise, go full speed
		ramping_factor = 1.0;
	}
	return ramping_factor;
}

// calculate the truly desired velocity based off the current velocity and ramping factor
double getVelocity(double current_vel, double ramping_factor, double vel, double acc){
	double ramped_vel = ramping_factor * vel,
		new_vel = ramped_vel;
    ROS_INFO("ramped_vel: %f, current_vel: %f", ramped_vel, current_vel);
	// how does the current velocity compare to the desired (ramped) vel?
	if (current_vel < ramped_vel) {  // maybe we halted, e.g. due to estop or obstacle;
		// may need to ramp up to v_max; do so within accel limits
		double v_test = current_vel + acc*(1.0 - current_vel/ramped_vel); // reduce acceleration as we get closer to our goal
		new_vel = (v_test < ramped_vel) ? v_test : ramped_vel; // don't overshoot ramped_vel
		
		// debug
		ROS_INFO("v_test: %f, acc: %f, dt_odom_: %f", v_test, acc, dt_odom_);
	}
    else if (current_vel > ramped_vel) { // travelling too fast. Ramp down.
		double v_test = current_vel - 1.2 * acc; // moving too fast--try decelerating faster than nominal acc
		new_vel = (v_test > ramped_vel) ? v_test : ramped_vel; // don't overshoot ramped_vel
		
		// debug
		ROS_INFO("odom vel: %f; sched vel: %f", odom_vel_, ramped_vel);
	} else {
		new_vel = ramped_vel; // silly third case: this is already true, if here. Issue the desired velocity
	}
    // ensure there is no emergency stop!
    if (lidar_alarm_msg.data == true || estop_on.data == true) { // The robot should stop when either condition is true
        new_vel = 0.0;
		// debug
        ROS_INFO("Halted the robot. User brake status = %i; Lidar alarm = %i; Estop = %i", halt_status.data, lidar_alarm_msg.data, estop_on.data);
    }
	return new_vel;
}

// move the robot linearly
void translationFunc (ros::Publisher& vel_cmd_publisher, ros::Publisher& vel_cmd_stamped_pub, double segment_length){     
	double start_x = odom_x_; // save our starting position so we can calculate how far we've gone
    double start_y = odom_y_;
    ROS_INFO("start pose: x %f, y= %f", start_x, start_y);

    geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands
    geometry_msgs::TwistStamped cmd_vel_stamped; // and a twist with timestamp

    cmd_vel.linear.x = 0.0; // initialize these values to zero
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
	
	double dist_to_go = segment_length, // track how much further we need to go
		new_cmd_vel = 0.0; // value of speed to be commanded; update each iteration

    while (ros::ok() && dist_to_go > 0.0) { // stop when ROS says so or when we've finished the segment
		// allow callbacks to populate fresh data
        ros::spinOnce();
        // compute distance travelled so far:
        double delta_x = odom_x_ - start_x;
        double delta_y = odom_y_ - start_y;
        double segment_length_done = sqrt(delta_x * delta_x + delta_y * delta_y);
        ROS_INFO("dist travelled: %f", segment_length_done);
		// calculate how much further we need to go
        dist_to_go = segment_length - segment_length_done;
		
		// get our velocity ramping factor
		double ramping_factor = getRampingFactor(dist_to_go, v_max, a_max);
        
		// check the lidar to see if we should slow down or stop
		if (lidar_initialized){ // make sure we've gotten data from the lidar so that lidar_nearest will be initialized
			double dist_to_stop = lidar_nearest.data;

			// debug
			ROS_INFO("Lidar nearest: %f", lidar_nearest.data);

			if (dist_to_stop <= dist_critical){ // stop in the critical zone
				v_max = 0.0;
				ROS_INFO("Lidar critical zone: v_max = %f", v_max);
			}
			else if (dist_to_stop <= dist_danger){ // go our safe speed in the danger zone
				v_max = v_safe;
				ROS_INFO("Lidar danger zone: v_max = %f", v_max);
			}
			else if (dist_to_stop <= dist_safe) { // in the safe zone, speed is proportional to how far away the object is
				v_max = v_max_original * (dist_to_stop / dist_safe);
				ROS_INFO("Lidar safe zone: v_max = %f", v_max);
			}
			else { // go as fast as possible otherwise
				v_max = v_max_original;
				ROS_INFO("Lidar carefree zone: v_max = %f", v_max);
			}
		}

		// now get the velocity
		new_cmd_vel = getVelocity(odom_vel_, ramping_factor, v_max, a_max);
		
		// prevent robot from going backwards
		if (new_cmd_vel < 0.0) {
			new_cmd_vel = 0.0;
		}
		
		// ensure once more that we haven't reached our destination
		if (dist_to_go <= 0.0) {
			new_cmd_vel = 0.0;
		}
    
        cmd_vel.linear.x = new_cmd_vel;
        cmd_vel.angular.z = 0.0; // spin command; always zero when we're moving linearly
		// publish the command
        vel_cmd_publisher.publish(cmd_vel);
		// also publish a time-stamped command
        cmd_vel_stamped.twist = cmd_vel;
        cmd_vel_stamped.header.stamp = ros::Time::now();
        vel_cmd_stamped_pub.publish(cmd_vel_stamped);
        rtimer->sleep(); // sleep for remainder of timed iteration
    }    
}

// turn the robot
void rotationFunc (ros::Publisher& vel_cmd_publisher, ros::Publisher& vel_cmd_stamped_pub, double segment_radian){
    double start_x = odom_x_, // save our starting position so we can calculate when we're done
		start_y = odom_y_,
		start_quat_z = odom_quat_z_,
		start_quat_w = odom_quat_w_;
	ROS_INFO("start pose: x %f, y= %f, phi = %f", start_x, start_y, 2.0 * atan2(start_quat_w, start_quat_z));

    geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands
    geometry_msgs::TwistStamped cmd_vel_stamped; // and a twist with timestamp

    cmd_vel.linear.x = 0.0; // initialize these values to zero
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    double radian_to_go = segment_radian, // track how much further we need to go
		new_cmd_omega = 0.0; // value of spin to be commanded; update each iteration

    while (ros::ok() && radian_to_go > radian_to_go_error) { // terminate if ROS has faulted or if we've finished the segment
        // allow callbacks to populate fresh data
		ros::spinOnce();
		// compute the angle between the start and current orientation
		// formula from http://math.stackexchange.com/questions/90081/quaternion-distance
        double segment_radian_done = acos(2.0*pow(odom_quat_z_ * start_quat_z + odom_quat_w_ * start_quat_w, 2)-1.0);
        // ROS_INFO("dist travelled: %f",segment_radian_done);
        radian_to_go = fabs(segment_radian) - fabs(segment_radian_done);
		// are we close enough to zero that we should assume zero?
		if (radian_to_go <= radian_to_go_error) {
			radian_to_go = 0.0;
		}
        
		// get the velocity ramping factor
        double ramping_factor = getRampingFactor(radian_to_go, omega_max, alpha_max);

		// getVelocity is well-behaved only with positive numbers
		odom_omega_ = fabs(odom_omega_);
        
		// get the velocity
		new_cmd_omega = getVelocity(odom_omega_, ramping_factor, omega_max, alpha_max);

		// if our segment is negative, flip the sign on the velocity
		if (segment_radian < 0.0) {
			new_cmd_omega = -new_cmd_omega;
        }
		
		// ensure once more that we haven't reached our destination
		if (radian_to_go <= radian_to_go_error) {
			new_cmd_omega = 0.0;
		}
    
        cmd_vel.linear.x = 0.0; // linear command; always zero when rotating
        cmd_vel.angular.z = new_cmd_omega;
		// publish the command
        vel_cmd_publisher.publish(cmd_vel); 
		// publish the time-stamped command
        cmd_vel_stamped.twist = cmd_vel;
        cmd_vel_stamped.header.stamp = ros::Time::now();
        vel_cmd_stamped_pub.publish(cmd_vel_stamped);
        rtimer->sleep(); // sleep for remainder of timed iteration
    }
}

// gets the path segment from the service client
// ampersands in the arguments allow us to return those values to the calling scope
void getSegment(ros::ServiceClient& client, int segment_ID, double& segment_radian, double& segment_length) {  //function to determine what value to load for move instructions
    // instantiate an object of a consistent type for requests and responses with:
    path_planner::path_segment srv;
    srv.request.id = segment_ID; //requests the next segment by its segment id in the server
    
    if (client.call(srv)) { //check to see if that segment id exists
		segment_radian = srv.response.heading; //load distance to rotate
		segment_length = srv.response.distance; //load distance to move
    } else {
		segment_length = 0.0; //if segment does not exist do not move
		segment_radian = 0.0;
    }

	ROS_INFO("Recieved segment #%i with heading %f and length %f", segment_ID, segment_radian, segment_length);
}

// main function!
int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_scheduler"); // name of this node will be "vel_scheduler"
    ros::NodeHandle nh; // get a ros nodehandle

	// load some topic names from the ROS param server
	// these topics differ from the simulator vs actual robot
	std::string cmd_vel_topic,
		cmd_vel_stamped_topic,
		odom_topic;
	if (!nh.getParam("/vel_scheduler/cmd_vel_topic", cmd_vel_topic)){
		ROS_WARN("vel_scheduler needs ROS param cmd_vel_topic");
		return 0;
	}
	if (!nh.getParam("/vel_scheduler/cmd_vel_stamped_topic", cmd_vel_stamped_topic)){
		ROS_WARN("vel_scheduler needs ROS param cmd_vel_stamped_topic");
		return 0;
	}
	if (!nh.getParam("/vel_scheduler/odom_topic", odom_topic)){
		ROS_WARN("vel_scheduler needs a ROS param odom_topic");
		return 0;
	}
	// create the publishers and subscribers
    ros::Publisher vel_cmd_publisher = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    ros::Publisher vel_cmd_stamped_pub = nh.advertise<geometry_msgs::TwistStamped>(cmd_vel_stamped_topic, 1);
    ros::Subscriber sub_odom = nh.subscribe(odom_topic, 1, odomCallback);
    ros::Subscriber sub_halt = nh.subscribe("user_brake", 1, haltCallback);
    ros::Subscriber sub_lidar_alarm = nh.subscribe("lidar_alarm", 1, lidarAlarmCallback);
    ros::Subscriber sub_lidar_nearest = nh.subscribe("lidar_nearest", 1, lidarNearestCallback);
    ros::Subscriber sub_estop_status = nh.subscribe("estop_status", 1, eStopStatusCallback);
 
    // initialize the timer
    rtimer = new ros::Rate(1.0 / DT);

    //creates a ROS “ServiceClient”. This service client expects to communicate requests and responses 
    //as defined in: path_planne::path_segment. Also, this service client expects to communicate with 
    //a named service, called "path_planner_service". 
    //this is the service name that David had defined inside of path_planner node.
    ros::ServiceClient client = nh.serviceClient<path_planner::path_segment>("path_planner_service"); //initializes service client that is responsible for acquiring the next move instruction
    
	// load v_max, a_max, etc from the param server
	getParams(nh);
	
	ROS_INFO("Waiting for odom data");
	// wait until odom is ready
	while (!odom_initialized){
		ros::spinOnce();
		rtimer->sleep();
	}

    // Wait until path_planner is ready to send data to us
    path_planner::path_segment srv;
    srv.request.id = 0;
	while (!client.call(srv)){
		rtimer->sleep();
	}

	// the number of segments; probably will load this from path_planner in the future
    int segment_tot = 5;
    for (int segment_ID = 0; ros::ok() && segment_ID < segment_tot; segment_ID++) {
		double segment_radian = 0.0;
		double segment_length = 0.0;
		// load the segment
		getSegment(client, segment_ID, segment_radian, segment_length);
		// should we turn or go straight?
        if (fabs(segment_radian) > segment_length){
            rotationFunc(vel_cmd_publisher, vel_cmd_stamped_pub, segment_radian);//call rotationFunc           
        } else {
            translationFunc(vel_cmd_publisher, vel_cmd_stamped_pub, segment_length);//call translationFunc    
        }
		// wait a half second between path segments
		for (int wait = 0; wait < 0.5/DT; wait++){
			rtimer->sleep();
		}
    } 
    ROS_INFO("completed move distance");

	// clean up the dynamic memory
    delete rtimer;

    return 0;
}            
