//intent of this program: modulate the velocity command to comply with a speed limit, v_max,
// acceleration limits, +/-a_max, and come to a halt gracefully at the end of
// an intended line segment

// notes on quaternions:
/*
From:
http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
q is a quaternion which represents the rotation, if you prefer to think in terms of the angle and axis of the rotation then q is:
q = cos(a/2) + i ( x * sin(a/2)) + j (y * sin(a/2)) + k ( z * sin(a/2))

where:

a = rotation angle
x,y,z = rotation axis


qx = ax * sin(angle/2)
qy = ay * sin(angle/2)
qz = az * sin(angle/2)
qw = cos(angle/2)


so, quaternion in 2-D plane (x,y,theta):
ax=0, ay=0, az = 1.0

qx = 0;
qy = 0;
qz = sin(angle/2)
qw = cos(angle/2)

therefore, theta = 2*atan2(qz,qw)
*/

//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message time
#include <path_planner/path_segment.h> // the service message class
#include <math.h>
#include <string>

// set some dynamic limits...
double v_max = 0.9; //1m/sec is a slow walk
double v_min = 0.1; // if command velocity too low, robot won't move
double a_max = 0.3; //1m/sec^2 is 0.1 g's
//const double a_max_decel = 0.1; // TEST
double omega_max = 0.5; //1 rad/sec-> about 6 seconds to rotate 1 full rev
double alpha_max = 0.7; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega
double DT = 1.0/50.0; // choose an update rate of 50Hz
double radian_to_go_error = 0.1;

// globals for communication w/ callbacks:
bool odom_initialized = false;
double odom_vel_ = 0.0; // measured/published system speed
double odom_omega_ = 0.0; // measured/published system yaw rate (spin)
double odom_x_=  0.0;
double odom_y_ = 0.0;
double odom_phi_ = 0.0;
double odom_quat_z_ = 0.0;
double odom_quat_w_ = 0.0;
double dt_odom_ = 0.0;
ros::Time t_last_callback_;
double dt_callback_= 0.0;
std_msgs::Bool lidar_alarm_msg;
double lidar_safe_distance;
std_msgs::Float32 lidar_nearest;
bool lidar_initialized = false;
std_msgs::Bool estop_on;
std_msgs::Bool halt_status;

ros::Rate *rtimer; // frequency corresponding to chosen sample period DT; the main loop will run this fast

void getParams(ros::NodeHandle& nh){
	// get the velocity/acceleration max values as specified by ROS params
	// third param is the default value
	nh.param("/vel_scheduler/max_linear_velocity", v_max, 1.0);
	nh.param("/vel_scheduler/max_linear_acceleration", a_max, 1.0);
	nh.param("/vel_scheduler/max_angular_velocity", omega_max, 1.0);
	nh.param("/vel_scheduler/max_angular_acceleration", alpha_max, 0.5);
    nh.param("/lidar_alarm/lidar_safe_distance", lidar_safe_distance, 0.5);
}

// receive odom messages and strip off the components we want to use
// tested this OK w/ stdr

// receive the pose and velocity estimates from the simulator (or the physical robot)
// copy the relevant values to global variables, for use by "main"
// Note: stdr updates odom only at 10Hz; Jinx is 50Hz (?)
void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    //here's a trick to compute the delta-time between successive callbacks:
    dt_callback_ = (ros::Time::now() - t_last_callback_).toSec();
    //ROS_INFO("dt_callback_: %f", dt_callback_);
    t_last_callback_ = ros::Time::now(); // let's remember the current time, and use it next iteration

    if (dt_callback_ > 0.15) { // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
        dt_callback_ = 0.1; // can choose to clamp a max value on this, if dt_callback is used for computations elsewhere
        ROS_WARN("large dt; dt = %lf", dt_callback_); // let's complain whenever this happens
    }
    
    // copy some of the components of the received message into global vars, for use by "main()"
    // we care about speed and spin, as well as position estimates x,y and heading
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    // see notes above for conversion for simple planar motion
    double quat_z = odom_rcvd.pose.pose.orientation.z;
    double quat_w = odom_rcvd.pose.pose.orientation.w;
    odom_phi_ = 2.0*atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    odom_quat_z_ = quat_z;
    odom_quat_w_ = quat_w;

	odom_initialized = true;

    // the output below could get annoying; may comment this out, but useful initially for debugging
    ROS_INFO("odom CB: x = %f, y= %f, phi = %f, v = %f, omega = %f", odom_x_, odom_y_, odom_phi_, odom_vel_, odom_omega_);
}

// receive the linear and angular and velocity estimates from the simulator (or the physical robot)
// copy the relevant values to global variables, for use by "main"
void haltCallback(const std_msgs::Bool& h_rcvd){    
    // the output below could get annoying; may comment this out, but useful initially for debugging
    ROS_INFO("halt status: %i", h_rcvd.data);
    // Update the global variable with the halt status
    halt_status.data = h_rcvd.data;
}

// receive the infor from lidar alarm
// copy the relevant values to global variables, for use by "main"
void lidarAlarmCallback(const std_msgs::Bool& la_rcvd){    
    // check for data on topic ""lidar_alarm"" 
    ROS_INFO("received lidar alarm value is: %i", la_rcvd.data);
    // post the received data in a global var for access by main prog
    lidar_alarm_msg.data = la_rcvd.data;
}

// receive the infor from lidar dist
// copy the relevant values to global variables, for use by "main"
void lidarNearestCallback(const std_msgs::Float32& ld_rcvd){    
    // check for data on topic "lidar_dist" 
    ROS_INFO("received lidar dist value is: %f", ld_rcvd.data);
    // post the received data in a global var for access by main prog
    lidar_initialized = true;
    lidar_nearest.data = ld_rcvd.data;
}

// receive the infor e-stop status
// copy the relevant values to global variables, for use by "main"
void eStopStatusCallback(const std_msgs::Bool& ess_rcvd){  
    // check for data on topic ""lidar_alarm"" 
    ROS_INFO("received estop status value is: %i", ess_rcvd.data);
    // post the received data in a global var for access by main prog
    estop_on.data = !ess_rcvd.data;
}

double getRampingFactor(double remaining, double vel, double acc){
	double ramping_factor = 0.0,
		time_to_decel = vel/acc,
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
		ramping_factor = sqrt(2 * remaining * acc) / vel;
		ROS_INFO("braking zone: ramping_factor = %f", ramping_factor);
	}
	else if (lidar_initialized && // make sure we've gotten data from the lidar so that lidar_nearest will be initialized
			 lidar_nearest.data <= lidar_safe_distance + dist_decel) { // we might get the lidar alarm soon, so start slowing
		double dist_to_stop = lidar_nearest.data - lidar_safe_distance;
		if (dist_to_stop < 0.0) dist_to_stop = 0.0;
		ramping_factor = sqrt(2 * dist_to_stop * acc) / vel;
		//ramping_factor = 0.0;
		ROS_INFO("lidar caution zone: ramping_factor = %f", ramping_factor);
	}
	else { // not ready to decel, so target vel is v_max, either accel to it or hold it
		ramping_factor = 1.0;
	}
	return ramping_factor;
}

double getVelocity(double current_vel, double ramping_factor, double vel, double acc){
	double ramped_vel = ramping_factor * vel,
		new_vel = vel;
    ROS_INFO("ramped_vel: %f, current_vel: %f", ramped_vel, current_vel);
	// how does the current velocity compare to the scheduled vel?
	if (current_vel < ramped_vel) {  // maybe we halted, e.g. due to estop or obstacle;
		// may need to ramp up to v_max; do so within accel limits
		double v_test = current_vel + acc*(1.0 - current_vel/ramped_vel); // if callbacks are slow, this could be abrupt
		ROS_INFO("v_test: %f, acc: %f, dt_callback_: %f", v_test, acc, dt_callback_);
		// operator:  c = (a>b) ? a : b;
		new_vel = (v_test < ramped_vel) ? v_test : ramped_vel; //choose lesser of two options
		// this prevents overshooting ramped_vel
	}
    else if (current_vel > ramped_vel) { //travelling too fast--this could be trouble
		// ramp down to the scheduled velocity.  However, scheduled velocity might already be ramping down at acc.
		// need to catch up, so ramp down even faster than acc.  Try 1.2*acc.
		ROS_INFO("odom vel: %f; sched vel: %f", odom_vel_, ramped_vel); //debug/analysis output; can comment this out
		
		double v_test = current_vel - 1.2 * acc/**dt_callback_*/; //moving too fast--try decelerating faster than nominal acc
		
		new_vel = (v_test > ramped_vel) ? v_test : ramped_vel; // choose larger of two options...don't overshoot ramped_vel
	} else {
		new_vel = ramped_vel; //silly third case: this is already true, if here.  Issue the scheduled velocity
	}
    // ensure there is no emergency stop!
    if (lidar_alarm_msg.data == true || estop_on.data == true) { // The robot should stop when either condition is true
        new_vel = 0.0;
        ROS_INFO("Halted the robot. Halt status = %i; Lidar alarm = %i; Estop = %i", halt_status.data, lidar_alarm_msg.data, estop_on.data);
    } 
	return new_vel;
}

void translationFunc (ros::Publisher& vel_cmd_publisher, ros::Publisher& vel_cmd_stamped_pub, double segment_length){
    // here is a crude description of one segment of a journey.  Will want to generalize this to handle multiple segments
    // define the desired path length of this segment, segment_1, linear advancing
    //double segment_length = 100; // desired travel distance in meters; anticipate travelling multiple segments
    
    double segment_length_done = 0.0; // need to compute actual distance travelled within the current segment
    
	double start_x = 0.0; // fill these in with actual values once odom message is received
    double start_y = 0.0; // subsequent segment start coordinates should be specified relative to end of previous segment
    double start_phi = 0.0;

    double scheduled_vel = 0.0; //desired vel, assuming all is per plan
    double new_cmd_vel = 0.0; // value of speed to be commanded; update each iteration
    double new_cmd_omega = 0.0; // update spin rate command as well

    geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands
    geometry_msgs::TwistStamped cmd_vel_stamped;

    cmd_vel.linear.x = 0.0; // initialize these values to zero
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    // let's wait for odom callback to start getting good values...
/*    odom_omega_ = 1000000; // absurdly high
    ROS_INFO("waiting for valid odom callback...");
    t_last_callback_ = ros::Time::now(); // initialize reference for computed update rate of callback
    while (odom_omega_ > 1000) {
        rtimer->sleep();
        ros::spinOnce();
    }
    ROS_INFO("received odom message; proceeding");
*/
    start_x = odom_x_;
    start_y = odom_y_;
    start_phi = odom_phi_;
    ROS_INFO("start pose: x %f, y= %f, phi = %f", start_x, start_y, start_phi);

    // compute some properties of trapezoidal velocity profile plan:
    double T_accel = v_max / a_max; //...assumes start from rest
    double T_decel = v_max / a_max; //(for same decel as accel); assumes brake to full halt
    double dist_accel = 0.5 * a_max * (T_accel * T_accel); //distance rqd to ramp up to full speed
    double dist_decel = 0.5 * a_max * (T_decel * T_decel); //same as ramp-up distance
    double dist_const_v = segment_length - dist_accel - dist_decel; //if this is <0, never get to full spd
    double T_const_v = dist_const_v / v_max; //will be <0 if don't get to full speed
    double T_segment_tot = T_accel + T_decel + T_const_v; // expected duration of this move

    while (ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted (or ctl-C)
    {
        ros::spinOnce(); // allow callbacks to populate fresh data
        // compute distance travelled so far:
        double delta_x = odom_x_ - start_x;
        double delta_y = odom_y_ - start_y;
        double segment_length_done = sqrt(delta_x * delta_x + delta_y * delta_y);
        ROS_INFO("dist travelled: %f", segment_length_done);
        double dist_to_go = segment_length - segment_length_done;
                    
        ROS_INFO("Lidar nearest: %f", lidar_nearest.data);
        
        //use segment_length_done to decide what vel should be, as per plan
        /* if (dist_to_go <= 0.0) { // at goal, or overshot; stop!
            scheduled_vel = 0.0;
        }
        else if (halt_status.data == true) { // if user brake, then stop!
            scheduled_vel = 0.0;
        }
        else if (dist_to_go <= dist_decel) { // possibly should be braking to a halt
            // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
            // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
            scheduled_vel = sqrt(2 * dist_to_go * a_max);
            ROS_INFO("braking zone: scheduled_vel = %f", scheduled_vel);
        }
        else if (lidar_initialized && lidar_nearest.data <= 1.0) { // we might get the lidar alarm soon, so start slowing. First make sure we've gotten data from the lidar so that lidar_nearest will be initialized
            scheduled_vel = 0.0;
            ROS_INFO("lidar caution zone: scheduled_vel = %f",scheduled_vel);
        }
        else { // not ready to decel, so target vel is v_max, either accel to it or hold it
            scheduled_vel = v_max;
        }

        //how does the current velocity compare to the scheduled vel?
        if (odom_vel_ < scheduled_vel) {  // maybe we halted, e.g. due to estop or obstacle;
            // may need to ramp up to v_max; do so within accel limits
            double v_test = odom_vel_ + a_max*dt_callback_; // if callbacks are slow, this could be abrupt
	    ROS_INFO("v_test: %f, a_max: %f, dt_callback_: %f", v_test, a_max, dt_callback_);
            // operator:  c = (a>b) ? a : b;
            new_cmd_vel = (v_test < scheduled_vel) ? v_test : scheduled_vel; //choose lesser of two options
            // this prevents overshooting scheduled_vel
        } else if (odom_vel_ > scheduled_vel) { //travelling too fast--this could be trouble
            // ramp down to the scheduled velocity.  However, scheduled velocity might already be ramping down at a_max.
            // need to catch up, so ramp down even faster than a_max.  Try 1.2*a_max.
            ROS_INFO("odom vel: %f; sched vel: %f", odom_vel_, scheduled_vel); //debug/analysis output; can comment this out
            
            double v_test = odom_vel_ - 1.2 * a_max*dt_callback_; //moving too fast--try decelerating faster than nominal a_max
            
            new_cmd_vel = (v_test > scheduled_vel) ? v_test : scheduled_vel; // choose larger of two options...don't overshoot scheduled_vel
        } else {
            new_cmd_vel = scheduled_vel; //silly third case: this is already true, if here.  Issue the scheduled velocity
        }
        ROS_INFO("scheduled_vel: %f, odom_vel_: %f, new_cmd_vel: %f", scheduled_vel, odom_vel_, new_cmd_vel);
    
        if (lidar_alarm_msg.data == true || estop_on.data == true) { // The robot should stop when either condition is true
            new_cmd_vel = 0.0;
            ROS_INFO("Halted the robot. Halt status = %i; Lidar alarm = %i; Estop = %i", halt_status.data, lidar_alarm_msg.data, estop_on.data);
        }
		*/
		
		double ramping_factor = getRampingFactor(dist_to_go, v_max, a_max);
		new_cmd_vel = getVelocity(odom_vel_, ramping_factor, v_max, a_max);
		
		// prevent robot from going backwards
		if (new_cmd_vel < 0.0) {
			new_cmd_vel = 0.0;
		}
    
        cmd_vel.linear.x = new_cmd_vel;
        cmd_vel.angular.z = 0.0; // spin command; always zero, in this example
        if (dist_to_go <= 0.0) { // if any of these conditions is true, the robot should stop
            cmd_vel.linear.x = 0.0;
        }
        vel_cmd_publisher.publish(cmd_vel); // publish the command
        cmd_vel_stamped.twist = cmd_vel;
        cmd_vel_stamped.header.stamp = ros::Time::now();
        vel_cmd_stamped_pub.publish(cmd_vel_stamped);
        rtimer->sleep(); // sleep for remainder of timed iteration
        if (dist_to_go <= 0.0) break; // halt this node when this segment is complete.
        // will want to generalize this to handle multiple segments
        // ideally, will want to receive segments dynamically as publications from a higher-level planner
    }    
}
void rotationFunc (ros::Publisher& vel_cmd_publisher, ros::Publisher& vel_cmd_stamped_pub, double segment_radian){
    double segment_radian_done = 0.0; // need to compute actual distance travelled within the current segment
    double start_x = 0.0; // fill these in with actual values once odom message is received
    double start_y = 0.0; // subsequent segment start coordinates should be specified relative to end of previous segment
    
    double start_phi = 0.0;
    double start_quat_z = 0.0,
	   start_quat_w = 0.0;

    double scheduled_vel = 0.0; //desired vel, assuming all is per plan
    double new_cmd_vel = 0.0; // value of speed to be commanded; update each iteration
    double new_cmd_omega = 0.0; // update spin rate command as well

    geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands
    geometry_msgs::TwistStamped cmd_vel_stamped;

    cmd_vel.linear.x = 0.0; // initialize these values to zero
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    // let's wait for odom callback to start getting good values...
/*    odom_omega_ = 1000000; // absurdly high
    ROS_INFO("waiting for valid odom callback...");
    t_last_callback_ = ros::Time::now(); // initialize reference for computed update rate of callback
    while (odom_omega_ > 1000) {
        rtimer->sleep();
        ros::spinOnce();
    }
    ROS_INFO("received odom message; proceeding");
*/
    start_x = odom_x_;
    start_y = odom_y_;
    start_phi = odom_phi_;
    start_quat_z = odom_quat_z_;
    start_quat_w = odom_quat_w_;
    ROS_INFO("start pose: x %f, y= %f, phi = %f", start_x, start_y, start_phi);

    // compute some properties of trapezoidal velocity profile plan:
    double T_accel = omega_max / alpha_max; //...assumes start from rest
    double T_decel = omega_max / alpha_max; //(for same decel as accel); assumes brake to full halt
    double dist_accel = 0.5 * alpha_max * (T_accel * T_accel); //distance rqd to ramp up to full speed
    double dist_decel = 0.5 * alpha_max * (T_decel * T_decel); //same as ramp-up distance
    double dist_const_v = abs(segment_radian) - dist_accel - dist_decel; //if this is <0, never get to full spd
    double T_const_v = dist_const_v / omega_max; //will be <0 if don't get to full speed
    double T_segment_tot = T_accel + T_decel + T_const_v; // expected duration of this move

    //dist_decel*= 2.0; // TEST TEST TEST
    while (ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted (or ctl-C)
    {
        ros::spinOnce(); // allow callbacks to populate fresh data
        // compute distance travelled so far:

		// compute the angle between the start and current orientation
		// formula from http://math.stackexchange.com/questions/90081/quaternion-distance
        double segment_radian_done = acos(2.0*pow(odom_quat_z_ * start_quat_z + odom_quat_w_ * start_quat_w, 2)-1.0);
        // ROS_INFO("dist travelled: %f",segment_radian_done);
        double radian_to_go = fabs(segment_radian) - fabs(segment_radian_done);
		if (radian_to_go <= radian_to_go_error) {
			radian_to_go = 0.0;
		}
        //ROS_INFO("Lidar nearest: %f", lidar_nearest.data);
        //use segment_length_done to decide what vel should be, as per plan
        /*if (radian_to_go <= 0.05) { // at goal, or overshot; stop!
            scheduled_vel = 0.0;
        }
        else if (halt_status.data == true) { // if user brake, then stop!
            scheduled_vel = 0.0;
        }
        else if (radian_to_go <= dist_decel) { // possibly should be braking to a halt
            // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
            // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
            scheduled_vel = sqrt(2 * radian_to_go * alpha_max);
            ROS_INFO("braking zone: scheduled_vel = %f",scheduled_vel);
        }
        else if (lidar_initialized && lidar_nearest.data <= 1.0) { // we might get the lidar alarm soon, so start slowing. First make sure we've gotten data from the lidar so that lidar_nearest will be initialized
        scheduled_vel = 0.0;
        	ROS_INFO("lidar caution zone: scheduled_vel = %f",scheduled_vel);
        }
        else { // not ready to decel, so target vel is v_max, either accel to it or hold it
            scheduled_vel = omega_max;
        }*/
        double ramping_factor = getRampingFactor(radian_to_go, omega_max, alpha_max);
        ROS_INFO("radian_to_go: %f, dist_decel: %f, odom_omega_: %f, scheduled_vel: %f", radian_to_go, dist_decel, odom_omega_, scheduled_vel);

		odom_omega_ = fabs(odom_omega_);
        //how does the current velocity compare to the scheduled vel?
        /*if (odom_omega_ < scheduled_vel) {  // maybe we halted, e.g. due to estop or obstacle;
            // may need to ramp up to v_max; do so within accel limits
            double v_test = odom_omega_ + alpha_max*dt_callback_;  // if callbacks are slow, this could be abrupt
            // operator:  c = (a>b) ? a : b;
            new_cmd_omega = (v_test < scheduled_vel) ? v_test : scheduled_vel; //choose lesser of two options
            // this prevents overshooting scheduled_vel
        } else if (odom_omega_ > scheduled_vel) { //travelling too fast--this could be trouble
            // ramp down to the scheduled velocity.  However, scheduled velocity might already be ramping down at a_max.
               // need to catch up, so ramp down even faster than a_max.  Try 1.2*a_max.
            ROS_INFO("odom vel: %f; sched vel: %f", odom_omega_, scheduled_vel); //debug/analysis output; can comment this out
            
            double v_test = odom_omega_ - 1.2 * alpha_max*dt_callback_; //moving too fast--try decelerating faster than nominal a_max
    
            new_cmd_omega = (v_test > scheduled_vel) ? v_test : scheduled_vel; // choose larger of two options...don't overshoot scheduled_vel
        } else {
            new_cmd_omega = scheduled_vel; //silly third case: this is already true, if here.  Issue the scheduled velocity
        }
        ROS_INFO("cmd vel: %f",new_cmd_vel); // debug output
    
        if (lidar_alarm_msg.data == true || estop_on.data == true) { // The robot should stop when either condition is true
            new_cmd_omega = 0.0;
			ROS_INFO("Halted the robot. Halt status = %i; Lidar alarm = %i; Estop = %i", halt_status.data, lidar_alarm_msg.data, estop_on.data);
        }
		*/
		new_cmd_omega = getVelocity(odom_omega_, ramping_factor, omega_max, alpha_max);

		if (segment_radian < 0.0) {
			new_cmd_omega = -new_cmd_omega;
        }
    
        cmd_vel.linear.x = 0.0; // linear command; always zero when rotating
        cmd_vel.angular.z = new_cmd_omega;
        if (radian_to_go <= radian_to_go_error) { // if any of these conditions is true, the robot should stop
            cmd_vel.angular.z = 0.0;
        }
        vel_cmd_publisher.publish(cmd_vel); // publish the command to jinx/cmd_vel
        cmd_vel_stamped.twist = cmd_vel;
        cmd_vel_stamped.header.stamp = ros::Time::now();
        vel_cmd_stamped_pub.publish(cmd_vel_stamped);
        rtimer->sleep(); // sleep for remainder of timed iteration
        if (radian_to_go <= radian_to_go_error) break; // halt this node when this segment is complete.
        // will want to generalize this to handle multiple segments
        // ideally, will want to receive segments dynamically as publications from a higher-level planner
    }
}

void next_segment(ros::ServiceClient& client, int segment_ID, double& segment_radian, double& segment_length) {  //function to determine what value to load for move instructions
    //instantiated an object of a consistent type for requests and responses with:
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_scheduler"); // name of this node will be "vel_scheduler"
    ros::NodeHandle nh; // get a ros nodehandle; standard yadda-yadda
    //create a publisher object that can talk to ROS and issue twist messages on named topic;
    // note: this is customized for stdr robot; would need to change the topic to talk to jinx, etc.
    
    /*if (argc < 3) {
        ROS_INFO("Velocity scheduler needs a topic name to published");
    }*/
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
    //this is the service name that David  had defined inside of path_planner node.
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

    int segment_tot = 4;
    for (int segment_ID = 0; ros::ok() && segment_ID <= segment_tot; segment_ID++) {
		double segment_radian = 0.0;
		double segment_length = 0.0;
		next_segment (client, segment_ID, segment_radian, segment_length);
        if (fabs(segment_radian) > segment_length){
            rotationFunc(vel_cmd_publisher, vel_cmd_stamped_pub, segment_radian);//call rotationFunc           
        } else {
            translationFunc(vel_cmd_publisher, vel_cmd_stamped_pub, segment_length);//call translationFunc    
        }
		// wait between path segments
		for (int wait = 0; wait < 0.5/DT; wait++){
			rtimer->sleep();
		}
    } 
    ROS_INFO("completed move distance");

	// clean up the dynamic memory
    delete rtimer;

    return 0;
}            
