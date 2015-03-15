// intent of this program: modulate the velocity command to comply with a speed limit, v_max,
// acceleration limits, +/-a_max, and come to a halt gracefully at the end of
// an intended line segment

// this header incorporates all the necessary #include files and defines the class "VelScheduler"
#include "vel_scheduler_class.h"

// constructor can do the initialization work, including setting up subscribers, publishers and services
// can use member variables to pass data from subscribers to other member functions
// CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
// DEFAULT CONSTRUCTOR DEFINITION
VelSchedulerClass::VelSchedulerClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("in class constructor of VelScheduler");
    initializePublisher();
    initializeSubscriber();
    initializeServices();
    
    // Some limits on speed and acceleration. These are loaded from the ROS param server.
    v_max_ = 0.0; // 1m/sec is a slow walk
    v_max_original_ = 0.0; // lidar will change v_max during execution, so save a backup of our original v_max
    a_max_ = 0.0; // 1m/sec^2 is 0.1 g's
    omega_max_ = 0.0; // 1 rad/sec about 6 seconds to rotate 1 full rev
    alpha_max_ = 0.0; // 0.5 rad/sec^2 takes 2 sec to get from rest to full omega
    DT_ = 1.0/50.0; // choose an update rate of 50Hz
    radian_to_go_error_ = 0.1;

    // For communication with odometry callbacks
    odom_initialized_ = false;
    odom_vel_ = 0.0; // measured/published system speed
    odom_omega_ = 0.0; // measured/published system yaw rate (spin)
    odom_x_= 0.0;
    odom_y_ = 0.0;
    odom_quat_z_ = 0.0;
    odom_quat_w_ = 0.0;
    //ros::Time t_last_callback_;
    dt_odom_= 0.0; // track the time between odom callbacks

    // For communication with lidar, user brake, and estop
    lidar_initialized_ = false; // to check if our data is valid
    /*std_msgs::Bool lidar_alarm_msg_;
    std_msgs::Float32 lidar_nearest_;
    std_msgs::Bool estop_on_;
    std_msgs::Bool halt_status_;*/

    // Values for controlling how lidar affects velocity
    lidar_safe_distance_ = 0.0; // this will be loaded from the ROS param server
    dist_safe_ = 0.0;
    dist_danger_ = 0.0;
    dist_critical_ = 0.0;
    v_safe_ = 0.0;
    
    // initialize the timer
    velSchedulerClass.rtimer_ = new ros::Rate(1.0 / DT_);
    ROS_INFO("Waiting for odom data");
    // wait until odom is ready
    while (!velSchedulerClass.odom_initialized_){
            ros::spinOnce();
            rtimer_->sleep();
    }

    // Wait until path_planner is ready to send data to us
    path_planner::path_segment srv;
    srv.request.id = 0;
    while (!velSchedulerClass.client_.call(srv)){
	rtimer_->sleep();
    }
    //ros::Rate *rtimer_; // frequency corresponding to chosen sample period DT; the main loop will run this fast
}
//member helper function to set up publishers;
void VelSchedulerClass::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    vel_cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_topic", 1);
    vel_cmd_stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped_topic", 1); 
    //add more publishers, as needed
    // note: COULD make publishers object as public member function, if want to use it within "main()"
}
    
// member helper function to set up subscribers;
// for instance: note odd syntax: &VelSchedulerClass::odomCallback is a pointer to a member function of VelSchedulerClass
// "this" keyword is required, to refer to the current instance of VelSchedulerClass
void VelSchedulerClass::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    sub_odom_ = nh_.subscribe("odom_topic", 1, &VelSchedulerClass::odomCallback, this);
    sub_halt_ = nh_.subscribe("user_brake", 1, &VelSchedulerClass::haltCallback, this);
    sub_lidar_alarm_ = nh_.subscribe("lidar_alarm", 1, &VelSchedulerClass::lidarAlarmCallback, this);
    sub_lidar_nearest_ = nh_.subscribe("lidar_nearest", 1, &VelSchedulerClass::lidarNearestCallback, this);
    sub_estop_status_ = nh_.subscribe("estop_status", 1, &VelSchedulerClass::eStopStatusCallback, this);
    //add more publishers, as needed   
}
    
// member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
// creates a ROS “ServiceClient”. This service client expects to communicate requests and responses 
// as defined in: path_planne::path_segment. Also, this service client expects to communicate with 
// a named service, called "path_planner_service". 
// this is the service name that David had defined inside of path_planner node.
void VelSchedulerClass::initializeServices()
{
    ROS_INFO("Initializing Services");
    client_ = nh_.serviceClient<path_planner::path_segment>("path_planner_service");
}
    
// member helper function to set up getParams
// get info from the ROS param server. Populate most of our global variables.
void VelSchedulerClass::getParams(ros::NodeHandle& nh_){
    // get the velocity/acceleration max values as specified by ROS params
    // third param of nh.param is the default value
    nh.param("/vel_scheduler/max_linear_velocity", v_max_, 1.0);
    v_max_original_ = v_max_;
    nh_.param("/vel_scheduler/max_linear_acceleration", a_max_, 1.0);
    nh_.param("/vel_scheduler/max_angular_velocity", omega_max_, 1.0);
    nh_.param("/vel_scheduler/max_angular_acceleration", alpha_max_, 0.5);
    nh_.param("/lidar_alarm/lidar_safe_distance", lidar_safe_distance_, 0.5);
    // set safe distances
    dist_safe_ = lidar_safe_distance_ + 2.0;
    dist_danger_ = lidar_safe_distance_ + 0.25;
    dist_critical_ = lidar_safe_distance_;
    
    // load some topic names from the ROS param server
    // these topics differ from the simulator vs actual robot
    std::string cmd_vel_topic_,
                cmd_vel_stamped_topic_,
                odom_topic_;
    if (!nh.getParam("/vel_scheduler/cmd_vel_topic", cmd_vel_topic_)){
            ROS_WARN("vel_scheduler needs ROS param cmd_vel_topic");
            return 0;
    }
    if (!nh.getParam("/vel_scheduler/cmd_vel_stamped_topic", cmd_vel_stamped_topic_)){
            ROS_WARN("vel_scheduler needs ROS param cmd_vel_stamped_topic");
            return 0;
    }
    if (!nh.getParam("/vel_scheduler/odom_topic", odom_topic_)){
            ROS_WARN("vel_scheduler needs a ROS param odom_topic");
            return 0;
    }
}
    
// receive the pose and velocity estimates from the simulator or the physical robot
// copy the relevant values to global variables
// Note: stdr updates odom only at 10Hz; Jinx is ~50Hz
void VelSchedulerClass::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
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
    odom_initialized_ = true;

    // debug output
    ROS_INFO("odom CB: x = %f, y= %f, quat_z = %f, quat_w = %f, v = %f, omega = %f", odom_x_, odom_y_, odom_quat_z_, odom_quat_w_, odom_vel_, odom_omega_);
}
    
// member function implementation for receiving haltCallback information
void VelScheduler::haltCallback(const std_msgs::Bool& h_rcvd){    
    // the output below could get annoying; may comment this out, but useful initially for debugging
    ROS_INFO("halt status: %i", h_rcvd.data);
    // Update the global variable with the halt status
    halt_status_.data = h_rcvd.data;
}
    
// member function implementation for receiving lidarAlarmCallback information
void VelScheduler::lidarAlarmCallback(const std_msgs::Bool& la_rcvd){    
    ROS_INFO("received lidar alarm value is: %i", la_rcvd.data);
    // post the received data in a global var for access by main prog
    lidar_alarm_msg_.data = la_rcvd.data;
}

// member function implementation for receiving lidarNearestCallback information
void VelScheduler::lidarNearestCallback(const std_msgs::Float32& ln_rcvd){    
    // check for data on topic "lidar_dist" 
    ROS_INFO("received lidar nearest value is: %f", ln_rcvd.data);
    // post the received data in a global var for access by main prog
    lidar_nearest_.data = ln_rcvd.data;
    lidar_initialized_ = true;
}

// member function implementation for receiving eStopStatusCallback information
void VelScheduler::eStopStatusCallback(const std_msgs::Bool& ess_rcvd){  
    // check for data on topic ""lidar_alarm"" 
    ROS_INFO("received estop status value is: %i", ess_rcvd.data);
    // post the received data in a global var for access by main prog
    estop_on_.data = !ess_rcvd.data; // boolean inverse just so that estop makes more intuitive sense
}

// member function implementation for receiving the calculating for the percentage of our max speed that we _want_ to go, based off of the distance remaining
double VelScheduler::getRampingFactor(double remaining_, double vel_, double acc_){
    double ramping_factor_ = 0.0,
           time_to_decel_ = vel_/acc_, // basic physics
           dist_decel_ = 0.5 * acc * (time_to_decel_ * time_to_decel_);
    if (remaining_ <= 0.0) { // at goal, or overshot; stop!
        ramping_factor_ = 0.0;
    }
    else if (halt_status_.data == true) { // if user brake, then stop!
        ramping_factor_ = 0.0;
    }
    else if (remaining_ <= dist_decel_) { // possibly should be braking to a halt
        // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
        // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
        // and factor = sqrt(2*dist*a)/v_max
        if (vel_ == 0.0) { // avoid dividing by zero
            ramping_factor_ = 0.0;
        }
        else {
            ramping_factor_ = sqrt(2 * remaining_ * acc_) / vel_;
        }
        ROS_INFO("braking zone: ramping_factor_ = %f", ramping_factor_);
    }
    else { // otherwise, go full speed
        ramping_factor_ = 1.0;
    }
    return ramping_factor_;
}

// member function implementation for receiving the calculating the truly desired velocity based off the current velocity and ramping factor
double VelScheduler::getVelocity(double current_vel_, double ramping_factor_, double vel_, double acc_){
    double ramped_vel_ = ramping_factor_ * vel_,
           new_vel_ = ramped_vel_;
           ROS_INFO("ramped_vel: %f, current_vel: %f", ramped_vel_, current_vel);
    // how does the current velocity compare to the desired (ramped) vel?
    if (current_vel_ < ramped_vel_) {  // maybe we halted, e.g. due to estop or obstacle;
        // may need to ramp up to v_max; do so within accel limits
        double v_test_ = current_vel_ + acc_*(1.0 - current_vel_/ramped_vel_); // reduce acceleration as we get closer to our goal
        new_vel_ = (v_test_ < ramped_vel_) ? v_test_ : ramped_vel_; // don't overshoot ramped_vel

        // debug
        ROS_INFO("v_test: %f, acc: %f, dt_odom_: %f", v_test_, acc_, dt_odom_);
    }
    else if (current_vel_ > ramped_vel_) { // travelling too fast. Ramp down.
        double v_test_ = current_vel_ - 1.2 * acc_; // moving too fast--try decelerating faster than nominal acc
        new_vel_ = (v_test_ > ramped_vel_) ? v_test_ : ramped_vel_; // don't overshoot ramped_vel

     // debug
     ROS_INFO("odom vel: %f; sched vel: %f", odom_vel_, ramped_vel_);
    } else {
            new_vel_ = ramped_vel_; // silly third case: this is already true, if here. Issue the desired velocity
    }
    // ensure there is no emergency stop!
    if (lidar_alarm_msg_.data == true || estop_on_.data == true) { // The robot should stop when either condition is true
    new_vel_ = 0.0;
		// debug
    ROS_INFO("Halted the robot. User brake status = %i; Lidar alarm = %i; Estop = %i", halt_status_.data, lidar_alarm_msg_.data, estop_on_.data);
    }
    return new_vel_;
}

// member function implementation for control robot translation
void VelScheduler::translationFunc (ros::Publisher& vel_cmd_publisher_, ros::Publisher& vel_cmd_stamped_pub_, double segment_length_){     
    double start_x_ = odom_x_; // save our starting position so we can calculate how far we've gone
    double start_y_ = odom_y_;
    ROS_INFO("start pose: x %f, y= %f", start_x_, start_y_);

    geometry_msgs::Twist cmd_vel_; //create a variable of type "Twist" to publish speed/spin commands
    geometry_msgs::TwistStamped cmd_vel_stamped_; // and a twist with timestamp

    cmd_vel_.linear.x = 0.0; // initialize these values to zero
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    cmd_vel_.angular.z = 0.0;
	
    double dist_to_go_ = segment_length_, // track how much further we need to go
           new_cmd_vel_ = 0.0; // value of speed to be commanded; update each iteration

    while (ros::ok() && dist_to_go_ > 0.0) { // stop when ROS says so or when we've finished the segment
        // allow callbacks to populate fresh data
        ros::spinOnce();
        // compute distance travelled so far:
        double delta_x_ = odom_x_ - start_x_;
        double delta_y_ = odom_y_ - start_y_;
        double segment_length_done_ = sqrt(delta_x_ * delta_x_ + delta_y_ * delta_y_);
        ROS_INFO("dist travelled: %f", segment_length_done_);
                // calculate how much further we need to go
        dist_to_go_ = segment_length_ - segment_length_done_;
		
        // get our velocity ramping factor
        double ramping_factor_ = getRampingFactor(dist_to_go_, v_max_, a_max_);
    
        // check the lidar to see if we should slow down or stop
        if (lidar_initialized_){ // make sure we've gotten data from the lidar so that lidar_nearest will be initialized
            double dist_to_stop_ = lidar_nearest_.data;

            // debug
            ROS_INFO("Lidar nearest: %f", lidar_nearest_.data);

            if (dist_to_stop_ <= dist_critical_){ // stop in the critical zone
                    v_max_ = 0.0;
                    ROS_INFO("Lidar critical zone: v_max = %f", v_max_);
            }
            else if (dist_to_stop_ <= dist_danger_){ // go our safe speed in the danger zone
                    v_max_ = v_safe_;
                    ROS_INFO("Lidar danger zone: v_max = %f", v_max_);
            }
            else if (dist_to_stop_ <= dist_safe_) { // in the safe zone, speed is proportional to how far away the object is
                    v_max_ = v_max_original_ * (dist_to_stop_ / dist_safe_);
                    ROS_INFO("Lidar safe zone: v_max = %f", v_max_);
            }
            else { // go as fast as possible otherwise
                    v_max_ = v_max_original_;
                    ROS_INFO("Lidar carefree zone: v_max = %f", v_max_);
            }
        }

        // now get the velocity
        new_cmd_vel_ = getVelocity(odom_vel_, ramping_factor_, v_max_, a_max_);

        // prevent robot from going backwards
        if (new_cmd_vel_ < 0.0) {
            new_cmd_vel_ = 0.0;
        }

        // ensure once more that we haven't reached our destination
        if (dist_to_go_ <= 0.0) {
            new_cmd_vel_ = 0.0;
        }

        cmd_vel_.linear.x = new_cmd_vel_;
        cmd_vel_.angular.z = 0.0; // spin command; always zero when we're moving linearly
                // publish the command
        vel_cmd_publisher_.publish(cmd_vel_);
                // also publish a time-stamped command
        cmd_vel_stamped_.twist = cmd_vel_;
        cmd_vel_stamped_.header.stamp = ros::Time::now();
        vel_cmd_stamped_pub_.publish(cmd_vel_stamped_);
        rtimer_->sleep(); // sleep for remainder of timed iteration
    }    
}

// member function implementation for control robot rotation
void VelScheduler::rotationFunc (ros::Publisher& vel_cmd_publisher_, ros::Publisher& vel_cmd_stamped_pub_, double segment_radian_){
    double start_x_ = odom_x_, // save our starting position so we can calculate when we're done
           start_y_ = odom_y_,
           start_quat_z_ = odom_quat_z_,
           start_quat_w_ = odom_quat_w_;
    ROS_INFO("start pose: x %f, y= %f, phi = %f", start_x_, start_y_, 2.0 * atan2(start_quat_w_, start_quat_z_));

    geometry_msgs::Twist cmd_vel_; //create a variable of type "Twist" to publish speed/spin commands
    geometry_msgs::TwistStamped cmd_vel_stamped_; // and a twist with timestamp

    cmd_vel_.linear.x_ = 0.0; // initialize these values to zero
    cmd_vel_.linear.y_ = 0.0;
    cmd_vel_.linear.z_ = 0.0;
    cmd_vel_.angular.x_ = 0.0;
    cmd_vel_.angular.y_ = 0.0;
    cmd_vel_.angular.z_ = 0.0;

    double radian_to_go_ = fabs(segment_radian_), // track how much further we need to go
		new_cmd_omega_ = 0.0; // value of spin to be commanded; update each iteration

    while (ros::ok() && radian_to_go_ > radian_to_go_error_) { // terminate if ROS has faulted or if we've finished the segment
    // allow callbacks to populate fresh data
        ros::spinOnce();
        // compute the angle between the start and current orientation
        // formula from http://math.stackexchange.com/questions/90081/quaternion-distance
        double segment_radian_done_ = acos(2.0*pow(odom_quat_z_ * start_quat_z_ + odom_quat_w_ * start_quat_w_, 2)-1.0);
        // ROS_INFO("dist travelled: %f",segment_radian_done_);
        radian_to_go_ = fabs(segment_radian_) - fabs(segment_radian_done_);
	// are we close enough to zero that we should assume zero?
        if (radian_to_go_ <= radian_to_go_error_) {
                radian_to_go_ = 0.0;
        }
    
        // get the velocity ramping factor
        double ramping_factor_ = getRampingFactor(radian_to_go_, omega_max_, alpha_max_);

        // getVelocity is well-behaved only with positive numbers
        odom_omega_ = fabs(odom_omega_);

        // get the velocity
        new_cmd_omega_ = getVelocity(odom_omega_, ramping_factor_, omega_max_, alpha_max_);

        // if our segment is negative, flip the sign on the velocity
        if (segment_radian_ < 0.0) {
            new_cmd_omega_ = -new_cmd_omega_;
        }
		
        // ensure once more that we haven't reached our destination
        if (radian_to_go_ <= radian_to_go_error_) {
            new_cmd_omega_ = 0.0;
        }

        cmd_vel_.linear.x = 0.0; // linear command; always zero when rotating
        cmd_vel_.angular.z = new_cmd_omega_;
        // publish the command
        vel_cmd_publisher_.publish(cmd_vel_); 
        // publish the time-stamped command
        cmd_vel_stamped_.twist = cmd_vel_;
        cmd_vel_stamped_.header.stamp = ros::Time::now();
        vel_cmd_stamped_pub_.publish(cmd_vel_stamped_);
        rtimer_->sleep(); // sleep for remainder of timed iteration
    }
}

// member function implementation for getSegment
// gets the path segment from the service client
// ampersands in the arguments allow us to return those values to the calling scope
void getSegment(ros::ServiceClient& client_, int segment_ID_, double& segment_radian_, double& segment_length_) {  //function to determine what value to load for move instructions
    // instantiate an object of a consistent type for requests and responses with:
    path_planner::path_segment srv_;
    srv_.request.id = segment_ID; //requests the next segment by its segment id in the server
    
    if (client_.call(srv_)) { //check to see if that segment id exists
	segment_radian_ = srv_.response.heading; //load distance to rotate
	segment_length_ = srv_.response.distance; //load distance to move
    } else {
	segment_length_ = 0.0; //if segment does not exist do not move
	segment_radian_ = 0.0;
    }

    ROS_INFO("Received segment #%i with heading %f and length %f", segment_ID_, segment_radian_, segment_length_);
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "velSchedulerClass"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    /*// load some topic names from the ROS param server
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
    }*/
    
    ROS_INFO("main: instantiating an object of type VelSchedulerClass");
    VelSchedulerClass velSchedulerClass(&nh);  //instantiate an VelSchedulerClass object called velSchedulerClass  and pass in pointer to nodehandle for constructor to use
    
    /*// initialize the timer
    velSchedulerClass.rtimer_ = new ros::Rate(1.0 / DT);
    ROS_INFO("Waiting for odom data");
    // wait until odom is ready
    while (!velSchedulerClass.odom_initialized_){
            ros::spinOnce();
            rtimer->sleep();
    }

    // Wait until path_planner is ready to send data to us
    path_planner::path_segment srv;
    srv.request.id = 0;
    while (!velSchedulerClass.client_.call(srv)){
	rtimer->sleep();
    }*/

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

    return 0;
} 
    