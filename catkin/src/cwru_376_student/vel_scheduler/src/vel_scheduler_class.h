// include this file in "vel_scheduler.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times

#ifndef VEL_SCHEDULER_CLASS_H_
#define VEL_SCHEDULER_CLASS_H_

// some generically useful stuff to include...
#include <math.h>
#include <string>
#include <ros/ros.h> // ALWAYS need to include this

// message types used in this vel_scheduler node;  include more message types, as needed
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <path_planner/path_segment.h> // the service message class for getting the next path segments

// define a class, including a constructor, member variables and member functions
class VelSchedulerClass
{
public:
    VelSchedulerClass(ros::NodeHandle* nodehandle); // "main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    ~VelSchedulerClass();
    // may choose to define public methods or public variables, if desired
    void processSegment();
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support publisher, subscriber, and service
    
    // objects to support publisher
    ros::Publisher vel_cmd_publisher_;
    ros::Publisher vel_cmd_stamped_pub_;
    
    // objects to support subscriber
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_halt_;
    ros::Subscriber sub_lidar_alarm_; 
    ros::Subscriber sub_lidar_nearest_;
    ros::Subscriber sub_estop_status_;
    
    // objects to support service
    ros::ServiceClient client_;
    
    // Since in vel_scheduler.cpp the following part of the code is the variables with global scope,
    // so I treat these variables in vel_scheduler_class.h as private members which is accessible 
    // only from within other members of the same class.
    // Some limits on speed and acceleration. These are loaded from the ROS param server.
    double v_max_,
           v_max_original_,
           a_max_,
           omega_max_,
           alpha_max_;
    double DT_;
    double radian_to_go_error_;

    std::string cmd_vel_topic_,
                cmd_vel_stamped_topic_,
                odom_topic_;
    

    // For communication with odometry callbacks
    bool odom_initialized_;
    double odom_vel_,
           odom_omega_,
           odom_x_,
           odom_y_, 
           odom_quat_z_,
           odom_quat_w_;
    ros::Time t_last_callback_;
    double dt_odom_; // track the time between odom callbacks

    // For communication with lidar, user brake, and estop
    bool lidar_initialized_;
    std_msgs::Bool lidar_alarm_msg_;
    std_msgs::Float32 lidar_nearest_;
    std_msgs::Bool estop_on_;
    std_msgs::Bool halt_status_;
    // Values for controlling how lidar affects velocity
    double lidar_safe_distance_, // this will be loaded from the ROS param server
	   dist_safe_,
	   dist_danger_,
	   dist_critical_,
	   v_safe_;

    ros::Rate *rtimer_; // frequency corresponding to chosen sample period DT; the main loop will run this fast
    
    // member functions as well:
    void initializePublishers();
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializeServices();
    
    // prototype for callback functions
    // get info from the ROS param server. Populate most of our global variables.
    void getParams();
    
    // receive the pose and velocity estimates from the simulator or the physical robot
    // copy the relevant values to global variables
    // Note: stdr updates odom only at 10Hz; Jinx is ~50Hz
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);

    // callback to check for a user brake
    void haltCallback(const std_msgs::Bool& h_rcvd);

    // callback to receive the info from lidar alarm
    void lidarAlarmCallback(const std_msgs::Bool& la_rcvd);

    // callback to receive the info from lidar nearest
    void lidarNearestCallback(const std_msgs::Float32& ln_rcvd);

    // callback to receive the info on e-stop status
    void eStopStatusCallback(const std_msgs::Bool& ess_rcvd);
    
    //prototype for callback for ROS service
    //void getSegment(ros::ServiceClient& client, int segment_ID, double& segment_radian, double& segment_length);
    
    //prototype for some other member functions which is used to control the velocity, position, orientation and path segments for robot to go
    double getRampingFactor(double remaining, double vel, double acc);
    double getVelocity(double current_vel, double ramping_factor, double vel, double acc);
    void translationFunc (ros::Publisher& vel_cmd_publisher, ros::Publisher& vel_cmd_stamped_pub, double segment_length);
    void rotationFunc (ros::Publisher& vel_cmd_publisher, ros::Publisher& vel_cmd_stamped_pub, double segment_radian);
    void getSegment(ros::ServiceClient& client, int segment_ID, double& segment_radian, double& segment_length);
};
#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef


