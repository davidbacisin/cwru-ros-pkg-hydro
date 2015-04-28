// This is class based beta irb120 IM interface cpp file
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.h>
#include <irb120_kinematics.h>
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sensor_msgs/JointState.h>
using namespace std;

class BetaInterfaceClass{
public:
    BetaInterfaceClass(ros::NodeHandle* nodehandle); // "main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    // the input parameter is a ros::NodeHandle* pointer
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support publisher, subscriber, and service
    
    // objects to support publisher
    ros::Publisher pub_;
    
    // objects to support subscriber
    ros::Subscriber sub_js_;
    ros::Subscriber sub_im_;
    
    // objects to support service
    ros::ServiceServer service_;
    
    // Since in beta_example_irb120_IM_interface.cpp the following part of the code is the variables with global scope,
    // so I treat these variables in vel_scheduler_class.h as private members which is accessible 
    // only from within other members of the same class.
    // Some parameters to define the initial pose and orient and final pose and orient of abby manipulator.
    
    //callback to subscribe to marker state
    Eigen::Vector3d g_p_;
    Vectorq6x1 g_q_state_;  //typedef Eigen::Matrix<double, 6, 1> Vectorq6x1
    //geometry_msgs::Quaternion g_quat; // global var for quaternion
    Eigen::Quaterniond g_quat_;
    Eigen::Matrix3d g_R_;
    Eigen::Affine3d g_A_flange_desired_;
    bool g_trigger_;
    tf::TransformListener *g_tf_;
    geometry_msgs::PoseStamped marker_;
    geometry_msgs::PoseStamped wrtlink1_;
    /*Eigen::Vector3d p_;
    Eigen::Vector3d n_des_,t_des_,b_des_;
    std::vector<Vectorq6x1> q6dof_solns_;
    Vectorq6x1 qvec_;*/
    ros::Rate* sleep_timer_; // update rate    
    /*Irb120_fwd_solver irb120_fwd_solver_; //instantiate forward and IK solvers
    Irb120_IK_solver ik_solver_;
    Eigen::Vector3d n_urdf_wrt_DH_,t_urdf_wrt_DH_,b_urdf_wrt_DH_;
    Eigen::Matrix3d R_urdf_wrt_DH_;

    trajectory_msgs::JointTrajectory new_trajectory_; // an empty trajectory

    Eigen::Affine3d A_flange_des_DH_;
    
    int nsolns_;*/
    
    // member functions as well:
    void initializePublishers();
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializeServices();
    
    // prototype for callback functions
    // callback function to get the position and orientation of the InteractiveMarker
    void markerListenerCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    
    // callback function to get the current joints angles of ABBY 
    void jointStateCB(const sensor_msgs::JointStatePtr &js_msg);
    
    // callback function to the get service callback of trigger
    bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response);
    
    // prototype for some other member functions which is used to control the trajectory and function to command ABBY to move
    // function to command robot to move to "qvec" using a trajectory message, sent via ROS-I
    void stuff_trajectory(Vectorq6x1 p, Vectorq6x1 q, double c, trajectory_msgs::JointTrajectory &new_trajectory);
    
    // function to command ABBY to move
    void moveABBY();

    
};

// constructor can do the initialization work, including setting up subscribers, publishers and services
// can use member variables to pass data from subscribers to other member functions
// CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
// DEFAULT CONSTRUCTOR DEFINITION
// Since nh_ is of type ros::NodeHandle, so that it should = *nodehandle, *nodehandle represents the value pointed by nodehandle
BetaInterfaceClass::BetaInterfaceClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    initializePublishers();
    initializeSubscribers();
    initializeServices();

    g_trigger_ = false;
    sleep_timer_ = new ros::Rate(10.0);

    ROS_INFO("Initializing global variable");

    moveABBY();
}

void BetaInterfaceClass::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
}

void BetaInterfaceClass::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    sub_js_ = nh_.subscribe("/joint_states",1,&BetaInterfaceClass::jointStateCB, this);
    sub_im_ = nh_.subscribe("example_marker/feedback", 1, &BetaInterfaceClass::markerListenerCB, this);
}

void BetaInterfaceClass::initializeServices() {
    ROS_INFO("Initializing Services");
    service_ = nh_.advertiseService("move_trigger", &BetaInterfaceClass::triggerService, this); 
}

void BetaInterfaceClass::markerListenerCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    // transform
    marker_.header.frame_id = "base_link";
    marker_.header.stamp = ros::Time::now();
    marker_.pose.position.x = feedback->pose.position.x;
    marker_.pose.position.y = feedback->pose.position.y;
    marker_.pose.position.z = feedback->pose.position.z;
    marker_.pose.orientation.x = feedback->pose.orientation.x;
    marker_.pose.orientation.y = feedback->pose.orientation.y;
    marker_.pose.orientation.z = feedback->pose.orientation.z;
    marker_.pose.orientation.w = feedback->pose.orientation.w;
    try {
        g_tf_->transformPose("link1", marker_, wrtlink1_);
    }
    catch (tf::TransformException& exception) {
        ROS_ERROR("%s", exception.what());
    }
    //copy to global vars:
    g_p_[0] = feedback->pose.position.x;
    g_p_[1] = feedback->pose.position.y;
    g_p_[2] = feedback->pose.position.z;
    g_quat_.x() = feedback->pose.orientation.x;
    g_quat_.y() = feedback->pose.orientation.y;
    g_quat_.z() = feedback->pose.orientation.z;
    g_quat_.w() = feedback->pose.orientation.w;   
    g_R_ = g_quat_.matrix();
}

void BetaInterfaceClass::jointStateCB(const sensor_msgs::JointStatePtr &js_msg) {
    
    for (int i=0;i<6;i++) {
        g_q_state_[i] = js_msg->position[i];
    }
    //cout<<"g_q_state: "<<g_q_state.transpose()<<endl;
    
}

bool BetaInterfaceClass::triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    // grab the most recent IM data and repackage it as an Affine3 matrix to set a target hand pose;
    g_A_flange_desired_.translation() = g_p_;
    g_A_flange_desired_.linear() = g_R_;
    cout<<"g_p: "<<g_p_.transpose()<<endl;
    cout<<"R: "<<endl;
    cout<<g_R_<<endl;
    g_trigger_ = true; //inform "main" that we have a new goal!
    return true;
}

//command robot to move to "qvec" using a trajectory message, sent via ROS-I
void BetaInterfaceClass::stuff_trajectory(Vectorq6x1 p, Vectorq6x1 q, double c, trajectory_msgs::JointTrajectory &new_trajectory) {
    
    // declaring a 50 number of elements vector objector trajectory_points(50)
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points(25);
    
    // new_trajectory.points.clear(); 
    auto jointsSum = new_trajectory.joint_names.size();
    auto trajPointsSum = trajectory_points.size(); // This trajPointsSum is the sum value of the trajectory points ABBY go thru
    ROS_INFO("the number of joints: ",jointsSum);
    
    new_trajectory.header.stamp = ros::Time::now();  
    
    for (auto i = 0; i < trajPointsSum; ++i){
         trajectory_points[i].positions.clear();
    }
    
    Vectorq6x1 interJointAngle; // default-initializing a variable interJointAngle of type Vectorq6x1
    for (size_t i = 0; i < min( p.size(), q.size() ); ++i){
        interJointAngle[i] = (p[i] - q[i]) / (trajPointsSum - 1);
    }
    
    std::vector<Vectorq6x1> interJointAngleVec(trajPointsSum - 1); // declaring a vector which has trajectory_points.size() - 1 number of elements
    auto interJointAngleVecSum = interJointAngleVec.size(); // This interJointAngleVecSum is the sum value of the trajectory points ABBY go thru except the first points
    interJointAngleVec[0] = q + interJointAngle; // defining the first element of the vector
    // defining the rest of the element of the vector
    for (auto i = 1; i < interJointAngleVecSum; i++){
        interJointAngleVec[i] = interJointAngleVec[i-1] + interJointAngle;
    }
    
    // push back each incremented joints angles into the corresponding trajectory points positions
    for (int ijnt = 0; ijnt < jointsSum; ijnt++){
        trajectory_points[0].positions.push_back(q[ijnt]); // stuff in position commands for 6 joints
    }
    trajectory_points[0].time_from_start = ros::Duration(c);
    for (auto i = 1; i < trajPointsSum; i++){
        //time_from_start is relative to trajectory.header.stamp 
        //each trajectory point's time_from_start must be greater than the last
        for (int ijnt = 0; ijnt < jointsSum; ijnt++){
            trajectory_points[i].positions.push_back(interJointAngleVec[i-1](ijnt)); // stuff in position commands for 6 joints
        }
        trajectory_points[i].time_from_start = ros::Duration(c + 0.05*i); 
    }
    
    // start from home pose... really, should should start from current pose!
    // push back all the trajectory points into trajectory
    for (int i = 0; i < trajPointsSum; i++){
        new_trajectory.points.push_back(trajectory_points[i]);
    }
}

void BetaInterfaceClass::moveABBY () {
    double a = 0.0;
    double b = 1.4;
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    std::vector<Vectorq6x1> q6dof_solns;
    Vectorq6x1 qvec;
    Vectorq6x1 ptAboveTb; // the joints angles which make ABBY from current position to a point above from the table
    ptAboveTb << 0.0, -2*3.14/6, 2*3.14/10, 0.0, 2*3.14/10, 0.0;
    //sleep_timer_ = new ros::Rate(10.0); //10Hz update rate    
    Irb120_fwd_solver irb120_fwd_solver; //instantiate forward and IK solvers
    Irb120_IK_solver ik_solver;
    Eigen::Vector3d n_urdf_wrt_DH,t_urdf_wrt_DH,b_urdf_wrt_DH;
  
    n_urdf_wrt_DH <<0,0,1;
    t_urdf_wrt_DH <<0,1,0;
    b_urdf_wrt_DH <<-1,0,0;
    Eigen::Matrix3d R_urdf_wrt_DH;
    R_urdf_wrt_DH.col(0) = n_urdf_wrt_DH;
    R_urdf_wrt_DH.col(1) = t_urdf_wrt_DH;
    R_urdf_wrt_DH.col(2) = b_urdf_wrt_DH;    

    trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory
    new_trajectory.joint_names.push_back("joint_1");
    new_trajectory.joint_names.push_back("joint_2");
    new_trajectory.joint_names.push_back("joint_3");
    new_trajectory.joint_names.push_back("joint_4");
    new_trajectory.joint_names.push_back("joint_5");
    new_trajectory.joint_names.push_back("joint_6");
    
    Eigen::Affine3d A_flange_des_DH;
    
    //   A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(qvec); //fwd_kin_solve

    //std::cout << "A rot: " << std::endl;
    //std::cout << A_fwd_DH.linear() << std::endl;
    //std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl; 
    
    bool tf_is_initialized = false;
    tf::StampedTransform baseLink_wrt_link1;
    g_tf_ = new tf::TransformListener(nh_);
    while (!tf_is_initialized) {
        try {
            g_tf_->lookupTransform("base_link", "link1", ros::Time(0), baseLink_wrt_link1);
            tf_is_initialized = true;
        }
        catch (tf::TransformException& exception) {
            ROS_ERROR("%s", exception.what());
            tf_is_initialized = false;
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
    }

    int nsolns;
    
    while(ros::ok()) {
        ros::spinOnce();
        if (g_trigger_) {
            // ooh!  excitement time!  got a new tool pose goal!
            g_trigger_=false; // reset the trigger
            //is this point reachable?
            A_flange_des_DH = g_A_flange_desired_;
            // takes a transformation
            A_flange_des_DH.linear() = g_A_flange_desired_.linear() * R_urdf_wrt_DH.transpose();
            cout<<"R des DH: "<<endl;
            cout<<A_flange_des_DH.linear()<<endl;
            nsolns = ik_solver.ik_solve(A_flange_des_DH);
            ROS_INFO("there are %d solutions",nsolns);

            if (nsolns>0) {      
                ik_solver.get_solns(q6dof_solns);  
                //qvec = q6dof_solns[0];
                // defining a joint limits vector for joint 0 and joint 1, such that each joint is specified within a range of motion
                std::vector<double> jointLimits {0,-M_PI,-M_PI/2,M_PI/6};
                std::vector<int> weight{1,1,1,7,5,1}; //defining a weight vector
                double sum;
                double minimum = 1e6;
                int bestikSoluNo = 0;
                Vectorq6x1 oneIkSolu;
                // Once nsolns > 0, choose a IK solution both meet  the specified joint limit and the weight requirement
                for (int i = 0; i < q6dof_solns.size(); ++i){
                    oneIkSolu = q6dof_solns[i];
                    if (oneIkSolu[0] < jointLimits[0] && oneIkSolu[0] > jointLimits[1] 
                        && oneIkSolu[1] < jointLimits[3] && oneIkSolu[1] > jointLimits[2]) {
                        sum = 0;
                        for (int ijnt = 0; ijnt < 6; ++ijnt){
                            sum = sum + oneIkSolu[ijnt] * weight[ijnt];
                        }
                        if (sum < minimum){
                            minimum = sum;
                            bestikSoluNo = i; // remember the IK solution which has the minimum last joint angle solution
                        }
                    }
                }
                qvec = q6dof_solns[bestikSoluNo];
                new_trajectory.points.clear();  
                stuff_trajectory(ptAboveTb, g_q_state_,a,new_trajectory);
                stuff_trajectory(qvec,ptAboveTb,b,new_trajectory);


                pub_.publish(new_trajectory);
            }
        }
        sleep_timer_ ->sleep();      
    }
}



// main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_listener"); // this will be the node name;
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ROS_INFO("main: instantiating an object of type BetaInterfaceClass");
    // Since the input parameter of betaInterfaceClass's constructor is of type ros::NodeHandle* which is a pointer type, so that
    // the &nh (a pointer) should be used as the input when we instantiate an BetaInterfaceClass object
    BetaInterfaceClass betaInterfaceClass(&nh);  
    //instantiate an BetaInterfaceClass object called betaInterfaceClass and pass in pointer to nodehandle for constructor to use

    return 0;
}