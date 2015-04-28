//test_ik_traj_sender2.cpp:
// compute a sequence of candidate solutions, then plan and execute a path to each
//wsn, March 2015
//used to test/visualize IK solutions.
// e.g.:
//    z_des = 0.3;
//    x_des = 0.4;
//    y_des = 0.45;
// sends robot to 8 different solutions
// watch out...may need to lead joint to goal, else may hit limit w/ wrap-around direction
// 

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <interactive_markers/interactive_marker_server.h>
#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <Eigen/Eigen>

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include <irb120_kinematics.h>
#include <joint_space_planner.h>
#define VECTOR_DIM 6 // chooose t plan w/ 6-dof vectors

// define some global variable
Vectorq6x1 g_q_state;  //typedef Eigen::Matrix<double, 6, 1> Vectorq6x1
Eigen::Quaterniond g_quat;
Eigen::Matrix3d g_R;
Eigen::Vector3d g_p;
geometry_msgs::PoseStamped marker;
geometry_msgs::PoseStamped wrtlink1;
tf::StampedTransform baseLink_wrt_link1;
tf::TransformListener *g_tf;
bool g_trigger=false;
Eigen::Affine3d g_A_flange_desired;

using namespace std;

void markerListenerCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
  // transform
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = feedback->pose.position.x;
  marker.pose.position.y = feedback->pose.position.y;
  marker.pose.position.z = feedback->pose.position.z;
  marker.pose.orientation.x = feedback->pose.orientation.x;
  marker.pose.orientation.y = feedback->pose.orientation.y;
  marker.pose.orientation.z = feedback->pose.orientation.z;
  marker.pose.orientation.w = feedback->pose.orientation.w;
  try {
    g_tf->transformPose("link1", marker, wrtlink1);
  }
  catch (tf::TransformException& exception) {
    ROS_ERROR("%s", exception.what());
  }
  g_p[0] = wrtlink1.pose.position.x;
  g_p[1] = wrtlink1.pose.position.y;
  g_p[2] = wrtlink1.pose.position.z;
  g_quat.x() = wrtlink1.pose.orientation.x;
  g_quat.y() = wrtlink1.pose.orientation.y;
  g_quat.z() = wrtlink1.pose.orientation.z;
  g_quat.w() = wrtlink1.pose.orientation.w; 
  g_R = g_quat.matrix();
}

// function: used to get the current joint angle, so that we can
// use these current value to make abby moving from current position
// to the next position
// void jointStateCB(const sensor_msgs::JointStatePtr &js_msg) {
    
//     for (int i=0;i<6;i++) {
//         g_q_state[i] = js_msg->position[i];
//     }
//     //cout<<"g_q_state: "<<g_q_state.transpose()<<endl;
    
// }

bool triggerService(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    // grab the most recent IM data and repackage it as an Affine3 matrix to set a target hand pose;
    g_A_flange_desired.translation() = g_p;
    g_A_flange_desired.linear() = g_R;
    cout<<"g_p: "<<g_p.transpose()<<endl;
    cout<<"R: "<<endl;
    cout<<g_R<<endl;
    g_trigger=true; //inform "main" that we have a new goal!
    return true;
}

// can data and callback
struct {
  bool is_found;
  geometry_msgs::PointStamped top;
  ros::Subscriber subscriber; 
} can;
void canTopCallback(const geometry_msgs::PointStamped& pt) {
  can.top = pt;
  can.is_found = true;
}


int main(int argc, char** argv) 
{
  // ROS set-ups:
  ros::init(argc, argv, "gripper_action"); //node name

  ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
  
  ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
  // ros::Subscriber sub_js = nh.subscribe("/joint_states",1,jointStateCB);
  ros::Subscriber sub_im = nh.subscribe("example_marker/feedback", 1, markerListenerCB);
  ros::ServiceServer service = nh.advertiseService("move_trigger", triggerService);
  can.subscriber = nh.subscribe("/can_top_position", 1, canTopCallback);

  Eigen::Vector3d p;
  Eigen::Vector3d n_des,t_des,b_des;
  std::vector<Vectorq6x1> q6dof_solns;
  Vectorq6x1 qvec;
  double x_des,y_des,z_des;
  std::vector<std::vector<Eigen::VectorXd> > path_options; 
  std::vector<Eigen::VectorXd>  single_layer_nodes;     
  std::vector<Eigen::VectorXd> optimal_path;
  Eigen::VectorXd weights;
  
  bool tf_is_initialized = false;
  g_tf = new tf::TransformListener(nh);
  while (!tf_is_initialized) {
    try {
      g_tf->lookupTransform("link1", "base_link", ros::Time(0), baseLink_wrt_link1);
      tf_is_initialized = true;
    }
    catch (tf::TransformException& exception) {
      ROS_ERROR("%s", exception.what());
      tf_is_initialized = false;
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }
  }
  // desired position the gripper should go
  x_des = g_p[0];
  y_des = g_p[1];
  z_des = g_p[2];
  // z_des = 0.3;
  // x_des = 0.4;
  // y_des = 0.45;

  // desired orientation the flange should keep, n:=x, t:=y, b:=z
  n_des<<1,0,0; // projection on x axis of link1
  t_des<<0,1,0; // projection on y axis of link1
  b_des = n_des.cross(t_des); // projection on z axis of link1

  Eigen::Matrix3d R_des;
  R_des.col(0) = n_des;
  R_des.col(1) = t_des;
  R_des.col(2) = b_des;
  // n_des << 1,0,0;
  // t_des << 0,1,0;
  // b_des = n_des.cross(t_des);
  
  // Eigen::Matrix3d R_des;
  // R_des.col(0) = n_des;
  // R_des.col(1) = t_des;
  // R_des.col(2) = b_des;
  
	Eigen::Affine3d a_tool_des; // expressed in DH frame
	a_tool_des.linear() = R_des;

//   //ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate to send new traj points as commands
  trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory

  trajectory_msgs::JointTrajectoryPoint trajectory_point1;
//   trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
//   // build an example trajectory:
  trajectory_point1.positions.clear();    
//   trajectory_point2.positions.clear();  
  
  new_trajectory.points.clear();
  new_trajectory.joint_names.push_back("joint_1");
  new_trajectory.joint_names.push_back("joint_2");
  new_trajectory.joint_names.push_back("joint_3");
  new_trajectory.joint_names.push_back("joint_4");
  new_trajectory.joint_names.push_back("joint_5");
  new_trajectory.joint_names.push_back("joint_6");   
  
//   //specify two points: initially, all home angles
//   for (int ijnt=0;ijnt<6;ijnt++) {
//       trajectory_point1.positions.push_back(0.0); // stuff in position commands for 6 joints
//       //should also fill in trajectory_point.time_from_start
//       trajectory_point2.positions.push_back(0.0); // stuff in position commands for 6 joints        
//   }
//   //ros::Duration t_from_start(0); //initialize duration to 0
  double t=0.0;
//   //double dt = 3;          
//   trajectory_point1.time_from_start =    ros::Duration(0);   
//   //trajectory_point2.time_from_start =    ros::Duration(2);  
//   new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector   
//   //new_trajectory.points.push_back(trajectory_point2); // add this single trajectory point to the trajectory vector      
//   new_trajectory.header.stamp = ros::Time::now();      
  
  ros::Rate sleep_timer(1.0); //1Hz update rate
  Irb120_fwd_solver irb120_fwd_solver;
  Irb120_IK_solver ik_solver;

  Eigen::Affine3d A_fwd_DH;
  
//   ROS_INFO("going home");
//   //for (int i=0;i<5;i++)
//  //{
//   pub.publish(new_trajectory);
//       ros::spinOnce();
//       sleep_timer.sleep();
// //}

//   new_trajectory.points.clear();
//   // start from home pose
//   new_trajectory.points.push_back(trajectory_point1); // add this single trajectory point to the trajectory vector   
//   new_trajectory.header.stamp = ros::Time::now();        
//   // now see about multiple solutions:

  qvec<<0,0,0,0,0,0;
  int nsolns=0;
  int nlayer = 0;
  for (double x_var = can.top.point.z + 0.18; x_var < x_des; x_var += 0.01) {
    p[0] = x_var;
    p[1]=  y_des;
    p[2] = z_des;
    a_tool_des.translation()=p;

    //is this point reachable?
    nsolns = ik_solver.ik_solve(a_tool_des); // for a specific y, the nsolns gives the number of IK solution.
    ROS_INFO("there are %d solutions",nsolns);
    ik_solver.get_solns(q6dof_solns);
    // if nsolns > 0, then we put all these solution into a vector named single_layer_nodes, so that
    // this layer only contains all the IK solution for y has a specific height
    if (nsolns>0) {
        Eigen::VectorXd soln_node;
        single_layer_nodes.clear();
        for (int isoln =0; isoln<nsolns;isoln++) {
            soln_node = q6dof_solns[isoln]; // convert to compatible datatype, the vector q6dof_solns stores all the solution of each value of y
            // and each element is a small vector which is composed of six joint angles
            single_layer_nodes.push_back(soln_node); // what this vector contained is as same as soln_node, but different type
        }
      path_options.push_back(single_layer_nodes); // this vector will contain all the IK solution for y changing form -0,4 to 0.4
      nlayer = path_options.size();
      ROS_INFO("filled layer %d",nlayer);
    }
  }

  optimal_path.resize(nlayer);    
  weights.resize(VECTOR_DIM);
  for (int i=0;i<VECTOR_DIM;i++) { // default--assign all weights equal 
      weights(i) = 1.0;
  }
     //do some planning:
   cout<<"instantiating a JointSpacePlanner:"<<endl;
   { //limit the scope of jsp here:
     JointSpacePlanner jsp (path_options,weights);
     cout<<"recovering the solution..."<<endl;
     jsp.get_soln(optimal_path); // HERE we goy the OPTIMAL PATH for moving manipulator
     // for each element optimal_path it only contains one specific IK solution while y changed
     // so that this vector is composed of the best IK solution combination for y changing from
     // start point to end point, which this result is obtained based on stagecoach problem
     //double trip_cost= jsp.get_trip_cost();

   }

   //now, jsp is deleted, but optimal_path lives on:
   cout<<"resulting solution path: "<<endl;
   for (int ilayer=0;ilayer<nlayer;ilayer++) {
       cout<<"ilayer: "<<ilayer<<" node: "<<optimal_path[ilayer].transpose()<<endl;
   }        

 

  // try to execute the plan:
  double dt = 0.4;
   //t+= 5.0; // go from home to first point in N sec; for simu, this does not behave same as on actual robot;
  for (int ilayer = 0; ilayer < nlayer; ilayer++) {
    qvec = optimal_path[ilayer];
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point1.positions[ijnt] = qvec[ijnt]; // put joint angles into trajectory_msgs::JointTrajectoryPoint varible
        // trajectory_point position member function.
    }
    t += dt;    
    trajectory_point1.time_from_start =    ros::Duration(t);  
    new_trajectory.points.push_back(trajectory_point1); // append another point
    // t += dt;    
    //trajectory_point1.time_from_start =    ros::Duration(t);   
    //new_trajectory.points.push_back(trajectory_point1); // go home between pts
    std::cout<<"qsoln = "<<qvec.transpose()<<std::endl;
    A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(qvec); //fwd_kin_solve

    std::cout << "A rot: " << std::endl;
    std::cout << A_fwd_DH.linear() << std::endl;
    std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;      
  }
  

  int npts = new_trajectory.points.size(); 
  int njnts = new_trajectory.points[0].positions.size();
  ROS_INFO("sending a trajectory with %d poses, each with %d joints ",npts,njnts);

  pub.publish(new_trajectory);
  ros::spinOnce();
  sleep_timer.sleep();
  return 0;
} 

