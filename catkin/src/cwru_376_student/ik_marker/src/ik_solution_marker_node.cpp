// marker_example.cpp
// Wyatt Newman, demo how to place markers in rviz
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <irb120_kinematics.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>


// declaring some global variable
geometry_msgs::Point makerPt;
int bestIkSoluNum;


// callback function
void reachPtCallback(const geometry_msgs::Point& ptRcvd) {
    // populate the message recieved from subscribed topic
    makerPt.x = ptRcvd.x;
    makerPt.y = ptRcvd.y;
    makerPt.z = ptRcvd.z;
    // debugging output
    ROS_INFO("marker position: x = %f, y = %f, z  = %f", makerPt.x, makerPt.y, makerPt.z);
}

// callback function
void bestIkSoluNumCallback(const std_msgs::Int16& bestIkSoluNumRcvd) {
    // debugging output
    ROS_INFO("The best IK solution No.: %i", bestIkSoluNumRcvd.data);
    bestIkSoluNum = bestIkSoluNumRcvd.data;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ik_solu_maker_placer");
    ros::NodeHandle nh;


    //ROS_INFO("hello, world");
    ros::Rate timer(1); //timer to run at 4 Hz
    ros::Subscriber rechPtSub = nh.subscribe("reachablePt",1,reachPtCallback);
    ros::Subscriber btIkNumSub = nh.subscribe("bestIkSoluNum",1,bestIkSoluNumCallback);

    // in rviz, "add" a "marker" and select this topic name: wsn_marker
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "sphere_list_marker", 1 );            
    visualization_msgs::Marker marker;  // instantiate a marker object
    geometry_msgs::Point point;  // points will be used to specify where the markers go
    marker.header.frame_id = "/base_link"; //base_link"; // select the reference frame 
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    // use SPHERE if you only want a single marker
    marker.type = visualization_msgs::Marker::SPHERE_LIST; //SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // if just using a single marker, specify the coordinates here, like this:

    //marker.pose.position.x = 0.4;  
    //marker.pose.position.y = -0.4;
    //marker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",marker.pose.position.x,marker.pose.position.y, marker.pose.position.z);    
    // otherwise, for a list of markers, put their coordinates in the "points" array, as below
    
    //whether a single marker or list of markers, need to specify marker properties
    // these will all be the same for SPHERE_LIST
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;


}
