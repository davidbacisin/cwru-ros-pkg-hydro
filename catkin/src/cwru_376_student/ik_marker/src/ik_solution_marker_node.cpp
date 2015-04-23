// ikSoluMarker_example.cpp
// Wyatt Newman, demo how to place ikSoluMarkers in rviz
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <irb120_kinematics.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <fstream>
#include <iostream>


// declaring some global variable
// geometry_msgs::Point markerPt;

// callback function
// void reachPtCallback(const geometry_msgs::Point& ptRcvd) {
//     // populate the message recieved from subscribed topic
//     markerPt.x = ptRcvd.x;
//     markerPt.y = ptRcvd.y;
//     markerPt.z = ptRcvd.z;
//     // debugging output
//     ROS_INFO("Possible marker position: x = %f, y = %f, z  = %f", markerPt.x, markerPt.y, markerPt.z);
// }

// callback function
// void bestIkSoluNumCallback(const std_msgs::Int16& bestIkSoluNumRcvd) {
//     // debugging output
//     ROS_INFO("The best IK solution No.: %i", bestIkSoluNumRcvd.data);
//     bestIkSoluNum = bestIkSoluNumRcvd.data;
// }


int main(int argc, char **argv) {
    ros::init(argc, argv, "ik_solu_marker_placer");
    ros::NodeHandle nh;
    //ROS_INFO("hello, world");
    ros::Rate timer(25); //timer to run at 10 Hz
    //two subscribers
    // ros::Subscriber rechPtSub = nh.subscribe("reachablePt",1,reachPtCallback);
    // ros::Subscriber btIkNumSub = nh.subscribe("bestIkSoluNum",1,bestIkSoluNumCallback);

    // in rviz, "add" a "ikSoluMarker" and select this topic name: wsn_ikSoluMarker
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("sphere_list_marker", 1);            
    visualization_msgs::Marker ikSoluMarker;  // instantiate a ikSoluMarker object
    geometry_msgs::Point point;  // points will be used to specify where the ikSoluMarkers go
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;

    // header for time/frame information
    ikSoluMarker.header.frame_id = "/base_link"; //base_link"; // select the reference frame 
    ikSoluMarker.header.stamp = ros::Time();
    // Namespace to place this object in... used in conjunction with id to create a unique name for the object  
    ikSoluMarker.ns = "my_namespace";
    // object ID useful in conjunction with the namespace for manipulating and deleting the object later
    ikSoluMarker.id = 0;
    // use SPHERE if you only want a single ikSoluMarker
    ikSoluMarker.type = visualization_msgs::Marker::SPHERE_LIST; // Type of the object: SPHERE LIST; Or just type in the corresponding number
    ikSoluMarker.action = visualization_msgs::Marker::ADD; // 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
    // if just using a single ikSoluMarker, specify the coordinates here, like this:

    //ikSoluMarker.pose.position.x = 0.4;  
    //ikSoluMarker.pose.position.y = -0.4;
    //ikSoluMarker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",ikSoluMarker.pose.position.x,ikSoluMarker.pose.position.y, ikSoluMarker.pose.position.z);    
    
    // otherwise, for a list of ikSoluMarkers, put their coordinates in the "points" array, as below
    //whether a single ikSoluMarker or list of ikSoluMarkers, need to specify ikSoluMarker properties
    // these will all be the same for SPHERE_LIST
    ikSoluMarker.pose.orientation.x = 0.0;
    ikSoluMarker.pose.orientation.y = 0.0;
    ikSoluMarker.pose.orientation.z = 0.0;
    ikSoluMarker.pose.orientation.w = 1.0;
    ikSoluMarker.scale.x = 0.05;
    ikSoluMarker.scale.y = 0.05;
    ikSoluMarker.scale.z = 0.05;
    ikSoluMarker.color.a = 1.0;
    ikSoluMarker.color.r = 1.0;
    ikSoluMarker.color.g = 0.0;
    ikSoluMarker.color.b = 0.0;
    ROS_INFO("for here");

    // allow callbacks to populate fresh data
    // ros::spinOnce();
    ikSoluMarker.points.clear();
    // read in a fil
    std::ifstream inFile;
    inFile.open("reachableMktPtPos.txt");
    // check error for opening a file to read
    // if (inFile.fail()) {
    //     std::cerr << "Error Opening File" << std::endl;
    //     // exit() exits your entire program, and reports back the argument you pass it. 
    //     // This allows any programs that are running your program to figure out why it exited incorrectly. 
    //     // (1 could mean failure to connect to a database, 2 could mean unexpected arguments, etc).
    //     exit(1);
    // }
    std::string line;
    if (inFile.is_open()) {
        while(std::getline(inFile, line) && ros::ok()) {
            switch (line[0]) {
                case 'x':
                    point.x = std::stod(line.substr(3));
                    break;
                case 'y':
                    point.y = std::stod(line.substr(3));
                    break;
                case 'z':
                    point.z = std::stod(line.substr(3));
                    ikSoluMarker.points.push_back(point);
                    ROS_INFO("publishing Maker Point...");
                    vis_pub.publish(ikSoluMarker);
                    timer.sleep();
                    break;
            }
        }
        inFile.close(); // always need this after reading in all data
    }
    else std::cout << "Unable to open file";
    return 0;
}
