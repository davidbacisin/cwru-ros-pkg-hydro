// reachablePtMarker_example.cpp
// Wyatt Newman, demo how to place reachablePtMarkers in rviz
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
    ros::Rate timer(1); //timer to run at 10 Hz
    //two subscribers
    // ros::Subscriber rechPtSub = nh.subscribe("reachablePt",1,reachPtCallback);
    // ros::Subscriber btIkNumSub = nh.subscribe("bestIkSoluNum",1,bestIkSoluNumCallback);

    // in rviz, "add" a "reachablePtMarker" and select this topic name: wsn_reachablePtMarker
    ros::Publisher vis_pub_1 = nh.advertise<visualization_msgs::Marker>("reachable_point_marker", 1);            
    visualization_msgs::Marker reachablePtMarker;  // instantiate a reachablePtMarker object
    geometry_msgs::Point reachablePoint;  // points will be used to specify where the reachablePtMarkers go
    reachablePoint.x = 0.0;
    reachablePoint.y = 0.0;
    reachablePoint.z = 0.0;

    // header for time/frame information
    reachablePtMarker.header.frame_id = "/base_link"; //base_link"; // select the reference frame 
    reachablePtMarker.header.stamp = ros::Time();
    // Namespace to place this object in... used in conjunction with id to create a unique name for the object  
    reachablePtMarker.ns = "my_namespace";
    // object ID useful in conjunction with the namespace for manipulating and deleting the object later
    reachablePtMarker.id = 0;
    // use SPHERE if you only want a single reachablePtMarker
    reachablePtMarker.type = visualization_msgs::Marker::SPHERE_LIST; // Type of the object: SPHERE LIST; Or just type in the corresponding number
    reachablePtMarker.action = visualization_msgs::Marker::ADD; // 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
    // if just using a single reachablePtMarker, specify the coordinates here, like this:

    //reachablePtMarker.pose.position.x = 0.4;  
    //reachablePtMarker.pose.position.y = -0.4;
    //reachablePtMarker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",reachablePtMarker.pose.position.x,reachablePtMarker.pose.position.y, reachablePtMarker.pose.position.z);    
    
    // otherwise, for a list of reachablePtMarkers, put their coordinates in the "points" array, as below
    //whether a single reachablePtMarker or list of reachablePtMarkers, need to specify reachablePtMarker properties
    // these will all be the same for SPHERE_LIST
    reachablePtMarker.pose.orientation.x = 0.0;
    reachablePtMarker.pose.orientation.y = 0.0;
    reachablePtMarker.pose.orientation.z = 0.0;
    reachablePtMarker.pose.orientation.w = 1.0;
    reachablePtMarker.scale.x = 0.05;
    reachablePtMarker.scale.y = 0.05;
    reachablePtMarker.scale.z = 0.05;
    reachablePtMarker.color.a = 1.0;
    reachablePtMarker.color.r = 1.0;
    reachablePtMarker.color.g = 0.0;
    reachablePtMarker.color.b = 0.0;
    ROS_INFO("for here");

    // allow callbacks to populate fresh data
    // ros::spinOnce();
    reachablePtMarker.points.clear();
    // read in a fil
    std::ifstream inFile_1;
    inFile_1.open("reachableMktPtPos.txt");
    // check error for opening a file to read
    // if (inFile_1.fail()) {
    //     std::cerr << "Error Opening File" << std::endl;
    //     // exit() exits your entire program, and reports back the argument you pass it. 
    //     // This allows any programs that are running your program to figure out why it exited incorrectly. 
    //     // (1 could mean failure to connect to a database, 2 could mean unexpected arguments, etc).
    //     exit(1);
    // }
    std::string line_1;
    if (inFile_1.is_open()) {
        while(std::getline(inFile_1, line_1) && ros::ok()) {
            switch (line_1[0]) {
                case 'x':
                    reachablePoint.x = std::stod(line_1.substr(3));
                    break;
                case 'y':
                    reachablePoint.y = std::stod(line_1.substr(3));
                    break;
                case 'z':
                    reachablePoint.z = std::stod(line_1.substr(3));
                    reachablePtMarker.points.push_back(reachablePoint);
                    break;
            }
        }
        inFile_1.close(); // always need this after reading in all data
    }
    else std::cout << "Unable to open file";
    


    ros::Publisher vis_pub_2 = nh.advertise<visualization_msgs::Marker>("unreachable_point_marker", 1);            
    visualization_msgs::Marker unreachablePtMarker;  // instantiate a unreachablePtMarker object
    geometry_msgs::Point unreachablePoint;  // points will be used to specify where the unreachablePtMarkers go
    unreachablePoint.x = 0.0;
    unreachablePoint.y = 0.0;
    unreachablePoint.z = 0.0;

    // header for time/frame information
    unreachablePtMarker.header.frame_id = "/base_link"; //base_link"; // select the reference frame 
    unreachablePtMarker.header.stamp = ros::Time();
    // Namespace to place this object in... used in conjunction with id to create a unique name for the object  
    unreachablePtMarker.ns = "my_namespace";
    // object ID useful in conjunction with the namespace for manipulating and deleting the object later
    unreachablePtMarker.id = 1;
    // use SPHERE if you only want a single unreachablePtMarker
    unreachablePtMarker.type = visualization_msgs::Marker::SPHERE_LIST; // Type of the object: SPHERE LIST; Or just type in the corresponding number
    unreachablePtMarker.action = visualization_msgs::Marker::ADD; // 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
    // if just using a single unreachablePtMarker, specify the coordinates here, like this:

    //unreachablePtMarker.pose.position.x = 0.4;  
    //unreachablePtMarker.pose.position.y = -0.4;
    //unreachablePtMarker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",unreachablePtMarker.pose.position.x,unreachablePtMarker.pose.position.y, unreachablePtMarker.pose.position.z);    
    
    // otherwise, for a list of unreachablePtMarkers, put their coordinates in the "points" array, as below
    //whether a single unreachablePtMarker or list of unreachablePtMarkers, need to specify unreachablePtMarker properties
    // these will all be the same for SPHERE_LIST
    unreachablePtMarker.pose.orientation.x = 0.0;
    unreachablePtMarker.pose.orientation.y = 0.0;
    unreachablePtMarker.pose.orientation.z = 0.0;
    unreachablePtMarker.pose.orientation.w = 1.0;
    unreachablePtMarker.scale.x = 0.05;
    unreachablePtMarker.scale.y = 0.05;
    unreachablePtMarker.scale.z = 0.05;
    unreachablePtMarker.color.a = 1.0;
    unreachablePtMarker.color.r = 0.0;
    unreachablePtMarker.color.g = 0.0;
    unreachablePtMarker.color.b = 1.0;
    ROS_INFO("for here");

    // allow callbacks to populate fresh data
    // ros::spinOnce();
    unreachablePtMarker.points.clear();
    // read in a fil
    std::ifstream inFile_2;
    inFile_2.open("unreachableMktPtPos.txt");
    // check error for opening a file to read
    // if (inFile_2.fail()) {
    //     std::cerr << "Error Opening File" << std::endl;
    //     // exit() exits your entire program, and reports back the argument you pass it. 
    //     // This allows any programs that are running your program to figure out why it exited incorrectly. 
    //     // (1 could mean failure to connect to a database, 2 could mean unexpected arguments, etc).
    //     exit(1);
    // }
    std::string line_2;
    if (inFile_2.is_open()) {
        while(std::getline(inFile_2, line_2) && ros::ok()) {
            switch (line_2[0]) {
                case 'x':
                    unreachablePoint.x = std::stod(line_2.substr(3));
                    break;
                case 'y':
                    unreachablePoint.y = std::stod(line_2.substr(3));
                    break;
                case 'z':
                    unreachablePoint.z = std::stod(line_2.substr(3));
                    unreachablePtMarker.points.push_back(unreachablePoint);
                    break;
            }
        }
        inFile_2.close(); // always need this after reading in all data
    }
    else std::cout << "Unable to open file";
    while(ros::ok()) {
        ROS_INFO("publishing reachable point marker");
        vis_pub_1.publish(reachablePtMarker);
        ROS_INFO("publishing unreachable point marker");
        vis_pub_2.publish(unreachablePtMarker);
        timer.sleep();
    }
    return 0;
}
