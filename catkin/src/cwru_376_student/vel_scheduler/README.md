# vel_scheduler

Here are example nodes that publish values on the "cmd_vel" topic.
The nodes are specialized to send to topic jinx/cmd_vel, which works to drive the robot "Jinx" in the lab.

The version "vel_scheduler" is reactive.  It ramps velocity up and down and will recover from halts. To do so, it uses odometry info, published by Jinx on topic /Jinx/odom.

Being able to make response to both LIDAR alarm, E-stop status and a third "halt" command (a topic for a software-induced halt from future higher-level nodes). The main function of "vel_scheduler" also subscribed another four topics, "/robot0/cmd_vel", "lidar_alarm", "lidar_dist", "estop_status".


## Example usage
To run the STDR simulator:  
'roslaunch cwru_376_launchers stdr_glennan_2.launch'
Then run a velocity commander, e.g.:
'rosrun example_robot_commander vel_scheduler'
Can also observe the speed commands by plotting using:
rqt_plot /robot0/cmd_vel/linear/x


    
