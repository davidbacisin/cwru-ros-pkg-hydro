# vel_scheduler

Here are nodes that publish speed and spin values on the topic specified by the ROS param
`/vel_scheduler/cmd_vel_topic` and `/vel_scheduler/cmd_vel_stamped_topic`. 

The version "vel_scheduler" is reactive.  It ramps velocity up and down and will recover from halts.
To do so, it uses odometry info, published by Jinx (or a simulator) on the topic specified by 
ROS param `/vel_scheduler/odom_topic`. In the case of rotation, "vel_scheduler" runs until 
the angle rotated is within a small error of the angle desired. This prevents the robot 
from rotating indefinitely due to the odometry being too inaccurate for an angle difference 
of exactly zero.

"vel_scheduler" is able to make responses to LIDAR alarm, E-stop status, and a third "halt" command 
(a topic for a software-induced halt from future higher-level nodes).

Other ROS params are `/vel_scheduler/max_linear_velocity`,
`/vel_scheduler/max_linear_acceleration`, `/vel_scheduler/max_angular_velocity`,
and `/vel_scheduler/max_angular_acceleration`, which all affect what their names 
suggest.

Finally, a simplistic "triangle_vel" node was used to test Jinx's real-world 
ability to speed up and slow down. From this node, we discovered about a half-
second delay between sending Jinx a velocity and her odometry reporting that 
velocity.

## Example usage
To run the STDR simulator:  
`roslaunch cwru_376_launchers stdr_glennan_2.launch`

Then run a velocity commander, e.g.:
`roslaunch vel_scheduler vel-scheduler-stdr.launch`

Can also observe the speed commands by plotting using:
rqt_plot /robot0/cmd_vel/linear/x


    
