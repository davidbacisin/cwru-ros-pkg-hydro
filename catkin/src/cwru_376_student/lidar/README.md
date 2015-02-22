# LIDAR nodes
This package includes the "lidar_interpreter" node, which translates raw data from the 
LIDAR into friendlier data for general use. The "lidar_interpreter" broadcasts a float 
on the ROS topic `/lidar_nearest`, representing the distance to the object nearest to 
the lidar. This node uses the ROS param `/lidar_interpreter/lidar_topic` to 
determine which topic to use for the LIDAR data.

This package also includes the "lidar_alarm" node, which broadcasts a warning on ROS 
topic `/lidar_alarm` if something gets too close to the LIDAR. This warning is a Bool 
such that `true` represents an active warning, and `false` represents no need to be 
alarmed. The distance at which the alarm first turns on can be specified in the 
ROS param, `/lidar_alarm/lidar_safe_distance`.