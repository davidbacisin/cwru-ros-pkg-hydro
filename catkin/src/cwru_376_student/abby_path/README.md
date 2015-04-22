# Abby Path Planner
A ROS Service which plans the path for the robot Abby. When the rosservice 
`abby_path_command` is called with value `1`, Abby will move forward to the current 
interactive marker position. If called with `-1`, Abby will move backward to 
the current interactive marker position.

The node determines, from the current odometry, 
how to traverse the space to the new position.

When a node queries this service, it receives the next path segment, 
formatted as a heading (in radians) and a length (in meters).
