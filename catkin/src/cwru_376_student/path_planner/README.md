# Path Planner
A ROS Service which plans the path for the robot. It loads a set of points 
from the ROS Parameter Server (on `/path_planner/x_coordinates` and 
`/path_planner/y_coordinates`) and determines, from the current odometry, 
how to traverse that path. This node periodically calibrates its position 
in relative odom space using AMCL to understand the error between 
absolute map coordinates and current odom coordinates.

When a node queries this service, it receives the next path segment, 
formatted as a heading (in radians) and a length (in meters).