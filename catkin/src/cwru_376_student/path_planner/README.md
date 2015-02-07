# Path Planner
A ROS Service which plans the path for the robot. Nodes can query this 
service for a specific path segment (by an integer ID), and they will 
receive a heading and a distance in response (both as floats).