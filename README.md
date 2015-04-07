This it the main repository for team beta of the CWRU Mobile Robotics course, EECS 376, Spring 2015.

All packages developed by team beta can be found in \catkin\src\cwru_376_student.
The rest of the files found within this repository are being synced from the class files provided by the instructor and TA.

# CHANGELOG

The following changelog is meant to serve as a general change index,
for details on a specific file please refer to the readme found at the package level.

02/01/15 - Synced all files from class repository to create our baseline repository.

02/05/15 - Added "lidar" to packages directory. /lidar/src/ contains "lidar_alarm.cpp" and "lidar_interpreter.cpp" which were used in the reactive velocity assignment.

02/10/15 - Add "path_planner" packages to directory. /path_planner/ contains an srv folder with a matching .srv file responsible for storing the path segments while the src folder contains the actually .cpp file for the node.

02/19/15 - Added ROS params to lidar and vel_scheduler nodes. Fixed velocity ramping so that it works in the real world. This included refactoring vel_scheduler to have several functions for calculating the desired velocity, and introducing a slowdown pattern for when the lidar detects objects within several meters. 

03/09/15 - Redesigned "path_planner" to function as the desired state generator, which is responsible for feeding path segment information to the "vel_scheduler". Further improved "vel_scheduler" velocity and acceleration control to function dynamically based on both robot parameters entered in the "vel_scheduler" launch file and real world object detection.

03/19/15 - Redesigned "vel_scheduler" to use class based architecture, this has improved usability, debugging efficiency, and will make further integrations more streamlined. AMCL is also in the process of being added to the team's functionality.
