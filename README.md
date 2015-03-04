This it the main repository for team beta of the CWRU Mobile Robotics course, EECS 376, Spring 2015.

All packages developed by team beta can be found in \catkin\src\cwru_376_student.
The rest of the files found within this repository are being synced from the class files provided by the instructor and TA.

--CHANGELOG--

The following changelog is meant to serve as a general change index,
for details on a specific file please refer to the readme found at the package level.

2/1/2015 - Synced all files from class repository to create our baseline repository.

2/5/2015 - Added "lidar" to packages directory. /lidar/src/ contains "lidar_alarm.cpp" and "lidar_interpreter.cpp" which were used in the reactive velocity assignment.

2/10/2015 - Add path_planner packages to directory. /path_planner/ contains an srv folder with a matching .srv file responsible for storing the path segments while the src folder contains the actually .cpp file for the node.