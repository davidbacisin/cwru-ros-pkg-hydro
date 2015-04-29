# Object Finder
Object Finder takes point cloud data and estimates the position of an object
within that cloud. It offers a variety of processing modes to assist with 
object finding. To run, `rosrun object_finder object_finder` and then 
perform `rosservice call process_mode X`, replacing `X` with the desired 
process mode as described below.

## Process Modes
0. Idle. Do nothing.

1. Automatic can finder which performs the following process, unaided:
From the set of points near table height (between 
0.6 and 1.2 meters from the floor), use RANSAC to find a plane. This plane 
is presumably a table. Then, reduce the search space to points above the table 
and no more than 5 centimeters above the table. Additionally, since the robot 
arm can only reach so far, ignore points that are further than 1.2 meters from 
the robot base. Using the centroid of the 
remaining points as an initial guess, refine the (x, y) position of the can 
using a gradient descent algorithm. Publish on `can_top_position` the 
topmost point on the center axis of the can, and publish the estimated 
can model on `/object_finder/computed_model`.

2. Automatic RANSAC-based can finder. From the search space (whether set using 
process mode 4 or otherwise the entire data set), use RANSAC to find the best 
fit for a cylinder. This does not work very well on noisy data. Publish the 
estimated model on `/object_finder/computed_model`.

3. Automatic RANSAC-based table finder. From the search space, use RANSAC 
to find the best fit for a plane. Published the estimated model on 
`/object_finder/computed_model`.

4. Provide a hint. Before calling this mode, select and publish points in 
RViz near the desired search area. By calling this process mode, `object_finder` 
will reduce the search space to a 0.1 meter-radius sphere centered at the 
centroid of the selected points. This reduces the number of outliers which 
could reduce the accuracy of RANSAC-based algorithms.

5. Update vision data. If using process modes 2, 3, or 4, call process mode 5 
first! This stores fresh data from the Kinect in the internal buffer which will
 be used to perform the other processing.
