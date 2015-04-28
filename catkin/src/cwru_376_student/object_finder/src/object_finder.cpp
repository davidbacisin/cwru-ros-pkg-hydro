#include "object_finder.h"

#define CAN_HEIGHT	0.12
#define CAN_RADIUS	0.035

// track the current process mode
ProcessMode process_mode = IDLE;

ObjectFinder::ObjectFinder(ros::NodeHandle nh): nh(nh) {
	hint_initialized = false;
	pcl_select = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>;

	kinect_initialized = false;
	pcl_kinect = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>;

	vertical_initialized = false;

	initializeSubscribers();
	initializePublishers();
	initializeServices();
}

void ObjectFinder::initializeSubscribers() {
	// for live stream from Kinect
	pclPoints = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &ObjectFinder::kinectCB, this);

	// the selected points from rviz
	selectedPoints = nh.subscribe<sensor_msgs::PointCloud2>("/selected_points", 1, &ObjectFinder::selectCB, this);
}

void ObjectFinder::initializePublishers() {
	// to display in rviz
	pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/object_finder/computed_model", 1);
	pubPcdCloud = nh.advertise<sensor_msgs::PointCloud2>("/kinect_pointcloud", 1);
}

void ObjectFinder::initializeServices() {
	service = nh.advertiseService("process_mode", &ObjectFinder::modeCB, this);
}

void ObjectFinder::selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
	pcl::fromROSMsg(*cloud, *pcl_select);
	//ROS_INFO("frame id is: %s",cloud->header.frame_id);
	int npts = pcl_select->width * pcl_select->height;
	
	// compute and save the centroid
	hint_point = computeCentroid(pcl_select);

	ROS_INFO("Received new patch with centroid at (%f, %f, %f)", hint_point.x(), hint_point.y(), hint_point.z());

	hint_initialized = true;
}

void ObjectFinder::kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
	kinect_raw = cloud;
	pcl::fromROSMsg(*cloud, *pcl_kinect);
	// ROS_INFO("Received data from kinect");
	kinect_initialized = true;
}

bool ObjectFinder::modeCB(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response) {
	response.resp = true;
	process_mode = (ProcessMode) request.req;
	ROS_INFO("Process mode selected: %d", process_mode);
	return true;
}

Eigen::Vector3f ObjectFinder::computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	Eigen::Vector3f centroid;
	centroid << 0, 0, 0; // initialize

	int size = cloud->width * cloud->height;
	for (int i=0; i < size; i++) {
		centroid += cloud->points[i].getVector3fMap();
	}
	if (size > 0) {
		centroid /= ((float) size);
	}
	return centroid;
}
Eigen::Vector3f ObjectFinder::computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<int> indices) {
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;

    int size = indices.size();
    for (int i = 0; i < size; i++) {
        centroid += cloud->points[indices[i]].getVector3fMap();
    }
    if (size > 0) {
        centroid /= ((float) size);
    }
    return centroid;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectFinder::getCanCloud(float radius, float height) {
	double theta, h;
	int i, npts = 0;
    	Eigen::Vector3f pt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr can_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// count the points
	for (theta = 0; theta < 2.0*M_PI; theta += 0.3)
		for (h = 0; h < CAN_HEIGHT; h += 0.01)  
			npts++;
	// resize the cloud
	can_cloud->points.resize(npts);
	// add the points
	for (i = 0, theta = 0; theta < 2.0*M_PI; theta += 0.3) {
		for (h=0; h < height; i++, h += 0.01) {
			// radius = coeff->values[6]
			pt[0] = radius * cos(theta);
			pt[1] = radius * sin(theta);
			pt[2] = h;
			can_cloud->points[i].getVector3fMap() = pt;
		}
	}

	// metadata
	can_cloud->is_dense = true;
	can_cloud->width = npts;
	can_cloud->height = 1;
	
	return can_cloud;
}

std::vector<int> ObjectFinder::segmentNearHint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius) {
	std::vector<int> indices(0);
	// segment the scene based on the input
	if (!hint_initialized) {
		ROS_WARN("Select some hint points in RViz first!");
		return indices;
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> segmenter;
	std::vector<float> sqr_distances;
	pcl::PointXYZ pcl_hint(hint_point.x(), hint_point.y(), hint_point.z());

	segmenter.setInputCloud(cloud);
	segmenter.radiusSearch(pcl_hint, radius, indices, sqr_distances);
	return indices;
}

pcl::ModelCoefficients::Ptr ObjectFinder::findCan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud) {
	// tell the segmenter what to find	
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setRadiusLimits(0.031, 0.035);
	// seg.setSamplesMaxDist(0.2);
	if (vertical_initialized) {
		seg.setAxis(vertical_axis);
		seg.setEpsAngle(0.001); // really small angle
	}

	Eigen::Vector3f inlier_centroid;
	pcl::ModelCoefficients::Ptr coeff = find(input_cloud, &inlier_centroid);

	// create a point cloud to display as the can
	double theta,h;
	int i, npts = 0;
    	Eigen::Vector3f pt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr can_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// count the points
	for (theta = 0; theta < 2.0*M_PI; theta += 0.3)
		for (h = 0; h < CAN_HEIGHT; h += 0.01)  
			npts++;
	// resize the cloud
	can_cloud->points.resize(npts);
	// add the points
	for (i = 0, theta = 0; theta < 2.0*M_PI; theta += 0.3) {
		for (h=0; h < CAN_HEIGHT; i++, h += 0.01) {
			// radius = coeff->values[6]
			pt[0] = coeff->values[6] * cos(theta);
			pt[1] = coeff->values[6] * sin(theta);
			pt[2] = h - CAN_HEIGHT/2.0;
			can_cloud->points[i].getVector3fMap() = pt;
		}
	}
	
	// transform to the appropriate location
  	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Affine3f trx = Eigen::Affine3f::Identity();
	
	// the coefficients will get us on the same axis of the cylinder, but not necessarily matching position along cylinder
	Eigen::Vector3f can_position(coeff->values[0] , coeff->values[1], coeff->values[2]),
		can_axis(-coeff->values[3], -coeff->values[4], -coeff->values[5]);
	// shift the can along the axis to the correct position
	//float diff = (can_position.dot(can_axis) - inlier_centroid.dot(can_axis));
	//ROS_INFO("Diff: %f", diff);
	can_position -= (can_position.dot(can_axis) - inlier_centroid.dot(can_axis)) * can_axis;
	trx.translate(can_position);
	
	// rotate to proper orientation
	Eigen::Quaternionf rotator;
	rotator.setFromTwoVectors(Eigen::Vector3f::UnitZ(), can_axis);
	trx.rotate(rotator);

	pcl::transformPointCloud(*can_cloud, *transformed_cloud, trx);
	
	// metadata
	transformed_cloud->header = input_cloud->header;
	transformed_cloud->header.frame_id = "camera_depth_optical_frame";
	transformed_cloud->header.stamp = ros::Time::now().toSec() * 1e6;
	transformed_cloud->is_dense = true;
	transformed_cloud->width = npts;
	transformed_cloud->height = 1;
	
	// publish
	pubCloud.publish(transformed_cloud);

	return coeff;
}

pcl::ModelCoefficients::Ptr ObjectFinder::findTable(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud) {
	// tell the segmenter what to find	
	seg.setModelType(pcl::SACMODEL_PLANE);
	//seg.setAxis(Eigen::Vector3f::UnitZ());

	Eigen::Vector3f inlier_centroid;
	pcl::ModelCoefficients::Ptr coeff = find(input_cloud, &inlier_centroid);

	// create a point cloud to display as the table
	float x, y;
	int i, npts = 100;
    	Eigen::Vector3f pt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// resize the cloud
	table_cloud->points.resize(npts);
	// add the points
	for (i = 0, x = -0.5; x < 0.5; x += 0.1) {
		for (y = -0.5; y < 0.5; i++, y += 0.1 ) {
			pt[0] = x;
			pt[1] = y;
			pt[2] = 0.0;
			table_cloud->points[i].getVector3fMap() = pt;
		}
	}
	
	// transform to the appropriate location
  	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Affine3f trx = Eigen::Affine3f::Identity();
	
	// the coefficients will get us on the same axis of the cylinder, but not necessarily matching position along cylinder
	Eigen::Vector3f table_position(inlier_centroid),
		table_axis(-coeff->values[0], -coeff->values[1], -coeff->values[2]);
	// shift the can along the axis to the correct position
	//float diff = (table_position.dot(table_axis) - inlier_centroid.dot(table_axis));
	//ROS_INFO("Diff: %f", diff);
	//can_position -= (can_position.dot(table_axis) - inlier_centroid.dot(table_axis)) * table_axis;
	trx.translate(table_position);
	
	// rotate to proper orientation
	Eigen::Quaternionf rotator;
	rotator.setFromTwoVectors(Eigen::Vector3f::UnitZ(), table_axis);
	trx.rotate(rotator);

	pcl::transformPointCloud(*table_cloud, *transformed_cloud, trx);

	// remember the vertical axis of the world
	vertical_axis = table_axis;
	vertical_initialized = true;
	
	// metadata
	transformed_cloud->header = input_cloud->header;
	transformed_cloud->header.frame_id = "camera_depth_optical_frame";
	transformed_cloud->header.stamp = ros::Time::now().toSec() * 1e6;
	transformed_cloud->is_dense = true;
	transformed_cloud->width = npts;
	transformed_cloud->height = 1;
	
	// publish
	pubCloud.publish(transformed_cloud);

	return coeff;
}


pcl::ModelCoefficients::Ptr ObjectFinder::find(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, Eigen::Vector3f *centroid) {
	pcl::PointCloud<pcl::Normal>::Ptr input_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	pcl::PointIndices::Ptr inliers_object(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_object(new pcl::ModelCoefficients);
	// compute the normals
	normal_estimator.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	normal_estimator.setKSearch(50);
	normal_estimator.setInputCloud(input_cloud);
	normal_estimator.compute(*input_normals);
	// initialize the algorithm
	seg.setOptimizeCoefficients(true);
	seg.setDistanceThreshold(0.01);
	seg.setMaxIterations(1000);
	seg.setMethodType(pcl::SAC_PROSAC);
	// set the data
	seg.setInputCloud(input_cloud);
	seg.setInputNormals(input_normals);
	// go
	seg.segment(*inliers_object, *coefficients_object);

	if (centroid) {
		*centroid = computeCentroid(input_cloud, inliers_object->indices);
	}

	return coefficients_object;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "object_finder");
	ros::NodeHandle nh;

	ROS_INFO("Starting object_finder");

	pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// load a point cloud from file
	/*
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *original_cloud) == -1) { 
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	ROS_INFO("Loaded %d data points from test_pcd.pcd", original_cloud->width * original_cloud->height);
	// set the cloud reference frame
	original_cloud->header.frame_id = "base_link";
	*/

	// so we can transform from kinect to odom space
	bool tf_initialized = false;
	tf::TransformListener *tf_p = new tf::TransformListener(nh);
	tf::StampedTransform odom_wrt_kinect;
	while (!tf_initialized && ros::ok()) {
		try {
			tf_p->lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), odom_wrt_kinect);
			tf_initialized = true;
		}
		catch (tf::TransformException& e) {
			ROS_WARN("%s", e.what());
			tf_initialized = false;
			ros::spinOnce();
			ros::Duration(0.5).sleep();
		}
	}

	ObjectFinder finder(nh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// whether or not to use the search cloud instead of the live stream
	bool use_search_cloud = false;

	while(ros::ok()) {
		switch(process_mode) {
			case HINT: {
				// reduce the amount of data to near the hint
				std::vector<int> segment_indices = finder.segmentNearHint(original_cloud, 0.1);
				// will be zero if the hint point was not specified
				if (segment_indices.size()){
					search_cloud->clear();
					pcl::copyPointCloud<pcl::PointXYZ>(*original_cloud, segment_indices, *search_cloud);
					use_search_cloud = true;
				}
				ROS_INFO("Using hint data");
				// reset state variables
				process_mode = IDLE;
				break;
			}
			case FIND_CAN:{
				// make sure we have data
				if (!use_search_cloud){
					ROS_INFO("No search cloud specified; using original cloud");
					pcl::copyPointCloud<pcl::PointXYZ>(*original_cloud, *search_cloud);
				}
				
				pcl::ModelCoefficients::Ptr coeff = finder.findCan(search_cloud);
					
				ROS_INFO("Found a can at (%f, %f, %f) angle (%f, %f, %f) with radius %f", coeff->values[0], coeff->values[1], coeff->values[2],
					coeff->values[3], coeff->values[4], coeff->values[5], coeff->values[6]);
				// reset state variables
				use_search_cloud = false;
				process_mode = IDLE;
				break;
			}
			case FIND_CAN2:{
				// transform the point cloud
				pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				Eigen::Affine3d kinect_to_odom;
				tf::transformTFToEigen(odom_wrt_kinect, kinect_to_odom);
				pcl::transformPointCloud(*finder.pcl_kinect, *transformed_cloud, kinect_to_odom);

				// filter the cloud along the z-axis to only keep stuff near table height
				pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PassThrough<pcl::PointXYZ> filter;
				filter.setInputCloud(transformed_cloud);
				filter.setFilterFieldName("z");
				filter.setFilterLimits(0.6, 1.2);
				filter.filter(*filtered_cloud);
				
				// metadata
				filtered_cloud->header.frame_id = "base_link"; // remember we transformed it!
				filtered_cloud->header.stamp = ros::Time::now().toSec() * 1e6;
				finder.pubCloud.publish(filtered_cloud);
				ros::Duration(1.0).sleep();
				
				// find the table using PROSAC segmentation
				pcl::SACSegmentation<pcl::PointXYZ> seg;
				pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices);
				pcl::ModelCoefficients::Ptr table_coefficients(new pcl::ModelCoefficients);
				seg.setOptimizeCoefficients(true);
				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setDistanceThreshold(0.01);
				seg.setMaxIterations(100);
				seg.setMethodType(pcl::SAC_PROSAC);	
				seg.setInputCloud(filtered_cloud);
				seg.segment(*table_inliers, *table_coefficients);

				// remove the table inliers
				pcl::ExtractIndices<pcl::PointXYZ> remover;
				remover.setInputCloud(filtered_cloud);
				remover.setIndices(table_inliers);
				remover.setNegative(true); // remove specified indices
				remover.filter(*filtered_cloud);

				// remove points below table
				filter.setInputCloud(filtered_cloud);
				filter.setFilterFieldName("z");
				double table_height = -table_coefficients->values[3]/table_coefficients->values[2];
				filter.setFilterLimits(table_height + 0.02, table_height + 0.05);
				filter.filter(*filtered_cloud);

				finder.pubCloud.publish(filtered_cloud);
				ros::Duration(1.0).sleep();

				// estimate the center based on the centroid
				Eigen::Vector4f can_center_4f; // compute3DCentroid requires a 4-vector for some reason
				pcl::compute3DCentroid(*filtered_cloud, can_center_4f);
				// can has to be on the table
				can_center_4f.z() = table_height;

				// refine the center with a limited number of iterations
				double r_sqd,
					r_err = 1.0, // make it run at least once
					dEdx,
					dEdy;
				Eigen::Vector3f can_center = can_center_4f.head<3>(),
					diff;
				for (int iter = 0; iter < 10 && fabs(r_err) > 0.02; iter++) {
					// reset					
					r_err = 0.0;
					dEdx = 0.0;
					dEdy = 0.0;
					// calculate the error of the guess
					for (int i=0; i < filtered_cloud->points.size(); i++) {
						diff = can_center - filtered_cloud->points[i].getVector3fMap();
						r_sqd = diff.x() * diff.x() + diff.y() * diff.y();
						r_err += r_sqd - CAN_RADIUS * CAN_RADIUS;
						dEdx += (r_sqd - CAN_RADIUS * CAN_RADIUS) * diff.x() / r_sqd;
						dEdy += (r_sqd - CAN_RADIUS * CAN_RADIUS) * diff.y() / r_sqd;
					}
					dEdx /= filtered_cloud->points.size();
					dEdy /= filtered_cloud->points.size();
					
					// step toward minimum
					can_center.x() -= dEdx;
					can_center.y() -= dEdy;
				}
				
				// create a model of the can
				pcl::PointCloud<pcl::PointXYZ>::Ptr can_cloud(finder.getCanCloud(CAN_RADIUS, CAN_HEIGHT));
				can_cloud->header.frame_id = "base_link";
			
				// translate and display
				pcl::PointCloud<pcl::PointXYZ>::Ptr display_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				Eigen::Affine3f transformer = Eigen::Affine3f::Identity();
				transformer.translate(can_center);
				pcl::transformPointCloud(*can_cloud, *display_cloud, transformer);

				// metadata
				display_cloud->header.frame_id = "base_link"; // remember we transformed it!
				display_cloud->header.stamp = ros::Time::now().toSec() * 1e6;
	
				// publish
				finder.pubCloud.publish(display_cloud);
				ROS_INFO("Can top at (%f, %f, %f)", can_center.x(), can_center.y(), can_center.z() + CAN_HEIGHT);

				// reset state variables
				process_mode = IDLE;
				break;
			}
			case FIND_TABLE:{
				// make sure we have data
				if (!use_search_cloud){
					ROS_INFO("No search cloud specified; using original cloud");
					pcl::copyPointCloud<pcl::PointXYZ>(*original_cloud, *search_cloud);
				}
				
				pcl::ModelCoefficients::Ptr coeff = finder.findTable(search_cloud);
					
				ROS_INFO("Found a table with orientation (%f, %f, %f)", coeff->values[0], coeff->values[1], coeff->values[2]);
				// reset state variables
				use_search_cloud = false;
				process_mode = IDLE;
				break;
			}
			case UPDATE_VISION:
				if (finder.kinect_initialized) {
					ROS_INFO("Using freshest kinect data");
					pcl::copyPointCloud<pcl::PointXYZ>(*finder.pcl_kinect, *original_cloud);
					pcl::copyPointCloud<pcl::PointXYZ>(*original_cloud, *search_cloud);
				}
				// reset state variables
				use_search_cloud = false;
				process_mode = IDLE;
				break;
			case IDLE:
			default:
				break;
		}
		
		finder.pubPcdCloud.publish(*search_cloud);

		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}
}
