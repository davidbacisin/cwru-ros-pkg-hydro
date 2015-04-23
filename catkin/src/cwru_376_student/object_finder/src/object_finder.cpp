#include "object_finder.h"

#define CAN_HEIGHT	0.25

// track the current process mode
ProcessMode process_mode = IDLE;

ObjectFinder::ObjectFinder(ros::NodeHandle nh): nh(nh) {
	hint_initialized = false;
	pcl_select = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>;

	kinect_initialized = false;
	pcl_kinect = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>;

	initializeSubscribers();
	initializePublishers();
	initializeServices();
}

void ObjectFinder::initializeSubscribers() {
	// for live stream from Kinect
	pclPoints = nh.subscribe<sensor_msgs::PointCloud2>("/kinect/depth/points", 1, &ObjectFinder::kinectCB, this);

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
	seg.setRadiusLimits(0.02, 0.09);

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
	float diff = (can_position.dot(can_axis) - inlier_centroid.dot(can_axis));
	ROS_INFO("Diff: %f", diff);
	can_position -= (can_position.dot(can_axis) - inlier_centroid.dot(can_axis)) * can_axis;
	trx.translate(can_position);
	
	// rotate to proper orientation
	Eigen::Quaternionf rotator;
	rotator.setFromTwoVectors(Eigen::Vector3f::UnitZ(), can_axis);
	trx.rotate(rotator);

	pcl::transformPointCloud(*can_cloud, *transformed_cloud, trx);
	
	// metadata
	transformed_cloud->header = input_cloud->header;
	transformed_cloud->header.frame_id = "kinect_pc_frame";
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
	normal_estimator.setKSearch(10);
	normal_estimator.setInputCloud(input_cloud);
	normal_estimator.compute(*input_normals);
	// initialize the algorithm
	seg.setOptimizeCoefficients(true);
	seg.setDistanceThreshold(0.01);
	seg.setMaxIterations(1000);
	seg.setMethodType(pcl::SAC_RANSAC);
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

	ObjectFinder finder(nh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// whether or not to use the search cloud instead of the live stream
	bool use_search_cloud = false;

	while(ros::ok()) {
		switch(process_mode) {
			case HINT: {
				// reduce the amount of data to near the hint
				std::vector<int> segment_indices = finder.segmentNearHint(original_cloud, 0.25);
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
