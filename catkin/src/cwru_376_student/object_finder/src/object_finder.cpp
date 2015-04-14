#include "object_finder.h"

// track the current process mode
ProcessMode process_mode = IDLE;

ObjectFinder::ObjectFinder(ros::NodeHandle nh): nh(nh) {
	hint_initialized = false;

	initializeSubscribers();
	initializePublishers();
	initializeServices();
}

void ObjectFinder::initializeSubscribers() {
	// for live stream from Kinect
	// pclPoints = nh.subscribe<sensor_msgs::PointCloud2>("/kinect/depth/points", 1, &ObjectFinder::kinectCB, this);

	// the selected points from rviz
	selectedPoints = nh.subscribe<sensor_msgs::PointCloud2>("/selected_Points", 1, &ObjectFinder::selectCB, this);
}

void ObjectFinder::initializePublishers() {
	// to display in rviz
	pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/plane_model", 1);
	pubPcdCloud = nh.advertise<sensor_msgs::PointCloud2>("/kinect_pointcloud", 1);
}

void ObjectFinder::initializeServices() {
	ros::ServiceServer service = nh.advertiseService("process_mode", &ObjectFinder::modeCB, this);
}

void ObjectFinder::selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
	pcl::fromROSMsg(*cloud, *pcl_select);
	ROS_INFO("RECEIVED NEW PATCH w/  %d * %d points", pcl_select->width, pcl_select->height);
	//ROS_INFO("frame id is: %s",cloud->header.frame_id);
	int npts = pcl_select->width * pcl_select->height;
	
	// compute and save the centroid
	hint_point = computeCentroid(pcl_select);
	hint_initialized = true;
}

bool ObjectFinder::modeCB(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response) {
	response.resp = true;
	process_mode = request.req;
	ROS_INFO("Process mode selected: %d", process_mode);
	return true;
}

Eigen::Vector3f ObjectFinder::computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	Eigen::Vector3f centroid;
	centroid << 0, 0, 0; // initialize

	int size = cloud->width * cloud->height;
	for (size_t i=0; i < size; i++) {
		centroid += cloud->points[i].getVector3fMap();
	}
	if (size > 0) {
		centroid /= ((float) size);
	}
	return centroid;
}


std::vector<int> ObjectFinder::segmentNearHint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius) {
	// segment the scene based on the input
	if (!hint_initialized) {
		ROS_WARN("Select some hint points in RViz first!");
		break;
	}
	KdTree segmenter;
	std::vector<int> indices;
	std::vector<float> sqr_distances;

	segmenter.setInputCloud(cloud, NULL); // null says to use the whole cloud instead of specific indices
	segmenter.radiusSearch(hint_point, radius, indices, sqr_distances);
	return indices;
}

void ObjectFinder::setObjectModel(pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr& model) {
	// this is just a setter method	
	object_model = model;
}

Eigen::VectorXf ObjectFinder::find() {
	// initialize the algorithm
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(object_model);
	// set the maximum allowed distance to the model
	ransac.setDistanceThreshold(0.01);
	ransac.setMaxIterations(100);
	// go
	ransac.computeModel();
	// return the point cloud
	std::vector<int> inliers;
	ransac.getInliers(inliers);

	// copy the inliers to a point cloud to display in rviz
	const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = object_model->getInputCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud;
	pcl::copyPointCloud<pcl::PointXYZ>(*input_cloud, inliers, *inlier_cloud);
	// publish
	pubCloud.publish(inlier_cloud);

	// return the model coefficients
	Eigen::VectorXf coeff;
	ransac->getModelCoefficients(coeff);
	return coeff;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "object_finder");
	ros::NodeHandle nh;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_disk,
		cloud_segment;

	// load a point cloud from file
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud_from_disk) == -1) { 
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	ROS_INFO("Loaded %d data points from test_pcd.pcd", cloud_from_disk->width * cloud_from_disk->height);
	// set the cloud reference frame
	cloud_from_disk->header.frame_id = "base_link";

	ObjectFinder finder(nh);

	while(ros::ok()) {
		switch(process_mode) {
			case FIND_CAN:
				// reduce the amount of data
				std::vector<int> segment_indices = finder.segmentNearHint(cloud_from_disk, 1.0);
				
				// load the can model
				finder.setObjectModel(new SampleConsensusModelCylinder<pcl::PointXYZ, pcl::PointXYZ>(cloud_from_disk, segment_indices));
				
				// tell it to go!
				Eigen::VectorXf coeff = finder.find();
				
				ROS_INFO("Found a can at (%f, %f, %f)", coeff(0), coeff(1), coeff(2));
				break;
			case IDLE:
			default:
				break;
		}
	}
}
