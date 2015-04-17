#include "object_finder.h"

// track the current process mode
ProcessMode process_mode = IDLE;

ObjectFinder::ObjectFinder(ros::NodeHandle nh): nh(nh) {
	hint_initialized = false;
	pcl_select = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>;

	initializeSubscribers();
	initializePublishers();
	initializeServices();
}

void ObjectFinder::initializeSubscribers() {
	// for live stream from Kinect
	// pclPoints = nh.subscribe<sensor_msgs::PointCloud2>("/kinect/depth/points", 1, &ObjectFinder::kinectCB, this);

	// the selected points from rviz
	selectedPoints = nh.subscribe<sensor_msgs::PointCloud2>("/selected_points", 1, &ObjectFinder::selectCB, this);
}

void ObjectFinder::initializePublishers() {
	// to display in rviz
	pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/object_finder_model", 1);
	pubPcdCloud = nh.advertise<sensor_msgs::PointCloud2>("/kinect_pointcloud", 1);
}

void ObjectFinder::initializeServices() {
	service = nh.advertiseService("process_mode", &ObjectFinder::modeCB, this);
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
	process_mode = (ProcessMode) request.req;
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

void ObjectFinder::setObjectModel(pcl::SampleConsensusModelFromNormals<pcl::PointXYZ, pcl::Normal>::Ptr& model) {
	// this is just a setter method	
	object_model = model;
}

pcl::ModelCoefficients::Ptr ObjectFinder::find(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud) {
	//const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud = object_model->getInputCloud();
	// compute and set the normals
	pcl::PointCloud<pcl::Normal>::Ptr input_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	normal_estimator.setKSearch(10);
	normal_estimator.setInputCloud(input_cloud);
	normal_estimator.compute(*input_normals);
	// initialize the algorithm
	//pcl::RandomSampleConsensus<pcl::Normal> ransac(object_model);
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	// set the maximum allowed distance to the model
	seg.setOptimizeCoefficients(true);
	seg.setDistanceThreshold(0.01);
	seg.setMaxIterations(100);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setInputCloud(input_cloud);
	seg.setInputNormals(input_normals);
	// go
	//ransac.computeModel();
	pcl::PointIndices::Ptr inliers_object(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_object(new pcl::ModelCoefficients);
	seg.segment(*inliers_object, *coefficients_object);
	// return the point cloud
	std::vector<int> inliers;
	//ransac.getInliers(inliers);

	// copy the inliers to a point cloud to display in rviz
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*input_cloud, inliers_object->indices, *inlier_cloud);
	// publish
	pubCloud.publish(inlier_cloud);

	// return the model coefficients
	//Eigen::VectorXf coeff;
	//ransac.getModelCoefficients(coeff);
	return coefficients_object;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "object_finder");
	ros::NodeHandle nh;

	ROS_INFO("Starting object_finder");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_disk(new pcl::PointCloud<pcl::PointXYZ>);

	// load a point cloud from file
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud_from_disk) == -1) { 
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	ROS_INFO("Loaded %d data points from test_pcd.pcd", cloud_from_disk->width * cloud_from_disk->height);
	// set the cloud reference frame
	cloud_from_disk->header.frame_id = "world";

	ObjectFinder finder(nh);

	while(ros::ok()) {
		switch(process_mode) {
			case FIND_CAN:{
				// reduce the amount of data
				//std::vector<int> segment_indices = finder.segmentNearHint(cloud_from_disk, 1.0);
				
				//if (segment_indices.size()){
					// load the can model
					//pcl::SampleConsensusModelFromNormals<pcl::PointXYZ, pcl::Normal>::Ptr can(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>(cloud_from_disk, segment_indices));

					//finder.setObjectModel(can);

					
					// tell it to go!
					pcl::ModelCoefficients::Ptr coeff = finder.find(cloud_from_disk);
					
					ROS_INFO("Found a can at %f; %d", coeff->values[0], coeff->values.size());
				//}
				break;
			}
			case IDLE:
			default:
				break;
		}
		
		finder.pubPcdCloud.publish(*cloud_from_disk);

		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}
}
