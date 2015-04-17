#ifndef _OBJECT_FINDER_H_
#define _OBJECT_FINDER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/PointIndices.h>
//#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cwru_srv/simple_int_service_message.h> // for the process mode service
#include <Eigen/Eigen>

enum ProcessMode {
	IDLE,
	FIND_CAN
};

class ObjectFinder {
public:
	ObjectFinder(ros::NodeHandle);

	std::vector<int> segmentNearHint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius);
	void setObjectModel(pcl::SampleConsensusModelFromNormals<pcl::PointXYZ, pcl::Normal>::Ptr& model);
	pcl::ModelCoefficients::Ptr find(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud);
	
	// make public so external functions can use the publisher
	ros::Publisher pubCloud,
		pubPcdCloud;	
private:
	ros::NodeHandle nh;
	Eigen::Vector3f hint_point;
	bool hint_initialized;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
		display_cloud,
		pcl_select,
		can_cloud;
	pcl::SampleConsensusModelFromNormals<pcl::PointXYZ, pcl::Normal>::Ptr object_model;

	// ros publishers, subscribers, services
	ros::Subscriber pclPoints,
		selectedPoints;
	ros::ServiceServer service;
	void initializeSubscribers();
	void initializePublishers();
	void initializeServices();

	void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
	void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
	bool modeCB(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response);

	Eigen::Vector3f computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif
