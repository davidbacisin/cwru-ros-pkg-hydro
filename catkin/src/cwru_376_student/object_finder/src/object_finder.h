#ifndef _OBJECT_FINDER_H_
#define _OBJECT_FINDER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>

#include <cwru_srv/simple_int_service_message.h> // for the process mode service
#include <Eigen/Eigen>

enum ProcessMode {
	IDLE,
	FIND_CAN
};

class ObjectFinder {
public:
	ObjectFinder(ros::NodeHandle);

	void setObjectModel(pcl::SampleConsensusModel<pcl::PointXYZ>& model);
	Eigen::VectorXf find();
private:
	~ObjectFinder();
	ros::NodeHandle nh;
	pcl::PointXYZ hint_point;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_disk,
		cloud_out,
		display_cloud,
		pcl_select,
		can_cloud;
	pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr object_model;

	// ros publishers, subscribers, services
	ros::Subscriber pclPoints,
		selectedPoints;
	ros::Publisher pubCloud,
		pubPcdCloud;
	void initializeSubscribers();
	void initializePublishers();
	void initializeServices();

	void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
	void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
	void modeCB(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response)
};

#endif
