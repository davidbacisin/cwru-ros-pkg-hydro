#ifndef _OBJECT_FINDER_H_
#define _OBJECT_FINDER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/recognition/ransac_based/obj_rec_ransac.h>

#include <cwru_srv/simple_int_service_message.h> // for the process mode service
#include <Eigen/Eigen>
#include <math.h>

enum ProcessMode {
	IDLE 		= 0,
	FIND_CAN 	= 1,
	HINT 		= 2,
	UPDATE_VISION	= 3,
	FIND_TABLE	= 4,
	FIND_CAN2	= 5
};

class ObjectFinder {
public:
	ObjectFinder(ros::NodeHandle);

	pcl::PointCloud<pcl::PointXYZ> getCanCloud(float radius, float height);

	std::vector<int> segmentNearHint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius);
	pcl::ModelCoefficients::Ptr findCan(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud);
	pcl::ModelCoefficients::Ptr findTable(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud);
	
	// make public so external functions can use the publisher
	ros::Publisher pubCloud,
		pubPcdCloud;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_kinect;
	sensor_msgs::PointCloud2ConstPtr kinect_raw;
	bool kinect_initialized;
private:
	ros::NodeHandle nh;
	Eigen::Vector3f hint_point,
		vertical_axis;
	bool hint_initialized,
		vertical_initialized;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_select;
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

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
	Eigen::Vector3f computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<int> indices);
	// to be called only by the higher-level methods which specify the object to be found
	pcl::ModelCoefficients::Ptr find(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, Eigen::Vector3f *centroid);
};

#endif
