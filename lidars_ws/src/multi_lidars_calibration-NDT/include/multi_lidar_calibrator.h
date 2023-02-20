#ifndef PROJECT_MULTI_LIDAR_CALIBRATOR_H
#define PROJECT_MULTI_LIDAR_CALIBRATOR_H

#include <string>
#include <vector>
#include <map>
//map 是一种关联容器， 提供一对一的关联， 关联的形式为： KEY----VALUE（键值对），关键字不能重复
#include <chrono>
//C++11中，<chrono>是标准模板库中与时间有关的头文件
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//该消息表示带有时间标签和参考坐标的估计位姿

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#define __APP_NAME__ "multi_lidar_calibrator"

class ROSMultiLidarCalibratorApp

{
	ros::NodeHandle node_handle_;
	ros::Publisher calibrated_cloud_publisher_;

	ros::Subscriber initialpose_subscriber_;
    tf::TransformBroadcaster tf_br;

    int child_topic_num_;

    std::map<std::string, std::vector<double>> transfer_map_;

    std::string points_parent_topic_str, points_child_topic_str;


	double voxel_size_;
	double ndt_epsilon_;
	double ndt_step_size_;
	double ndt_resolution_;

	double initial_x_;
	double initial_y_;
	double initial_z_;
	double initial_roll_;
	double initial_pitch_;
	double initial_yaw_;

	int ndt_iterations_;

	std::string parent_frame_;
	std::string child_frame_;

	Eigen::Matrix4f current_guess_;//4×4float矩阵

//时间同步器
	typedef
	message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
			sensor_msgs::PointCloud2>   SyncPolicyT;

	typedef pcl::PointXYZ PointT;

	message_filters::Subscriber<sensor_msgs::PointCloud2>   *cloud_parent_subscriber_, *cloud_child_subscriber_;
	message_filters::Synchronizer<SyncPolicyT>              *cloud_synchronizer_;

    pcl::PointCloud<PointT>::Ptr in_parent_cloud_, in_child_cloud_, in_child_filtered_cloud_;

	/*!
	 * Receives 2 synchronized point cloud messages.
	 * @param[in] in_parent_cloud_msg Message containing pointcloud classified as ground.
	 * @param[in] in_child_cloud_msg Message containing pointcloud classified as obstacle.
	 */
	void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_parent_cloud_msg,
	                    const sensor_msgs::PointCloud2::ConstPtr& in_child_cloud_msg);

	/*!
	 * Obtains parameters from the command line, initializes subscribers and publishers.
	 * @param in_private_handle ROS private handle to get parameters for this node.
	 */
	void InitializeROSIo(ros::NodeHandle& in_private_handle);

	/*!
	 * Applies a Voxel Grid filter to the point cloud
	 * @param in_cloud_ptr point cloud to downsample
	 * @param out_cloud_ptr downsampled point cloud
	 * @param in_leaf_size voxel side size
	 */
	void DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr, pcl::PointCloud<PointT>::Ptr out_cloud_ptr, double in_leaf_size);

	/*!
	 * Publishes a PointCloud in the specified publisher
	 * @param in_publisher Publisher to use
	 * @param in_cloud_to_publish_ptr Cloud to Publish
	 */
	void PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr);

    void MatrixToTranform(Eigen::Matrix4f & matrix, tf::Transform & trans);

    void PerformNdtOptimize();

public:
	void Run();

	ROSMultiLidarCalibratorApp();
};

#endif //PROJECT_MULTI_LIDAR_CALIBRATOR_H
