#pragma once

#include <ros/ros.h>
#include <limits.h>
#include <float.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#define PI 3.1415926

class PointsGroundFilter
{
private:
    std::string sub_topic_;
    std::string pub_ground_topic_;
    std::string pub_no_ground_topic_;
    
    bool show_points_size_;
    bool show_time_;

    float sensor_height_;
    float radius_divider_;
    float theta_divider_;
    float local_slope_threshold_;
    float general_slope_threshold_;
    
    bool ground_filter_mode_;
    float ground_meank_;
    float ground_stdmul_;
    
    bool no_ground_filter_mode_;
    float no_ground_meank_;
    float no_ground_stdmul_;

    ros::Subscriber sub_;
    ros::Publisher pub_ground_, pub_no_ground_;
    
    struct PointXYZRTColor
    {
        pcl::PointXYZI point;

        float radius; // radius to (0, 0)
        float theta;  // polar angle in XY plane

        size_t radius_idx;
        size_t theta_idx;

        size_t original_idx; // index in the original point clouds
    };
    
    void convertPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
                           std::vector<std::vector<PointXYZRTColor>>& pc_converted);
    
    void classifyPointCloud(const std::vector<std::vector<PointXYZRTColor>>& pc,
                            pcl::PointIndices& ground_indices,
                            pcl::PointIndices& no_ground_indices);
                        
    void callback(const sensor_msgs::PointCloud2ConstPtr pc_msg);
public:
    PointsGroundFilter(ros::NodeHandle& nh);
    ~PointsGroundFilter();
};



