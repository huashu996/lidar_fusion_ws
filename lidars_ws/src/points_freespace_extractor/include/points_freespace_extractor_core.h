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

class PointsFreespaceExtractor
{
private:
    std::string sub_topic_;
    std::string pub_marker_topic_;
    
    bool show_points_size_;
    bool show_time_;

    float sensor_height_;
    float radius_divider_;
    float theta_divider_;
    float curb_height_threshold_;
    float general_slope_threshold_;
    float passable_width_;

    ros::Subscriber sub_;
    ros::Publisher pub_marker_;
    
    struct PointXYZRTColor
    {
        pcl::PointXYZI point;

        float radius; // radius to (0, 0)
        float theta;  // polar angle in XY plane

        size_t radius_idx;
        size_t theta_idx;

        size_t original_idx; // index in the original point clouds
    };
    
    std::vector<PointXYZRTColor> last_space_;
    bool filter_initialized_;
    
    void convertPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
                           std::vector<std::vector<PointXYZRTColor>>& pc_converted);
    
    void classifyPointCloud(const std::vector<std::vector<PointXYZRTColor>>& pc,
                            pcl::PointIndices& freespace_indices);
    void refineFreespaceByPassableWidth(const std::vector<PointXYZRTColor>& pc,
                                        std::vector<PointXYZRTColor>& pc_refined);
    void smoothFreespace(const std::vector<PointXYZRTColor>& pc,
                         std::vector<PointXYZRTColor>& pc_smoothed);
    void extractFreespace(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_freespace,
                          visualization_msgs::Marker& region);
    void callback(const sensor_msgs::PointCloud2ConstPtr pc_msg);

public:
    PointsFreespaceExtractor(ros::NodeHandle& nh);
    ~PointsFreespaceExtractor();
};



