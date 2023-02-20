#pragma once

#include "omp.h"
#include <ros/ros.h>
 #include<flann/flann.h>
#include <opencv2/opencv.hpp>
#include <assert.h>
#include <ctime>
#include <algorithm>
#include <numeric>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#define PI 3.1415926

class EuCluster
{
private:
    std::string sub_topic_;
    std::string pub_topic_;

    bool show_objects_num_;
    bool show_time_;

    bool fit_obb_;
    
    double min_cluster_points_num_;
    double max_cluster_points_num_;
    
    double min_cluster_size_;
    double max_cluster_size_;
    
    int seg_num_;
    std::vector<double> seg_distance_;
    std::vector<double> cluster_distance_;

    bool road_info_;
    std::vector<float> road_edge_left_;
    std::vector<float> road_edge_right_;

    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::vector<size_t> sortIndexes(const std::vector<float>& v,
                                    const bool& increase);
    void projectPointCloudOnLine(const std::vector<float>& xs,
                                 const std::vector<float>& ys,
                                 const float& x0,
                                 const float& y0,
                                 const float& phi,
                                 float& xp_max,
                                 float& yp_max,
                                 float& xp_min,
                                 float& yp_min);
    float computeDistanceBetweenPointAndLine(const float& x1,
                                             const float& y1,
                                             const float& x2,
                                             const float& y2,
                                             const float& x,
                                             const float& y);
    float computeBoundingBoxByOrientation(const std::vector<float>& xs,
                                          const std::vector<float>& ys,
                                          float& x0,
                                          float& y0,
                                          float& l,
                                          float& w,
                                          float& phi);
    void fitOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                                visualization_msgs::Marker& marker);
    void fitBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                        visualization_msgs::Marker& marker);
    void clusterInSegment(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                          const double& cluster_distance,
                          visualization_msgs::MarkerArray& objs);
    void cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                 visualization_msgs::MarkerArray& objs);
    void crop(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
              const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cropped,
              const std::vector<float>& edge_left,
              const std::vector<float>& edge_right);
    void callback(const sensor_msgs::PointCloud2ConstPtr in);

public:
    EuCluster(ros::NodeHandle &nh);
    ~EuCluster();
};
