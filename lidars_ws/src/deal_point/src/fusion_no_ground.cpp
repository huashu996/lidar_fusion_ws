#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <functional>
#include <pcl/filters/statistical_outlier_removal.h>
template<typename PointType>
class GroundSeg:public pcl::PCLBase<PointType>
{
public:
    GroundSeg(){
    }
 
    void seg(pcl::PointIndices::Ptr& ground_indices,pcl::PointIndices::Ptr&  no_ground_indices) {
        bool init_ret = this->initCompute();
        if(init_ret == false) {
            return;
        }
        pcl::console::TicToc tt;
        typename pcl::search::KdTree<PointType>::Ptr kdtree_ptr(new pcl::search::KdTree<PointType>);
        pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices());
        indices->indices.reserve(this->getInputCloud()->size());
        {
            tt.tic();
            pcl::NormalEstimationOMP<PointType,pcl::Normal> normal_est_omp;
            normal_est_omp.setInputCloud(this->getInputCloud());
            //normal_est_omp.setIndices(this->getIndices());
            normal_est_omp.setSearchMethod(kdtree_ptr);
            normal_est_omp.setViewPoint(1e9,1e9,1e9);
            normal_est_omp.setKSearch(35); //neighbour size
            normal_est_omp.compute(*normal_ptr);
            {
                for(int i = 0; i < normal_ptr->points.size();++i) {
                    const auto normal = normal_ptr->at(i);
                    const double abs_normal_z = std::abs(normal.normal_z);
                    if(std::isnormal(abs_normal_z) && abs_normal_z > 0.5) {
                        indices->indices.push_back(i);
                    }
                }
            }
            std::cout <<__FUNCTION__ << ":" <<__LINE__<<",calc normal cost " << tt.toc() << " ms" << std::endl;
        }
#if 1
        {
            tt.tic();
            pcl::SACSegmentation<pcl::PointXYZI> seg;
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            //pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
 
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setDistanceThreshold(0.3);
            seg.setInputCloud(this->getInputCloud());
            seg.setIndices(indices);
            //std::cout << __FUNCTION__ << __LINE__ << std::endl;
            seg.segment(*ground_indices,*coefficients);
            {
                pcl::ExtractIndices<PointType> extract;   //点提取对象
                extract.setInputCloud(this->getInputCloud());
                extract.setIndices(ground_indices);
                extract.setNegative(true);//设置成true是保存滤波后剩余的点，false是保存在区域内的点
                extract.filter(no_ground_indices->indices);
            }

            std::cout <<__FUNCTION__ << ":" <<__LINE__<<",ransac seg cost " << tt.toc() << " ms" << std::endl;
        }
#endif
        this->deinitCompute();
        //std::cout << __FUNCTION__ << __LINE__ <<"end"<< std::endl;
    }
};
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "points_ground_filter_node");
    ros::NodeHandle nh;
    std::string sub_topic_;
    std::string pub_ground_topic_;
    std::string pub_no_ground_topic_;
    ros::Subscriber sub_;
    ros::Publisher pub_ground_, pub_no_ground_;
    nh.param<std::string>("sub_topic", sub_topic_,"/pandar_points_no_ground");
    nh.param<std::string>("pub_ground_topic", pub_ground_topic_, "/fusion_points_ground");
    nh.param<std::string>("pub_no_ground_topic", pub_no_ground_topic_, "/fusion_points_no_ground");
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(pub_ground_topic_, 1);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(pub_no_ground_topic_, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_no_ground_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr  ground_indices(new pcl::PointIndices);
    pcl::PointIndices::Ptr  no_ground_indices(new pcl::PointIndices);
    GroundSeg<pcl::PointXYZI> ground_seg;
    
    const boost::function<void (const boost::shared_ptr<sensor_msgs::PointCloud2 const>&)> callback =[&](sensor_msgs::PointCloud2::ConstPtr pc_msg) {
        pcl::fromROSMsg(*pc_msg, *cloud);
        ground_seg.setInputCloud(cloud);
        ground_seg.seg(ground_indices, no_ground_indices);
        std::cerr << "ground points: " << ground_indices->indices.size() << std::endl;
        std::cerr << "no_ground points: " << no_ground_indices->indices.size() << std::endl;
        pcl::ExtractIndices<pcl::PointXYZI> extractor_ground;
        extractor_ground.setInputCloud(cloud);
        extractor_ground.setIndices(boost::make_shared<pcl::PointIndices>(*ground_indices));
        extractor_ground.setNegative(false); // true removes the indices, false leaves only the indices
        extractor_ground.filter(*pc_ground_filtered);

        pcl::ExtractIndices<pcl::PointXYZI> extractor_no_ground;
        extractor_no_ground.setInputCloud(cloud);
        extractor_no_ground.setIndices(boost::make_shared<pcl::PointIndices>(*no_ground_indices));
        extractor_no_ground.setNegative(false); // true removes the indices, false leaves only the indices
        extractor_no_ground.filter(*pc_no_ground_filtered);
        
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(pc_no_ground_filtered);
        sor.setMeanK(5);//考虑查询临近点数
        sor.setStddevMulThresh (1.0);//距离
        sor.filter (*pc_no_ground_filtered);

        // 发布地面点云
        sensor_msgs::PointCloud2 pc_msg_ground;
        pcl::toROSMsg(*pc_ground_filtered, pc_msg_ground);
        pc_msg_ground.header.stamp = pc_msg->header.stamp;
        pc_msg_ground.header.frame_id = pc_msg->header.frame_id;
        pub_ground_.publish(pc_msg_ground);
        
        // 发布非地面点云
        sensor_msgs::PointCloud2 pc_msg_no_ground;
        pcl::toROSMsg(*pc_no_ground_filtered, pc_msg_no_ground);
        pc_msg_no_ground.header.stamp = pc_msg->header.stamp;
        pc_msg_no_ground.header.frame_id = pc_msg->header.frame_id;
        pub_no_ground_.publish(pc_msg_no_ground);
    };
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(sub_topic_, 1, callback);
    ros::spin();
    return 0;
}
