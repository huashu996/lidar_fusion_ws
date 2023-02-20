#include <ros/ros.h>
#include <ros/time.h>

#include <string>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <boost/thread/thread.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/passthrough.h>
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
typedef pcl::PointXYZI PointT;

class Lidar_Fusion
{

                  
private:
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_pandar_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_left_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_right_cloud_;
    message_filters::Synchronizer<MySyncPolicy> *m_sync_;
    
    ros::NodeHandle nh_;
    ros::Publisher pub_fusion_cloud_;
    ros::Publisher pub_regr_fusion_cloud_;
	string pandar_lidar_points_;
	string left_lidar_points_;
	string right_lidar_points_;
	string fusion_topic_;
	string regr_fusion_topic_;
    pcl::PointCloud<PointT>::Ptr left_local_laser;
    pcl::PointCloud<PointT>::Ptr right_local_laser;
    pcl::PointCloud<PointT>::Ptr pandar_local_laser;
    pcl::PointCloud<PointT>::Ptr left_right_cloud;
    pcl::PointCloud<PointT>::Ptr fusion_cloud;
    pcl::PointCloud<PointT>::Ptr regr_fusion_cloud;
    pcl::PointCloud<PointT>::Ptr right_cloud;
    pcl::PointCloud<PointT>::Ptr left_cloud;
    pcl::PointCloud<PointT>::Ptr left_cloud_rground;
    pcl::PointCloud<PointT>::Ptr right_cloud_rground;
    tf::TransformListener tf_listener_;
public:
    //构造函数
    Lidar_Fusion():
    nh_("~"){
    //读入参数
    left_lidar_points_ = nh_.param<std::string>("left_lidar_topic", "left_lidar_topic");
    right_lidar_points_ = nh_.param<std::string>("right_lidar_topic", "right_lidar_topic");
    pandar_lidar_points_ = nh_.param<std::string>("pandar_lidar_topic", "pandar_lidar_topic");
    fusion_topic_ = nh_.param<std::string>("fusion_topic", "fusion_topic");
    sub_left_cloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, left_lidar_points_, 1, ros::TransportHints().tcpNoDelay());
    regr_fusion_topic_ = nh_.param<std::string>("regr_fusion_topic", "regr_fusion_topic");
    sub_left_cloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, left_lidar_points_, 1, ros::TransportHints().tcpNoDelay());
	sub_right_cloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, right_lidar_points_, 1, ros::TransportHints().tcpNoDelay());
	sub_pandar_cloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, pandar_lidar_points_, 1, ros::TransportHints().tcpNoDelay());
	m_sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *sub_pandar_cloud_,*sub_left_cloud_, *sub_right_cloud_);
	//只有两个话题消息同时到达时才会进入回调函数
	m_sync_->registerCallback(boost::bind(&Lidar_Fusion::Callback, this, _1, _2,_3));
	allocateMemory(); //初始化

    }
    void allocateMemory()
    {
        left_local_laser.reset(new pcl::PointCloud<PointT>());
        right_local_laser.reset(new pcl::PointCloud<PointT>());
        pandar_local_laser.reset(new pcl::PointCloud<PointT>());
        left_right_cloud.reset(new pcl::PointCloud<PointT>());
        fusion_cloud.reset(new pcl::PointCloud<PointT>());
        regr_fusion_cloud.reset(new pcl::PointCloud<PointT>());
        right_cloud.reset(new pcl::PointCloud<PointT>());
        left_cloud.reset(new pcl::PointCloud<PointT>());
        left_cloud_rground.reset(new pcl::PointCloud<PointT>());
        right_cloud_rground.reset(new pcl::PointCloud<PointT>());
    }
    void resetParameters(){ 
        left_local_laser->clear();
        right_local_laser->clear();
        pandar_local_laser->clear();
        left_right_cloud->clear();
        fusion_cloud->clear();
        regr_fusion_cloud->clear();
        right_cloud->clear();
        left_cloud->clear();
        left_cloud_rground->clear();
        right_cloud_rground->clear();
    }
    void Callback(const sensor_msgs::PointCloud2::ConstPtr &pandar_msg,const sensor_msgs::PointCloud2::ConstPtr &left_msg, const sensor_msgs::PointCloud2::ConstPtr &right_msg)
    {
        resetParameters();
        pcl::fromROSMsg(*pandar_msg, *pandar_local_laser);
        
        static tf::StampedTransform trans_left_lidar_in_base, trans_right_lidar_in_base;
	    static bool left_transform_get=false, right_transform_get=false;
	    
	    pcl::fromROSMsg(*left_msg, *left_local_laser);
	    bool ok1 = tf_listener_.waitForTransform(pandar_msg->header.frame_id, left_msg->header.frame_id, ros::Time(0), ros::Duration(2.0));
	    tf_listener_.lookupTransform(pandar_msg->header.frame_id, left_msg->header.frame_id, ros::Time(0), trans_left_lidar_in_base);
	    if(ok1)
	    	left_transform_get = true;
	    pcl_ros::transformPointCloud(*left_local_laser, *left_cloud, trans_left_lidar_in_base);

	    
	    pcl::fromROSMsg(*right_msg, *right_local_laser);
	    bool ok2 = tf_listener_.waitForTransform(pandar_msg->header.frame_id, right_msg->header.frame_id, ros::Time(0), ros::Duration(2.0));
	    tf_listener_.lookupTransform(pandar_msg->header.frame_id, right_msg->header.frame_id, ros::Time(0), trans_right_lidar_in_base);
	    if(ok2)
	    	left_transform_get = true;
	    pcl_ros::transformPointCloud(*right_local_laser, *right_cloud, trans_right_lidar_in_base);
	    *left_right_cloud =  *left_cloud + *right_cloud;
	    *fusion_cloud = *left_right_cloud + *pandar_local_laser;
	    //发布融合后的点云
        sensor_msgs::PointCloud2::Ptr fusion_cloud_msg(new sensor_msgs::PointCloud2);
	    pcl::toROSMsg (*fusion_cloud, *fusion_cloud_msg);
	    fusion_cloud_msg->header = pandar_msg->header;
	    pub_fusion_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(fusion_topic_, 10);
	    pub_fusion_cloud_.publish(fusion_cloud_msg);
	    
	    //发布左右去除地面，上部分切割的点云
	    //滤波
        
        pcl::PassThrough<pcl::PointXYZI> pass2;
        pass2.setInputCloud (left_right_cloud);
        pass2.setFilterFieldName ("x");
        pass2.setFilterLimits (-1.1, 0.9);
        pass2.setFilterLimitsNegative (true);
        pass2.filter (*left_right_cloud);

        
        pcl::PassThrough<pcl::PointXYZI> pass3;
        pass3.setInputCloud (left_right_cloud);
        pass3.setFilterFieldName ("y");
        pass3.setFilterLimits (-0.9, 0.9);
        pass3.setFilterLimitsNegative (true);
        pass3.filter (*left_right_cloud);
        

        
        
	    *regr_fusion_cloud = *left_right_cloud + *pandar_local_laser;
	    //*regr_fusion_cloud = *pandar_local_laser;
	    pcl::PassThrough<pcl::PointXYZI> pass4;
        pass4.setInputCloud (regr_fusion_cloud);
        pass4.setFilterFieldName ("z");
        pass4.setFilterLimits (-1.8, 2);
        pass4.setFilterLimitsNegative (false);
        pass4.filter (*regr_fusion_cloud);
        
        pcl::PassThrough<pcl::PointXYZI> pass5;
        pass4.setInputCloud (regr_fusion_cloud);
        pass4.setFilterFieldName ("y");
        pass4.setFilterLimits (-20, 20);
        pass4.setFilterLimitsNegative (false);
        pass4.filter (*regr_fusion_cloud);
        
        pcl::PassThrough<pcl::PointXYZI> pass6;
        pass4.setInputCloud (regr_fusion_cloud);
        pass4.setFilterFieldName ("x");
        pass4.setFilterLimits (-50, 50);
        pass4.setFilterLimitsNegative (false);
        pass4.filter (*regr_fusion_cloud);
        //发布融合后的点云
        sensor_msgs::PointCloud2::Ptr regr_fusion_cloud_msg(new sensor_msgs::PointCloud2);
	    pcl::toROSMsg (*regr_fusion_cloud, *regr_fusion_cloud_msg);
	    regr_fusion_cloud_msg->header = pandar_msg->header;
	    pub_regr_fusion_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(regr_fusion_topic_, 10);
	    pub_regr_fusion_cloud_.publish(regr_fusion_cloud_msg);

    }
};

int main(int argc, char** argv)
{
  //1、节点初始化 及定义参数
  ros::init(argc, argv, "fusion_lidar_node");
  Lidar_Fusion RF;
  ros::spin();
  return 0;
}
