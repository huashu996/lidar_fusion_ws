#include <omp.h>
#include <ros/ros.h>
#include <string>

#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <perception_msgs/Obstacle.h>
#include <perception_msgs/ObstacleArray.h>

#include <Eigen/Dense>
#include <cassert>
#include <limits.h>
#include <float.h>

#include "kalman.hpp"
#include "obj.hpp"

static std::vector<int> COLOR_B = {244, 233, 156, 103, 63, 33, 3, 0, 0};
static std::vector<int> COLOR_G = {67, 30, 99, 58, 81, 150, 169, 188, 150};
static std::vector<int> COLOR_R = {54, 99, 176, 183, 181, 243, 244, 212, 136};

class MTT
{
private:
    std::string sub_topic_;
    std::string pub_topic_;
    std::string pub_topic_obstacle_array_;
    std::string frame_id_;

    bool show_objects_num_;
    bool show_time_;

    double time_interval_;
    double gate_threshold_;
    int blind_update_limit_;

    double sigma_ax_;
    double sigma_ay_;
    double sigma_ox_;
    double sigma_oy_;

    double min_scale_;
    double max_scale_;
    double min_height_;
    double max_height_;

    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub_obstacle_array_;

    std::vector<Object> objs_observed_;
    std::vector<int> objs_label_; // 0: unoccupied, 1: for objs_tracked_, 2: for objs_temp_
    
    std::vector<Object> objs_tracked_;
    std::vector<Object> objs_temp_;
    int number_;

    void observe(const visualization_msgs::MarkerArray& in);
    void update();
    void augment();
    void publishMarkers(visualization_msgs::MarkerArray& markers);
    void publishObstacles(perception_msgs::ObstacleArray& obstacles);
    void callback(const visualization_msgs::MarkerArray& in);

public:
    MTT(ros::NodeHandle& nh);
    ~MTT();
};

MTT::MTT(ros::NodeHandle& nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/objects");
    nh.param<std::string>("pub_topic", pub_topic_, "/objects_tracked");
    nh.param<std::string>("pub_topic_obstacle_array", pub_topic_obstacle_array_, "/obstacle_array");
    nh.param<std::string>("frame_id", frame_id_, "pandar");

    nh.param<bool>("show_objects_num", show_objects_num_, false);
    nh.param<bool>("show_time", show_time_, false);

    nh.param<double>("time_interval", time_interval_, 0.1);
    nh.param<double>("gate_threshold", gate_threshold_, 1000);
    nh.param<int>("blind_update_limit", blind_update_limit_, 5);

    nh.param<double>("sigma_ax", sigma_ax_, 0.1);
    nh.param<double>("sigma_ay", sigma_ay_, 0.1);
    nh.param<double>("sigma_ox", sigma_ox_, 0.1);
    nh.param<double>("sigma_oy", sigma_oy_, 0.1);

    nh.param<double>("min_scale", min_scale_, 1.0);
    nh.param<double>("max_scale", max_scale_, 6.0);
    nh.param<double>("min_height", min_height_, 1.0);
    nh.param<double>("max_height", max_height_, 2.5);

    sub_ = nh.subscribe(sub_topic_, 1, &MTT::callback, this);
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_, 1);
    pub_obstacle_array_ = nh.advertise<perception_msgs::ObstacleArray>(pub_topic_obstacle_array_, 1);

    number_ = 0;

    ros::spin();
}

MTT::~MTT()
{
}

void MTT::observe(const visualization_msgs::MarkerArray& in)
{
    objs_observed_.clear();
    objs_label_.clear();
    
    for(int i = 0; i < in.markers.size(); i++)
    {
        Object obj;
        obj.x0 = in.markers[i].pose.position.x;
        obj.y0 = in.markers[i].pose.position.y;
        obj.z0 = in.markers[i].pose.position.z;
        obj.l = in.markers[i].scale.x;
        obj.w = in.markers[i].scale.y;
        obj.h = in.markers[i].scale.z;

        double xx = in.markers[i].pose.orientation.x;
        double yy = in.markers[i].pose.orientation.y;
        double zz = in.markers[i].pose.orientation.z;
        double ww = in.markers[i].pose.orientation.w;
        
        Eigen::Quaterniond q(ww, xx, yy, zz);
        Eigen::Vector3d q_eul = q.toRotationMatrix().eulerAngles(2, 1, 0);
        double phi = q_eul[0];

        obj.phi = phi;
        obj.has_orientation = false;

        obj.xref = in.markers[i].pose.position.x;
        obj.yref = in.markers[i].pose.position.y;

        double obj_scale = obj.l > obj.w ? obj.l : obj.w;
        double obj_height = obj.h;
        
        if((obj_scale >= min_scale_) && (obj_scale <= max_scale_) &&
           (obj_height >= min_height_) && (obj_height <= max_height_))
        {
            objs_observed_.push_back(obj);
        }
    }
    
    int num = objs_observed_.size(); objs_label_.resize(num);
    for(int i = 0; i < num; i++)
    {
        objs_label_[i] = 0;
    }
}

void MTT::update()
{
    #pragma omp for
    for(int j = 0; j < objs_tracked_.size(); j++)
    {
        bool flag = false;
        int idx = 0;
        double ddm = DBL_MAX;

        for(int k = 0; k < objs_observed_.size(); k++)
        {
            if(objs_label_[k] != 0) {continue;}
            
            double x = objs_observed_[k].xref;
            double y = objs_observed_[k].yref;
            double dd = objs_tracked_[j].tracker.compute_the_residual(x, y);
            if((dd < ddm) && (dd < gate_threshold_))
            {
                idx = k; ddm = dd; flag = true;
            }
        }

        if(flag)
        {
            objs_label_[idx] = 1;
            
            double zx = objs_observed_[idx].xref;
            double zy = objs_observed_[idx].yref;
            objs_tracked_[j].tracker.predict();
            objs_tracked_[j].tracker.update(zx, zy);
            objs_tracked_[j].tracker_blind_update = 0;

            objs_tracked_[j].xref = objs_tracked_[j].tracker.get_state()(0, 0);
            objs_tracked_[j].vx = objs_tracked_[j].tracker.get_state()(1, 0);
            objs_tracked_[j].yref = objs_tracked_[j].tracker.get_state()(2, 0);
            objs_tracked_[j].vy = objs_tracked_[j].tracker.get_state()(3, 0);

            objs_tracked_[j].x0 = objs_observed_[idx].x0;
            objs_tracked_[j].y0 = objs_observed_[idx].y0;
            objs_tracked_[j].z0 = objs_observed_[idx].z0;
            objs_tracked_[j].l = objs_observed_[idx].l;
            objs_tracked_[j].w = objs_observed_[idx].w;
            objs_tracked_[j].h = objs_observed_[idx].h;
            objs_tracked_[j].phi = objs_observed_[idx].phi;
            objs_tracked_[j].has_orientation = objs_observed_[idx].has_orientation;
        }
        else
        {
            objs_tracked_[j].tracker.predict();
            objs_tracked_[j].tracker_blind_update += 1;

            objs_tracked_[j].xref = objs_tracked_[j].tracker.get_state()(0, 0);
            objs_tracked_[j].vx = objs_tracked_[j].tracker.get_state()(1, 0);
            objs_tracked_[j].yref = objs_tracked_[j].tracker.get_state()(2, 0);
            objs_tracked_[j].vy = objs_tracked_[j].tracker.get_state()(3, 0);
            
            objs_tracked_[j].x0 = objs_tracked_[j].xref;
            objs_tracked_[j].y0 = objs_tracked_[j].yref;
        }
    }
    
    std::vector<int> miss;
    for(int j = 0; j < objs_tracked_.size(); j++)
    {
        miss.push_back(objs_tracked_[j].tracker_blind_update);
    }
    
    std::vector<Object> objs_copy = objs_tracked_; objs_tracked_.clear();
    for(int j = 0; j < objs_copy.size(); j++)
    {
        if(miss[j] <= blind_update_limit_)
        {
            objs_tracked_.push_back(objs_copy[j]);
        }
    }
}

void MTT::augment()
{
    #pragma omp for
    for(int j = 0; j< objs_temp_.size(); j++)
    {
        bool flag = false;
        int idx = 0;
        double ddm = DBL_MAX;

        for(int k = 0; k < objs_observed_.size(); k++)
        {
            if(objs_label_[k] != 0) {continue;}
            
            double x = objs_observed_[k].xref;
            double y = objs_observed_[k].yref;
            double dd = objs_temp_[j].tracker.compute_the_residual(x, y);
            if((dd < ddm) && (dd < gate_threshold_))
            {
                idx = k; ddm = dd; flag = true;
            }
        }

        if(flag)
        {
            objs_label_[idx] = 2;
            
            double zx = objs_observed_[idx].xref;
            double zy = objs_observed_[idx].yref;
            double x = objs_temp_[j].tracker.get_state()(0, 0);
            double y = objs_temp_[j].tracker.get_state()(2, 0);
            double t = time_interval_;
            objs_temp_[j].tracker.init(t, zx, (zx - x) / t, zy, (zy - y) / t,
                                       sigma_ax_, sigma_ay_, sigma_ox_, sigma_oy_);

            number_ += 1;
            number_ = number_ % 10000;
            objs_temp_[j].number = number_;
            
            objs_temp_[j].x0 = objs_observed_[idx].x0;
            objs_temp_[j].y0 = objs_observed_[idx].y0;
            objs_temp_[j].z0 = objs_observed_[idx].z0;
            objs_temp_[j].l = objs_observed_[idx].l;
            objs_temp_[j].w = objs_observed_[idx].w;
            objs_temp_[j].h = objs_observed_[idx].h;
            objs_temp_[j].phi = objs_observed_[idx].phi;
            objs_temp_[j].has_orientation = objs_observed_[idx].has_orientation;

            assert((COLOR_B.size() == COLOR_G.size()) && (COLOR_B.size() == COLOR_R.size()));
            
            int num_c = COLOR_R.size();
            objs_temp_[j].color_r = COLOR_R[number_ % num_c];
            objs_temp_[j].color_g = COLOR_G[number_ % num_c];
            objs_temp_[j].color_b = COLOR_B[number_ % num_c];
            
            objs_tracked_.push_back(objs_temp_[j]);
        }
    }
    
    objs_temp_.clear();
    for(int j = 0; j< objs_observed_.size(); j++)
    {
        if(objs_label_[j] == 0)
        {
            objs_temp_.push_back(objs_observed_[j]);
        }
    }
    
    #pragma omp for
    for(int j = 0; j < objs_temp_.size(); j++)
    {
        double x = objs_temp_[j].xref;
        double y = objs_temp_[j].yref;
        objs_temp_[j].tracker.init(time_interval_, x, 0, y, 0,
                                   sigma_ax_, sigma_ay_, sigma_ox_, sigma_oy_);
    }
}

void MTT::publishMarkers(visualization_msgs::MarkerArray& markers)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;
    
    for(int i = 0; i < objs_tracked_.size(); i++)
    {
        visualization_msgs::Marker mar;
        mar.header = header;

        // 设置该标记的命名空间和ID，ID应该是独一无二的
        // 具有相同命名空间和ID的标记将会覆盖
        mar.ns = "obstacle";
        mar.id = objs_tracked_[i].number;

        // 设置标记类型
        mar.type = visualization_msgs::Marker::CUBE;
        
        // 设置标记行为，ADD为添加，DELETE为删除
        mar.action = visualization_msgs::Marker::ADD;

        // 设置标记位姿
        mar.pose.position.x = objs_tracked_[i].x0;
        mar.pose.position.y = objs_tracked_[i].y0;
        mar.pose.position.z = objs_tracked_[i].z0;
        mar.pose.orientation.x = 0;
        mar.pose.orientation.y = 0;
        mar.pose.orientation.z = sin(0.5 * objs_tracked_[i].phi);
        mar.pose.orientation.w = cos(0.5 * objs_tracked_[i].phi);

        // 设置标记尺寸
        mar.scale.x = objs_tracked_[i].l;
        mar.scale.y = objs_tracked_[i].w;
        mar.scale.z = objs_tracked_[i].h;

        // 设置标记颜色，应确保不透明度alpha非零
        mar.color.r = (float) objs_tracked_[i].color_r / 255;
        mar.color.g = (float) objs_tracked_[i].color_g / 255;
        mar.color.b = (float) objs_tracked_[i].color_b / 255;
        mar.color.a = 0.85;

        // 设置标记生存时间，单位为s
        mar.lifetime = ros::Duration(0.1);
        mar.text = ' ';

        markers.markers.push_back(mar);
    }

    pub_.publish(markers);
}

void MTT::publishObstacles(perception_msgs::ObstacleArray& obstacles)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;
    
    for(int i = 0; i < objs_tracked_.size(); i++)
    {
        perception_msgs::Obstacle obs;
        obs.header = header;

        // 设置命名空间和ID，ID应该是独一无二的
        obs.ns = "obstacle";
        obs.id = objs_tracked_[i].number;

        // 设置位姿
        obs.pose.position.x = objs_tracked_[i].x0;
        obs.pose.position.y = objs_tracked_[i].y0;
        obs.pose.position.z = objs_tracked_[i].z0;
        obs.pose.orientation.x = 0;
        obs.pose.orientation.y = 0;
        obs.pose.orientation.z = sin(0.5 * objs_tracked_[i].phi);
        obs.pose.orientation.w = cos(0.5 * objs_tracked_[i].phi);

        // 设置尺寸
        obs.scale.x = objs_tracked_[i].l;
        obs.scale.y = objs_tracked_[i].w;
        obs.scale.z = objs_tracked_[i].h;

        // 设置速度
        obs.v_validity = true;
        obs.vx = objs_tracked_[i].vx;
        obs.vy = objs_tracked_[i].vy;
        obs.vz = 0;
        
        // 设置加速度
        obs.a_validity = false;
        obs.ax = 0;
        obs.ay = 0;
        obs.az = 0;

        obstacles.obstacles.push_back(obs);
    }

    pub_obstacle_array_.publish(obstacles);
}

void MTT::callback(const visualization_msgs::MarkerArray& markers_in)
{
    ros::Time time_start = ros::Time::now();

    observe(markers_in);
    update();
    augment();
    
    visualization_msgs::MarkerArray markers_out;
    publishMarkers(markers_out);

    perception_msgs::ObstacleArray obstacles;
    publishObstacles(obstacles);

    ros::Time time_end = ros::Time::now();

    if(show_objects_num_ || show_time_)
    {
        std::cout << "" << std::endl;
        std::cout << "[multiple_target_tracking]" << std::endl;
    }

    if(show_objects_num_)
    {
        std::cout << "Size of observed objects: " << markers_in.markers.size() << std::endl;
        std::cout << "Size of tracked objects: " << markers_out.markers.size() << std::endl;
    }

    if(show_time_)
    {
        std::cout << "Time cost per frame: " << time_end - time_start << "s" << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multiple_target_tracking");
    ros::NodeHandle nh("~");

    omp_set_num_threads(4);

    MTT mtt(nh);
    return 0;
}
