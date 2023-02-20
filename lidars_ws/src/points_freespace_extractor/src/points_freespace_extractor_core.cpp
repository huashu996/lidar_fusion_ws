#include "points_freespace_extractor_core.h"

PointsFreespaceExtractor::PointsFreespaceExtractor(ros::NodeHandle& nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/rslidar_points");
    nh.param<std::string>("pub_marker_topic", pub_marker_topic_, "/feasible_region");
    
    nh.param<bool>("show_points_size", show_points_size_, false);
    nh.param<bool>("show_time", show_time_, false);

    nh.param<float>("sensor_height", sensor_height_, 2.0);
    nh.param<float>("radius_divider", radius_divider_, 0.15);
    nh.param<float>("theta_divider", theta_divider_, 0.4);
    nh.param<float>("curb_height_threshold", curb_height_threshold_, 0.2);
    nh.param<float>("general_slope_threshold", general_slope_threshold_, 4.0);
    nh.param<float>("passable_width", passable_width_, 2.0);
    
    sub_ = nh.subscribe(sub_topic_, 1, &PointsFreespaceExtractor::callback, this);
    pub_marker_ = nh.advertise<visualization_msgs::Marker>(pub_marker_topic_, 1);
    
    filter_initialized_ = false;
    
    ros::spin();
}

PointsFreespaceExtractor::~PointsFreespaceExtractor()
{
}

void PointsFreespaceExtractor::convertPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
                                                 std::vector<std::vector<PointXYZRTColor>>& pc_converted)
{
    // floor(x)返回小于或等于x的最大整数
    // ceil(x)返回大于x的最小整数
    size_t num = ceil(360 / theta_divider_);

    // pc_converted中包含num条射线，每条射线都是PointXYZRTColor类型点的集合
    pc_converted.resize(num);

    // 以射线的形式组织点云
    #pragma omp for
    for(size_t i = 0; i < pc->points.size(); i++)
    {
        PointXYZRTColor new_point;
        float radius = sqrt(pc->points[i].x * pc->points[i].x + pc->points[i].y * pc->points[i].y);
        float theta = atan2(pc->points[i].y, pc->points[i].x) * 180 / PI;
        if(theta < 0)
        {
            theta += 360;
        }

        size_t radius_idx = floor(radius / radius_divider_);
        size_t theta_idx = floor(theta / theta_divider_);

        new_point.point = pc->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radius_idx = radius_idx;
        new_point.theta_idx = theta_idx;
        new_point.original_idx = i;

        pc_converted[theta_idx].push_back(new_point);
    }

    // 将同一条射线上的点按照半径排序
    #pragma omp for
    for(size_t i = 0; i < num; i++)
    {
        std::sort(pc_converted[i].begin(), pc_converted[i].end(), [](const PointXYZRTColor& a, const PointXYZRTColor& b) {return a.radius < b.radius;});
    }
}

void PointsFreespaceExtractor::classifyPointCloud(const std::vector<std::vector<PointXYZRTColor>>& pc,
                                                  pcl::PointIndices& freespace_indices)
{
    freespace_indices.indices.clear();
    
    // 遍历每一条射线
    #pragma omp for
    for(size_t i = 0; i < pc.size(); i++)
    {
        float pre_radius = 0;
        float pre_z = - sensor_height_;
        bool pre_freespace = true;
        
        // 遍历射线上的每一个点，提取可行域内的点
        for(size_t j = 0; j < pc[i].size(); j++) 
        {
            float cur_radius = pc[i][j].radius;
            float cur_z = pc[i][j].point.z;
            bool cur_freespace;

            // 如果当前点与上一个点距离很近，则舍弃当前点，相当于体素栅格滤波
            if(fabs(cur_z - pre_z) <= 0.15 && fabs(cur_radius - pre_radius) <= 0.15) {continue;}
            
            if(j == 0)
            {
                // 根据全局坡度判定
                float slope_g = atan2(fabs(cur_z - (- sensor_height_)), cur_radius) * 180 / PI;
                if(slope_g <= general_slope_threshold_) {cur_freespace = true;}
                else {cur_freespace = false;}
            }
            else
            {
                // 根据高度差判定
                if(fabs(cur_z - pre_z) <= curb_height_threshold_ && pre_freespace)
                {
                    cur_freespace = true;
                }
                else
                {
                    cur_freespace = false;
                }
            }
            
            if(cur_freespace) {freespace_indices.indices.push_back(pc[i][j].original_idx);}

            pre_radius = cur_radius;
            pre_z = cur_z;
            pre_freespace = cur_freespace;
        }
    }
}

void PointsFreespaceExtractor::refineFreespaceByPassableWidth(const std::vector<PointXYZRTColor>& space,
                                                              std::vector<PointXYZRTColor>& space_refined)
{
    space_refined.clear();
    size_t num = ceil(360 / theta_divider_);
    for(size_t p = 0; p < num; p++)
    {
        int iter = 0;
        int real_n = 0;
        
        float p_radius = space[p].radius;
        float expect_t = atan2(passable_width_, p_radius) * 180 / PI;
        int expect_n = expect_t / theta_divider_;
        while(iter < 10)
        {
            // 正向搜索满足通过条件的点
            int iter_f = 1;
            float min_radius_f = space[(p + iter_f) % num].radius;
            while(p_radius - min_radius_f < 1.0 && real_n < expect_n)
            {
                real_n++, iter_f++;
                min_radius_f = space[(p + iter_f) % num].radius;
            }
            
            // 反向搜索满足通过条件的点
            int iter_b = 1;
            float min_radius_b = space[(num + p - iter_b) % num].radius;
            while(p_radius - min_radius_b < 1.0 && real_n < expect_n)
            {
                real_n++, iter_b++;
                min_radius_b = space[(num + p - iter_b) % num].radius;
            }
            
            // 调整可行域范围
            iter++;
            if(real_n < expect_n)
            {
                p_radius = min_radius_f < min_radius_b ? min_radius_f : min_radius_b;
            }
            else
            {
                break;
            }
        }
        
        PointXYZRTColor new_point;
        new_point.radius = p_radius;
        new_point.point.x = p_radius * cos(p * theta_divider_ * PI / 180);
        new_point.point.y = p_radius * sin(p * theta_divider_ * PI / 180);
        space_refined.push_back(new_point);
    }
}

void PointsFreespaceExtractor::smoothFreespace(const std::vector<PointXYZRTColor>& space,
                                               std::vector<PointXYZRTColor>& space_smoothed)
{
    space_smoothed.clear();
    size_t num = ceil(360 / theta_divider_);
    for(size_t p = 0; p < num; p++)
    {
        int n = 2;
        std::vector<float> radius_vec;
        radius_vec.resize(2 * n + 1);
        
        radius_vec[n] = space[p].radius;
        for(int k = 0; k < n; k++)
        {
            radius_vec[n + k + 1] = space[(p + k + 1) % num].radius;
            radius_vec[n - k - 1] = space[(p - k - 1 + num) % num].radius;
        }
        
        float mean_radius = 0;
        for(int k = 0; k < 2 * n + 1; k++) {mean_radius += radius_vec[k];}
        mean_radius /= 2 * n + 1;

        PointXYZRTColor new_point;
        new_point.radius = mean_radius;
        new_point.point.x = mean_radius * cos(p * theta_divider_ * PI / 180);
        new_point.point.y = mean_radius * sin(p * theta_divider_ * PI / 180);
        space_smoothed.push_back(new_point);
    }
}

void PointsFreespaceExtractor::extractFreespace(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_freespace,
                                                visualization_msgs::Marker& region)
{
    std::vector<PointXYZRTColor> space;
    size_t num = ceil(360 / theta_divider_);

    // 初始化PointXYZRTColor型数组，元素的索引由极角决定
    #pragma omp for
    for(size_t p = 0; p < num; p++)
    {
        PointXYZRTColor new_point;
        new_point.radius = 0;
        new_point.point.x = 0;
        new_point.point.y = 0;
        space.push_back(new_point);
    }

    // 将最远点作为数组中每个极角对应的元素
    #pragma omp for
    for(size_t i = 0; i < pc_freespace->points.size(); i++)
    {
        float radius = sqrt(pc_freespace->points[i].x * pc_freespace->points[i].x + pc_freespace->points[i].y * pc_freespace->points[i].y);
        float theta = atan2(pc_freespace->points[i].y, pc_freespace->points[i].x) * 180 / PI;
        if(theta < 0) {theta += 360;}

        size_t theta_idx = floor(theta / theta_divider_);

        if(radius > space[theta_idx].radius)
        {
            space[theta_idx].radius = radius;
            space[theta_idx].point.x = pc_freespace->points[i].x;
            space[theta_idx].point.y = pc_freespace->points[i].y;
        }
    }
    
    // 利用通过宽度调整可行域
    for(int k = 0; k < 5; k++)
    {
        std::vector<PointXYZRTColor> space_refined;
        refineFreespaceByPassableWidth(space, space_refined);
        space = space_refined;
    }
    
    // 均值滤波
    for(int k = 0; k < 1; k++)
    {
        std::vector<PointXYZRTColor> space_smoothed;
        smoothFreespace(space, space_smoothed);
        space = space_smoothed;
    }
    
    // 惯性滤波
    if(!filter_initialized_)
    {
        last_space_ = space;
        filter_initialized_ = true;
    }
    else
    {
        for(size_t p = 0; p < num; p++)
        {
            space[p].point.x = 0.5 * space[p].point.x + 0.5 * last_space_[p].point.x;
            space[p].point.y = 0.5 * space[p].point.y + 0.5 * last_space_[p].point.y;
        }
        last_space_ = space;
    }
    
    // 用连续三角形表示可行域，计算三角形的顶点
    geometry_msgs::Point origin;
    origin.x = 0, origin.y = 0, origin.z = - sensor_height_;
    for(size_t p = 0; p < num; p++)
    {
        geometry_msgs::Point p1;
        p1.x = space[p].point.x;
        p1.y = space[p].point.y;
        p1.z = - sensor_height_;
        
        geometry_msgs::Point p2;
        p2.x = space[(p + 1) % num].point.x;
        p2.y = space[(p + 1) % num].point.y;
        p2.z = - sensor_height_;
        
        region.points.push_back(origin);
        region.points.push_back(p1);
        region.points.push_back(p2);
    }
    
    region.ns = "feasible_region";
    region.id = 0;
    
    region.type = visualization_msgs::Marker::TRIANGLE_LIST;
    region.action = visualization_msgs::Marker::ADD;

    region.scale.x = 1;
    region.scale.y = 1;
    region.scale.z = 1;

    region.color.r = 0.0f;
    region.color.g = 0.8f;
    region.color.b = 0.0f;
    region.color.a = 0.65;

    region.lifetime = ros::Duration(0.1);
}

void PointsFreespaceExtractor::callback(const sensor_msgs::PointCloud2ConstPtr pc_msg)
{
    ros::Time time_start = ros::Time::now();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_current(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_freespace(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc_msg, *pc_current);
    
    // 组织点云
    std::vector<std::vector<PointXYZRTColor>> pc_organized;
    convertPointCloud(pc_current, pc_organized);
    
    // 判定可行域点云
    pcl::PointIndices freespace_indices;
    classifyPointCloud(pc_organized, freespace_indices);
    
    pcl::ExtractIndices<pcl::PointXYZI> extractor_freespace;
    extractor_freespace.setInputCloud(pc_current);
    extractor_freespace.setIndices(boost::make_shared<pcl::PointIndices>(freespace_indices));
    extractor_freespace.setNegative(false); // true removes the indices, false leaves only the indices
    extractor_freespace.filter(*pc_freespace);
    
    // 提取可行域
    visualization_msgs::Marker region;
    extractFreespace(pc_freespace, region);
    region.header = pc_msg->header;
    pub_marker_.publish(region);

    ros::Time time_end = ros::Time::now();
    
    if(show_points_size_ || show_time_)
    {
        std::cout << "" << std::endl;
        std::cout << "[points_freespace_extractor]" << std::endl;
    }
    
    if(show_points_size_)
    {
        std::cout << "Size of freespace point clouds: " << pc_freespace->points.size() << std::endl;
    }

    if(show_time_)
    {
        std::cout << "Time cost per frame: " << time_end - time_start << "s" << std::endl;
    }
}


