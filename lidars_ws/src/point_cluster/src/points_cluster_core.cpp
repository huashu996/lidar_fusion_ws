#include "points_cluster_core.h"

EuCluster::EuCluster(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/fusion_rslidar_points");
    nh.param<std::string>("pub_topic", pub_topic_, "/objects");

    nh.param<bool>("show_objects_num", show_objects_num_, false);
    nh.param<bool>("show_time", show_time_, false);

    nh.param<bool>("fit_obb", fit_obb_, false);
    
    nh.param<double>("min_cluster_points_num", min_cluster_points_num_, 5);
    nh.param<double>("max_cluster_points_num", max_cluster_points_num_, 4000);
    
    nh.param<double>("min_cluster_size", min_cluster_size_, 0.1);
    nh.param<double>("max_cluster_size", max_cluster_size_, 8);
    
    nh.param<int>("seg_num", seg_num_, 5);
    ros::param::get("~seg_distance", seg_distance_);
    ros::param::get("~cluster_distance", cluster_distance_);

    nh.param<bool>("road_info", road_info_, false);
    
    sub_ = nh.subscribe(sub_topic_, 1, &EuCluster::callback, this);
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_, 1);
    
    ros::spin();
}

EuCluster::~EuCluster()
{
}

std::vector<size_t> EuCluster::sortIndexes(const std::vector<float>& v,
                                           const bool& increase)
{
    std::vector<size_t> idxes(v.size());
    
    iota(idxes.begin(), idxes.end(), 0);
    
    if(increase)
        std::sort(idxes.begin(), idxes.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
    else
        std::sort(idxes.begin(), idxes.end(), [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});
    
    return idxes;
}

void EuCluster::projectPointCloudOnLine(const std::vector<float>& xs,
                                        const std::vector<float>& ys,
                                        const float& x0,
                                        const float& y0,
                                        const float& phi,
                                        float& xp_max,
                                        float& yp_max,
                                        float& xp_min,
                                        float& yp_min)
{
    assert(phi >= 0 && phi < M_PI);
    assert(xs.size() == ys.size());
    size_t num = xs.size();
    
    // ?????????????????????????????????
    float vx = cos(phi);
    float vy = sin(phi);
    
    xp_max = -FLT_MAX;
    yp_max = -FLT_MAX;
    xp_min = FLT_MAX;
    yp_min = FLT_MAX;
    
    if(fabs(vy) > 0.0001)
    {
        for(int i = 0; i < num; i++)
        {
            float yp = ys[i] * vy * vy + xs[i] * vx * vy + y0 * vx * vx - x0 * vx * vy;
            float xp = ((yp - y0) * vx + x0 * vy) / vy;
            
            // ???????????????????????????
            if(yp > yp_max) {yp_max = yp; xp_max = xp;}
            if(yp < yp_min) {yp_min = yp; xp_min = xp;}
        }
    }
    else
    {
        for(int i = 0; i < num; i++)
        {
            float yp = y0;
            float xp = xs[i];
            
            // ???????????????????????????
            if(xp > xp_max) {xp_max = xp; yp_max = yp;}
            if(xp < xp_min) {xp_min = xp; yp_min = yp;}
        }
    }
}

float EuCluster::computeDistanceBetweenPointAndLine(const float& x1,
                                                    const float& y1,
                                                    const float& x2,
                                                    const float& y2,
                                                    const float& x,
                                                    const float& y)
{
    float vx = x2 - x1;
    float vy = y2 - y1;
    float dis;
    if(fabs(vx) > 0.0001)
    {
        float k = vy / vx;
        dis = fabs(k * x - y - k * x1 + y1) / sqrt(k * k + 1);
    }
    else
    {
        dis = fabs(x - x1);
    }
    
    return dis;
}

float EuCluster::computeBoundingBoxByOrientation(const std::vector<float>& xs,
                                                 const std::vector<float>& ys,
                                                 float& x0,
                                                 float& y0,
                                                 float& l,
                                                 float& w,
                                                 float& phi)
{
    assert(phi >= 0 && phi < M_PI);
    assert(xs.size() == ys.size());
    size_t num = xs.size();
    
    float angle = phi;
    float angle_second;
    if(angle < M_PI / 2) angle_second = angle + M_PI / 2;
    else angle_second = angle - M_PI / 2;
    
    // ?????????????????????????????????
    float xp1, yp1, xp2, yp2;
    projectPointCloudOnLine(xs, ys, 0, 0, angle, xp1, yp1, xp2, yp2);
    float xp3, yp3, xp4, yp4;
    projectPointCloudOnLine(xs, ys, 0, 0, angle_second, xp3, yp3, xp4, yp4);
    
    // ?????????????????????????????????????????????????????????
    float dd1 = xp1 * xp1 + yp1 * yp1;
    float dd2 = xp2 * xp2 + yp2 * yp2;
    float dd3 = xp3 * xp3 + yp3 + yp3;
    float dd4 = xp4 * xp4 + yp4 + yp4;
    
    float xpl_near = dd1 < dd2 ? xp1 : xp2;
    float ypl_near = dd1 < dd2 ? yp1 : yp2;
    float xpl_far = dd1 < dd2 ? xp2 : xp1;
    float ypl_far = dd1 < dd2 ? yp2 : yp1;
    
    float xpw_near = dd3 < dd4 ? xp3 : xp4;
    float ypw_near = dd3 < dd4 ? yp3 : yp4;
    float xpw_far = dd3 < dd4 ? xp4 : xp3;
    float ypw_far = dd3 < dd4 ? yp4 : yp3;
    
    // ??????????????????????????????????????????
    float xnn = xpl_near + xpw_near;
    float ynn = ypl_near + ypw_near;
    
    float xfn = xpl_far + xpw_near;
    float yfn = ypl_far + ypw_near;
    
    float xff = xpl_far + xpw_far;
    float yff = ypl_far + ypw_far;
    
    float xnf = xpl_near + xpw_far;
    float ynf = ypl_near + ypw_far;
    
    // ????????????????????????
    l = sqrt(pow(xp1 - xp2, 2) + pow(yp1 - yp2, 2));
    w = sqrt(pow(xp3 - xp4, 2) + pow(yp3 - yp4, 2));
    
    if(l >= w) phi = angle;
    else{phi = angle_second; float temp = l; l = w; w = temp;}
    
    x0 = (xp1 + xp2 + xp3 + xp4) / 2;
    y0 = (yp1 + yp2 + yp3 + yp4) / 2;
    
    // ????????????????????????Sum of Characteristic Distance???SCD???
    float scd = 0;
    for(int i = 0; i < num; i++)
    {
        float dis1 = computeDistanceBetweenPointAndLine(xnn, ynn, xfn, yfn, xs[i], ys[i]);
        float dis2 = computeDistanceBetweenPointAndLine(xfn, yfn, xff, yff, xs[i], ys[i]);
        float dis3 = computeDistanceBetweenPointAndLine(xff, yff, xnf, ynf, xs[i], ys[i]);
        float dis4 = computeDistanceBetweenPointAndLine(xnf, ynf, xnn, ynn, xs[i], ys[i]);
        
        // ????????????
        std::vector<float> dises = {dis1, dis2, dis3, dis4};
        std::vector<size_t> idxes = sortIndexes(dises, true);
        scd += dises[idxes[0]];
    }
    
    return scd;
}

void EuCluster::fitOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                                       visualization_msgs::Marker& marker)
{
    size_t num = pc->points.size();
    assert(num > 0);

    std::vector<float> xs, ys;
    xs.resize(num);
    ys.resize(num);
    for(int i = 0; i < num; i++)
    {
        xs[i] = pc->points[i].x;
        ys[i] = pc->points[i].y;
    }
    
    // ??????????????????
    std::vector<cv::Point> points;
    points.resize(num);
    for(int i = 0; i < num; i++)
    {
        points[i].x = (int)(xs[i] * 100);
        points[i].y = (int)(ys[i] * 100);
    }
    
    // STEP1???????????????
    std::vector<cv::Point> hull;
    convexHull(points, hull, false);
    
    // STEP2??????????????????
    std::vector<cv::Point> approximated_hull;
    approxPolyDP(hull, approximated_hull, 10, true); // ????????????0.1m
    
    // STEP3???????????????????????????m???????????????????????????
    int m = approximated_hull.size() < 4 ? approximated_hull.size() : 4;
    std::vector<float> distances, angles;
    size_t hull_size = approximated_hull.size();
    
    for(int j = 0; j < hull_size; j++)
    {
        float x1 = approximated_hull[j].x;
        float y1 = approximated_hull[j].y;
        float x2 = approximated_hull[(j + 1) % hull_size].x;
        float y2 = approximated_hull[(j + 1) % hull_size].y;
        
        float dx = x2 - x1;
        float dy = y2 - y1;
        
        float dis, ang;
        
        dis = sqrt(pow(dx, 2) + pow(dy, 2));
        distances.push_back(dis);
        
        if(dx == 0) ang = M_PI / 2;
        else ang = atan(dy / dx);
        if(ang < 0) ang += M_PI; // ang?????????[0, M_PI)
        angles.push_back(ang);
    }
    
    std::vector<float> angles_sorted;
    std::vector<size_t> idxes = sortIndexes(distances, false);
    for(int j = 0; j < m; j++)
    {
        angles_sorted.push_back(angles[idxes[j]]);
    }
    
    // STEP4???????????????????????????2D????????????????????????????????????????????????SCD????????????????????????
    float x0m, y0m, lm, wm, phim;
    float scd_min = FLT_MAX;
    
    for(int j = 0; j < m; j++)
    {
        float phi = angles_sorted[j];
        float x0, y0, l, w;
        
        // ??????phi??????x0 y0 l w??????????????????l???w???????????????phi???????????????scd
        float scd = computeBoundingBoxByOrientation(xs, ys, x0, y0, l, w, phi);
        
        if(scd < scd_min)
        {
            scd_min = scd;
            
            x0m = x0;
            y0m = y0;
            lm = l;
            wm = w;
            phim = phi;
        }
    }

    float obj_min_z = std::numeric_limits<float>::max();
    float obj_max_z = -std::numeric_limits<float>::max();

    for(int i = 0; i < num; i++)
    {
        if(pc->points[i].z < obj_min_z)
            obj_min_z = pc->points[i].z;
        if(pc->points[i].z > obj_max_z)
            obj_max_z = pc->points[i].z;
    }

    // ??????????????????
    marker.pose.position.x = x0m;
    marker.pose.position.y = y0m;
    marker.pose.position.z = (obj_min_z + obj_max_z) / 2;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = sin(0.5 * phim);
    marker.pose.orientation.w = cos(0.5 * phim);

    // ??????????????????
    marker.scale.x = lm;
    marker.scale.y = wm;
    marker.scale.z = obj_max_z - obj_min_z;
}

void EuCluster::fitBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                               visualization_msgs::Marker& marker)
{
    // ??????3D?????????
    float obj_min_x = std::numeric_limits<float>::max();
    float obj_max_x = -std::numeric_limits<float>::max();
    float obj_min_y = std::numeric_limits<float>::max();
    float obj_max_y = -std::numeric_limits<float>::max();
    float obj_min_z = std::numeric_limits<float>::max();
    float obj_max_z = -std::numeric_limits<float>::max();

    for(size_t p = 0; p < pc->points.size(); p++)
    {
        if(pc->points[p].x < obj_min_x)
            obj_min_x = pc->points[p].x;
        if(pc->points[p].x > obj_max_x)
            obj_max_x = pc->points[p].x;
        if(pc->points[p].y < obj_min_y)
            obj_min_y = pc->points[p].y;
        if(pc->points[p].y > obj_max_y)
            obj_max_y = pc->points[p].y;
        if(pc->points[p].z < obj_min_z)
            obj_min_z = pc->points[p].z;
        if(pc->points[p].z > obj_max_z)
            obj_max_z = pc->points[p].z;
    }

    float phi = 0;

    // ??????????????????
    marker.pose.position.x = (obj_min_x + obj_max_x) / 2;
    marker.pose.position.y = (obj_min_y + obj_max_y) / 2;
    marker.pose.position.z = (obj_min_z + obj_max_z) / 2;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = sin(0.5 * phi);
    marker.pose.orientation.w = cos(0.5 * phi);

    // ??????????????????
    marker.scale.x = obj_max_x - obj_min_x;
    marker.scale.y = obj_max_y - obj_min_y;
    marker.scale.z = obj_max_z - obj_min_z;
}

void EuCluster::clusterInSegment(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                                 const double& cluster_distance,
                                 visualization_msgs::MarkerArray& objs)
{
    // ?????????????????????
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pc, *pc_2d);
    
    #pragma omp for
    for(size_t i = 0; i < pc_2d->points.size(); i++) {pc_2d->points[i].z = 0;}

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    if(pc_2d->points.size() > 0) {tree->setInputCloud(pc_2d);}

    // ?????????????????????????????????????????????
    std::vector<pcl::PointIndices> local_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> eucluster;
    eucluster.setInputCloud(pc_2d);
    eucluster.setClusterTolerance(cluster_distance);
    eucluster.setMinClusterSize(min_cluster_points_num_);
    eucluster.setMaxClusterSize(max_cluster_points_num_);
    eucluster.setSearchMethod(tree);
    eucluster.extract(local_indices);

    // ????????????????????????
    // local_indices.size()???????????????????????????
    #pragma omp for
    for(size_t i = 0; i < local_indices.size(); i++)
    {
        // ????????????
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_sub(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            pc_sub->points.push_back(pc->points[*pit]);
        }

        // ????????????
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();

        for(size_t p = 0; p < pc_sub->points.size(); p++)
        {
            if(pc_sub->points[p].x < min_x)
                min_x = pc_sub->points[p].x;
            if(pc_sub->points[p].x > max_x)
                max_x = pc_sub->points[p].x;
            if(pc_sub->points[p].y < min_y)
                min_y = pc_sub->points[p].y;
            if(pc_sub->points[p].y > max_y)
                max_y = pc_sub->points[p].y;
        }

        // floor(x)?????????????????????x???????????????
        // ceil(x)????????????x???????????????

        int num_x = ceil((max_x - min_x) / max_cluster_size_);
        int num_y = ceil((max_y - min_y) / max_cluster_size_);

        // ??????????????????
        bool x_oversize = false;
        bool y_oversize = false;

        if(num_x > 1) x_oversize = true;
        if(num_y > 1) y_oversize = true;

        // ???pc_sub???????????????num_x*num_y??????????????????XY?????????????????????????????????????????????????????????????????????
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> pc_refined_array;
        pc_refined_array.resize(num_x, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>(num_y));
        for(size_t nx = 0; nx < num_x; nx++)
        {
            for(size_t ny = 0; ny < num_y; ny++)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
                pc_refined_array[nx][ny] = tmp;
            }
        }

        for(size_t p = 0; p < pc_sub->points.size(); p++)
        {
            size_t x_idx = floor((pc_sub->points[p].x - min_x) / max_cluster_size_);
            size_t y_idx = floor((pc_sub->points[p].y - min_y) / max_cluster_size_);
            pc_refined_array[x_idx][y_idx]->points.push_back(pc_sub->points[p]);
        }

        // ???num_x*num_y???????????????3D?????????
        for(size_t nx = 0; nx < num_x; nx++)
        {
            for(size_t ny = 0; ny < num_y; ny++)
            {
                // ????????????????????????Marker????????????
                visualization_msgs::Marker marker;

                // ??????3D?????????
                if(fit_obb_)
                {
                    if(pc_refined_array[nx][ny]->points.size() == 0) {continue;}
                    fitOrientedBoundingBox(pc_refined_array[nx][ny], marker);
                }
                else
                {
                    fitBoundingBox(pc_refined_array[nx][ny], marker);
                }

                // ???????????????????????????
                if(marker.scale.x < min_cluster_size_ &&
                   marker.scale.y < min_cluster_size_ &&
                   marker.scale.z < min_cluster_size_)
                {
                    continue;
                }
                
                // ??????????????????????????????????????????????????????
                double tolerant_size = max_cluster_size_ / 5;
                if(x_oversize && y_oversize)
                {
                    if(marker.scale.x < tolerant_size && marker.scale.y < tolerant_size) {continue;}
                }
                else if(x_oversize && !y_oversize)
                {
                    if(marker.scale.x < tolerant_size) {continue;}
                }
                else if(y_oversize && !x_oversize)
                {
                    if(marker.scale.y < tolerant_size) {continue;}
                }
                
                objs.markers.push_back(marker);
            }
        }
    }
}

void EuCluster::cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                        visualization_msgs::MarkerArray& objs)
{
    // ???????????????????????????
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_array(seg_num_);
    for(size_t j = 0; j < pc_array.size(); j++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        pc_array[j] = tmp;
    }

    // ????????????????????????????????????????????????????????????????????????????????????
    #pragma omp for
    for(size_t i = 0; i < pc->points.size(); i++)
    {
        float distance = sqrt(pow(pc->points[i].x, 2) + pow(pc->points[i].y, 2));

        for(size_t j = 0; j < pc_array.size(); j++)
        {
            if(distance < seg_distance_[j])
            {
                pc_array[j]->points.push_back(pc->points[i]);
                break;
            }
        }
    }

    // ????????????????????????
    for(size_t j = 0; j < pc_array.size(); j++) {clusterInSegment(pc_array[j], cluster_distance_[j], objs);}
}

void EuCluster::crop(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cropped,
                     const std::vector<float>& edge_left,
                     const std::vector<float>& edge_right)
{
    pcl::ExtractIndices<pcl::PointXYZ> clipper;

    clipper.setInputCloud(pc);
    pcl::PointIndices indices;
    
    #pragma omp for
    for(size_t i = 0; i < pc->points.size(); i++)
    {
        float x = pc->points[i].x;
        float y = pc->points[i].y;
        if(y > x * x * x * edge_right[0] + x * x * edge_right[1] + x * edge_right[2] + edge_right[3] &&
            y < x * x * x * edge_left[0] + x * x * edge_left[1] + x * edge_left[2] + edge_left[3])
        {
            continue;
        }
        indices.indices.push_back(i);
    }

    clipper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    clipper.setNegative(true);
    clipper.filter(*pc_cropped);
}

void EuCluster::callback(const sensor_msgs::PointCloud2ConstPtr in)
{
    ros::Time time_start = ros::Time::now();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_current(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cropped(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*in, *pc_current);

    if(road_info_)
    {
        ros::param::get("~road_edge_left", road_edge_left_);
        ros::param::get("~road_edge_right", road_edge_right_);
        crop(pc_current, pc_cropped, road_edge_left_, road_edge_right_);
    }
    else
    {
        pc_cropped = pc_current;
    }
    
    visualization_msgs::MarkerArray objs;
    cluster(pc_cropped, objs);
    
    for(size_t i = 0; i < objs.markers.size(); i++)
    {
        objs.markers[i].header = in->header;

        // ?????????????????????????????????ID???ID????????????????????????
        // ???????????????????????????ID??????????????????????????????
        objs.markers[i].ns = "obstacle";
        objs.markers[i].id = i;
        
        // ??????????????????
        objs.markers[i].type = visualization_msgs::Marker::CUBE;
        
        // ?????????????????????ADD????????????DELETE?????????
        objs.markers[i].action = visualization_msgs::Marker::ADD;

        // ???????????????????????????????????????alpha??????0
        objs.markers[i].color.r = 0.8f;
        objs.markers[i].color.g = 0.0f;
        objs.markers[i].color.b = 0.0f;
        objs.markers[i].color.a = 0.85;

        objs.markers[i].lifetime = ros::Duration(0.1);
        objs.markers[i].text = ' ';
        std::cout <<objs.markers[i].pose<<std::endl;
    }
    pub_.publish(objs);

    ros::Time time_end = ros::Time::now();

    if(show_objects_num_ || show_time_)
    {
        std::cout << "" << std::endl;
        std::cout << "[points_cluster]" << std::endl;
    }

    if(show_objects_num_)
    {
        std::cout << "Size of objects: " << objs.markers.size() << std::endl;
    }

    if(show_time_)
    {
        std::cout << "Time cost per frame: " << time_end - time_start << "s" << std::endl;
    }
}


