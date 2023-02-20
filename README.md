## lidars_ws工作空间
### 简介
这是一个多激光雷达点云基本处理的包，包含了去除地面、聚类、跟踪等功能。
multi_lidars_calibration-NDT为多激光雷达标定的功能包
### 安装编译启动
```
	git clone git@github.com:huashu996/lidar_fusion_ws.git --recursive
	cd lidars_ws
	catkin_make
	source devel/setup.bash
	cd ..
	./all_launch.bash
```
## 一、雷达驱动包
    HesaiLidar-ros功能包 —— 40线pandar驱动
    ros_rslidar功能包 —— 16线RS驱动
- 启动三个雷达
```
    roslaunch rslidar_pointcloud three_lidar.launch
```
- 发布话题
```
    <param name="left_lidar_topic" value="/ns2/rslidar_points" />
    <param name="right_lidar_topic" value="/ns1/rslidar_points" />
    <param name="pandar_lidar_topic" value="/ns3/pandar_points" />
```
    TF坐标系  /tf
## 二、点云后处理
### 1、lidar_fusion功能包 —— 40线雷达和2个16线雷达融合成一个节点
- 接收话题
```
    <param name="left_lidar_topic" value="/ns2/rslidar_points" />
    <param name="right_lidar_topic" value="/ns1/rslidar_points" />
    <param name="pandar_lidar_topic" value="/ns3/pandar_points" />
```
   
运用TF坐标系关系融合 /tf

- 发布话题
``` 
    <param name="fusion_topic" value="/fusion_rslidar_points" /> 融合的原始点云
    <param name="fusion_topic" value="/regr_fusion_rslidar_points" />
    启动
    roslaunch lidar_fusion lidar_fusion.launch
```

### 2、points_ground_filter功能包 —— 去除地面点云
- 接收话题
```
    <param name="sub_topic" value="/regr_fusion_rslidar_points"/>
```
- 发布话题
```
    <param name="pub_ground_topic" value="/pandar_points_ground"/>
    <param name="pub_no_ground_topic" value="/pandar_points_no_ground"/>
```
- 启动
```
    roslaunch points_ground_filter points_ground_filter.launch
```
### 3、deal_point功能包 —— 对点云进行范围上的预处理
- 接收话题
``` 
    <param name="sub_topic" value="/pandar_points_no_ground"/>
```
- 发布话题
``` 
    <param name="pub_no_ground_topic" value="/fusion_points_no_ground"/>
```
- 启动
```
    roslaunch deal_point test.launch
```
   
### 4、point_cluster功能包 —— 点云聚类
- 接收话题
```
    <param name="sub_topic" value="/fusion_points_no_ground" />
```
- 发布话题
```
    <param name="pub_topic" value="/objects" />
```
- 启动
```
    roslaunch points_cluster points_cluster.launch
```
### 5、multiple_target_tracking功能包 —— 点云聚类跟踪
- 接收话题
```
    <param name="sub_topic" value="/objects"/>
```
- 发布话题
```
    <param name="pub_topic" value="/objects_tracked"/>
    <param name="pub_topic_obstacle_array" value="/obstacles_array"/>
```
- 启动
```
    roslaunch multiple_target_tracking multiple_target_tracking.launch
```
### 6、points_freespace_extractor功能包 —— 可行域划分
- 接收话题
```
    <param name="sub_topic" value="/regr_fusion_rslidar_points"/>
```
- 发布话题
```
    <param name="pub_marker_topic" value="/feasible_region"/>
```
- 启动
```
    roslaunch points_freespace_extractor points_freespace_extractor.launch
```
./all_launch.bash 启动所有节点
    
    
    
    
    
    
    
    
