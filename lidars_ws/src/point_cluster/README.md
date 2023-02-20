# points_cluster

ROS package for clustering points

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/points_cluster.git
   cd ..
   catkin_make
   ```
   
## 参数配置
 - 修改`points_cluster/launch/points_cluster.launch`
   ```Shell
   <param name="sub_topic" value="/pandar_points_no_ground" />
   <param name="pub_topic" value="/objects" />

   <param name="show_objects_num" value="true" />
   <param name="show_time" value="true" />

   <param name="fit_obb" value="true" />
   
   <param name="min_cluster_points_num" value="5" />
   <param name="max_cluster_points_num" value="1000" />
   
   <param name="min_cluster_size" value="0.1" />
   <param name="max_cluster_size" value="4" />
   
   <param name="seg_num" value="3" />
   <rosparam param="seg_distance" > [20, 40, 60] </rosparam>
   <rosparam param="cluster_distance" > [0.25, 0.5, 0.75] </rosparam>

   <param name="road_info" value="true" />
   <rosparam param="road_edge_left" > [0, 0, 0, 50.75] </rosparam>
   <rosparam param="road_edge_right" > [0, 0, 0, -50.25] </rosparam>
   ```
    - `sub_topic`指明订阅的点云话题。
    - `pub_topic`指明发布的聚类结果话题，类型为MarkerArray，可以通过RVIZ查看。
    - `fit_obb`如果设置为true，则对聚类目标拟合带方向包围盒，否则拟合不带方向包围盒。
    - `min_cluster_points_num`和`max_cluster_points_num`为聚类点云数量限制。
    - `min_cluster_size`和`max_cluster_size`为聚类点云尺寸限制，单位为米。
    - `seg_distance`和`cluster_distance`为不同距离范围下的聚类阈值，单位为米。
    - `road_edge_left`和`road_edge_right`为道路边缘的曲线参数，曲线形同y=ax^3+bx^2+cx+d。

## 运行
 - 启动`points_cluster`
   ```Shell
   roslaunch points_cluster points_cluster.launch
   ```

