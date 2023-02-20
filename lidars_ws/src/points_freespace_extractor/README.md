# points_freespace_extractor

ROS package for extracting freespace region

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/points_freespace_extractor.git
   cd ..
   catkin_make
   ```
   
## 参数配置
 - 修改`points_freespace_extractor/launch/points_freespace_extractor.launch`
   ```Shell
   <param name="sub_topic" value="/pandar_points_processed"/>
   <param name="pub_marker_topic" value="/feasible_region"/>
        
   <param name="show_points_size" value="true"/>
   <param name="show_time" value="true"/>

   <param name="sensor_height" value="2.0"/>
   <param name="radius_divider" value="0.15"/>
   <param name="theta_divider" value="0.4"/>
   <param name="curb_height_threshold" value="0.2"/>
   <param name="general_slope_threshold" value="4.0"/>
   <param name="passable_width" value="2.0"/>
   ```
    - `sub_topic`指明订阅的点云话题。
    - `pub_marker_topic`指明发布的可行域话题，类型为Marker，可以通过RVIZ查看。
    - `sensor_height`为传感器距地面高度，单位为米。
    - `radius_divider`为径向距离单元的长度，单位为米。
    - `theta_divider`为激光雷达水平角分辨率，单位为度。
    - `curb_height_threshold`为路沿高度阈值，单位为米。
    - `general_slope_threshold`为全局坡度阈值，单位为度。
    - `passable_width`为可通行宽度，单位为米。

## 运行
 - 启动`points_freespace_extractor`
   ```Shell
   roslaunch points_freespace_extractor points_freespace_extractor.launch
   ```

## 说明
 - 当激光雷达水平安装时，可行域提取效果较好。
