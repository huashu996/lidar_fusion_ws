# multiple_target_tracking

ROS package for multiple target tracking

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/multiple_target_tracking.git
   git clone https://github.com/shangjie-li/perception_msgs.git
   cd ..
   catkin_make
   ```
   
## 参数配置
 - 修改`multiple_target_tracking/launch/multiple_target_tracking.launch`
   ```Shell
   <param name="sub_topic" value="/objects"/>
   <param name="pub_topic" value="/objects_tracked"/>
   <param name="pub_topic_obstacle_array" value="/obstacles_array"/>
   <param name="frame_id" value="pandar"/>
        
   <param name="show_objects_num" value="true"/>
   <param name="show_time" value="true"/>

   <param name="time_interval" value="0.1"/>
   <param name="gate_threshold" value="400"/>
   <param name="blind_update_limit" value="1"/>
        
   <param name="sigma_ax" value="1"/>
   <param name="sigma_ay" value="1"/>
   <param name="sigma_ox" value="0.1"/>
   <param name="sigma_oy" value="0.1"/>
        
   <param name="min_scale" value="1.5"/>
   <param name="max_scale" value="6.0"/>
   <param name="min_height" value="0.5"/>
   <param name="max_height" value="2.5"/>
   ```
    - `sub_topic`指明订阅的MarkerArray类型话题。
    - `pub_topic`指明发布的MarkerArray类型话题，可以通过RVIZ查看。
    - `pub_topic_obstacle_array`指明发布的ObstacleArray类型话题。
    - `time_interval`为时间间隔。
    - `gate_threshold`为跟踪门阈值。
    - `blind_update_limit`为中断更新的次数限制。
    - `sigma_ax` `sigma_ay` `sigma_ox` `sigma_oy`指明过程噪声和量测噪声。
    - `min_scale`和`max_scale`指明对跟踪目标的尺寸限制。
    - `min_height`和`max_height`指明对跟踪目标的高度限制。

## 运行
 - 启动`multiple_target_tracking`
   ```Shell
   roslaunch multiple_target_tracking multiple_target_tracking.launch
   ```

