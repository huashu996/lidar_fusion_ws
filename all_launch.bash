#!/bin/bash
source /home/neousys/cxl/fusion_lidar/lidars_ws/devel/setup.bash


gnome-terminal --window --tab -e "roslaunch rslidar_pointcloud three_lidar.launch"
sleep 2

gnome-terminal --window --tab -e "roslaunch lidars_fusion lidars_fusion.launch "
sleep 2

gnome-terminal --window --tab -e " roslaunch points_ground_filter points_ground_filter.launch"
sleep 2
gnome-terminal --window --tab -e " roslaunch deal_point test.launch"
sleep 2
gnome-terminal --window --tab -e "roslaunch points_cluster points_cluster.launch"
sleep 2

gnome-terminal --window --tab -e "roslaunch multiple_target_tracking multiple_target_tracking.launch"
sleep 2

gnome-terminal --window --tab -e "roslaunch points_freespace_extractor points_freespace_extractor.launch"
sleep 2

gnome-terminal --window --tab -e "rviz"
sleep 2

wait
exit 0
