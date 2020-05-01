#! /bin/bash
source /opt/ros/kinetic/setup.bash
source ~/cartographer/install_isolated/setup.bash
source ~/mrobot/devel/setup.bash
source /usr/local/YOUIBOT/install/setup.bash

roslaunch youibot_node youibot_nodelet_trans.launch &
sleep 3
roslaunch mm_powerup mm_ini.launch &
roslaunch youibot_charge_lidar get_charge_pose_trans.launch &
roslaunch youibot_move_action youibot_move_action.launch &
sleep 1
roslaunch youi_navigation robot_navigation.launch &
sleep 3
echo 'mrobot2018' | sudo -S sudo /usr/local/nginx/sbin/nginx &
rosrun mm_video_streamer streamer_camera_node &
#roslaunch mm_arm_control ur10_test_run.launch &
sleep 3
roslaunch mm_visual_postion visual_wrapper_and_apps.launch &
roslaunch mm_robot_decision mm_decision.launch 
read

