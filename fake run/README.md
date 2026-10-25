# 2.2 SLAM

## Work in Progress

> **Status: outline, not yet written.** Needs an author from the SLAM team



```sh
source sim/install/setup.bash

ros2 launch eufs_sim2 eufs_sim2.launch.py #confirm w/ ros2 topic list

ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
#python3 publish_centerline_markers.py #just for visualise useless?
#ros2 topic echo /camera/cones --once  /odom /imu/data /plugin/gnss_plugin/fix  #test
ros2 service call /set_mission eufs_msgs/srv/SetMission "{mission: 4}" 

ros2 service call /go std_srvs/srv/Trigger "{}" 

python3 fixed_path_driver.py  #subscribes /odom, lookahead 4 m, target 5 m/s, steering = atan(2*L*y/Ld^2)
python3 camera_opencv_viewer.py #angle=atan2(y,x), u = W/2 - fx*tan(angle), radius scaled by 1/x



ros2 service call /reset std_srvs/srv/Trigger "{}"
```