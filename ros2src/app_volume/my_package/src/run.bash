ros2 run rviz2 rviz2 -d install/realsense2_camera/share/realsense2_camera/launch/default.rviz
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true