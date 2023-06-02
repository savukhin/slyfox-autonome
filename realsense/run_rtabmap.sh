. /opt/ros/humble/setup.sh

ros2 pkg executables realsense2_camera

# ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true

ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true