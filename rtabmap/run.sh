. /opt/ros/humble/setup.sh

ros2 pkg executables rtabmap_launch

LC_NUMERIC="en_US.UTF-8"

ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info  \
    approx_sync:=false \
    frame_id:=camera_color_optical_frame \
    wait_for_transform:=0.5 \
    imu_topic:=/imu/data \
    wait_imu_to_init:=true \
    rviz:=false \
    rtabmapviz:=true
