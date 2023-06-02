. /opt/ros/humble/setup.sh
ros2 pkg executables imu_filter_madgwick

# ros2 run imu_complementary_filter complementary_filter_node --ros-args -p use_mag:=false
# ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p /imu/data_raw:=/imu/data_raw -p /imu/data:=/rtabmap/imu
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false
# ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
#     -p use_mag:=false \
#     -p publish_tf:=false \
#     -p world_frame:="enu" \
#     -p /imu/data_raw:=/imu/data_raw