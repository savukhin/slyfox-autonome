. /opt/ros/humble/setup.sh

ros2 pkg executables robot_localization

ros2 run robot_localization ekf_node --ros-args --params-file ekf.yaml
# ros2 launch robot_localization ekf.launch.py imu0:=/imu/data odom0:=/camera/color/odometry --params-file demo_params.yaml