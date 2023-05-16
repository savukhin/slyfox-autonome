sudo apt-get install ros-humble-imu-tools

mkdir deps
cd deps

mkdir imu_tools
# cd imu_tools
git clone git@github.com:CCNYRoboticsLab/imu_tools.git imu_tools -b humble
# cd ..

mkdir -p multiwii_ros2/src
# cd multiwii_ros2/src
git clone https://github.com/christianrauch/msp
git clone https://github.com/christianrauch/multiwii_ros2
# cd ..

https://github.com/IntelRealSense/realsense-ros

ros2 launch rtabmap_ros rtabmap.launch.py