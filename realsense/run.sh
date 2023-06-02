. install/local_setup.sh


ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true

colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential --parallel-workers 1

colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1



MAKEFLAGS="-j1 -l1" colcon build --symlink-install --merge-install --event-handlers desktop_notification- --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential --parallel-workers 1

