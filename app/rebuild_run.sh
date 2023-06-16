rm -rf build install log

MAKEFLAGS="-j1 -l1" colcon build --symlink-install --merge-install --event-handlers desktop_notification- --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential --parallel-workers 1
# MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential --parallel-workers 1
. install/setup.sh
ros2 run proxy_holder my_node
# ros2 run proxy_holder proxy_holder_node