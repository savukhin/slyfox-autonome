rm -rf log install build
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --event-handlers desktop_notification- --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential --parallel-workers 1