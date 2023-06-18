LC_NUMERIC="en_US.UTF-8"

. /opt/ros/humble/setup.sh 
. ./multiwii-my-module/install/setup.sh
. ./app/install/setup.sh

ros2 launch proxy_holder launch_full.py rviz:=true rtabmapviz:=true
