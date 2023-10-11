. install/setup.sh

export LC_NUMERIC="en_US.UTF-8"

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# !! Warning: first time to load gazebo model takes really long time
ros2 launch proxy_holder gazebo.py headless:=False gazebo_timeout:=500
# ros2 launch proxy_holder launch_full.py rviz:=true rtabmapviz:=true rx_serial_type:=dummy fcu_serial_type:=dummy