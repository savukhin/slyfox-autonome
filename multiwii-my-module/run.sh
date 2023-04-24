. install/local_setup.sh

ros2 run multiwii_node multiwii_node --ros-args \
    -p sub/imu:=0.01 -p sub/motor:=0.1 -p sub/rc:=0.1 -p sub/attitude:=0.1 -p sub/altitude:=0.1 -p sub/analog:=0.1 -p sub/voltage:=1.0 -p sub/current:=1.0 -p sub/battery:=1.0 -p device_path:=/dev/ttyACM0