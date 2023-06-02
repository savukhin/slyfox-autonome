. /opt/ros/humble/setup.sh
. install/local_setup.sh

# ros2 run viso2_ros stereo_odometer namespace:=/camera/color sensor_frame_id:='camera_color_optical_frame'
ros2 launch viso2_ros demo.launch.py namespace:=/camera/color sensor_frame_id:='camera_color_optical_frame'
