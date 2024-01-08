import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory 
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

proxy_holder_params = [
    {'name': 'rx_serial_type',                  'default': 'usb', 'description': 'dummy or usb - type of RX serial'},
    {'name': 'rx_serial',                  'default': '/dev/USB0', 'description': 'path to RX serial if USB'},
    {'name': 'rx_serial_baud',                  'default': '115200', 'description': 'RX serial baud rate if USB'},
    {'name': 'fcu_serial_type',                  'default': 'usb', 'description': 'dummy or usb - type of FCU serial'},
    {'name': 'fcu_serial',                  'default': '/dev/USB0', 'description': 'path to FCU serial if USB'},
    {'name': 'fcu_serial_baud',                  'default': '115200', 'description': 'FCU serial baud rate if USBl'},

    {'name': 'pid_p_throttle',                  'default': '1.0', 'description': 'P of throttle PID'},
    {'name': 'pid_i_throttle',                  'default': '1.0', 'description': 'I of throttle PID'},
    {'name': 'pid_d_throttle',                  'default': '1.0', 'description': 'D of throttle PID'},
    {'name': 'pid_p_pitch',                  'default': '1.0', 'description': 'P of pitch PID'},
    {'name': 'pid_i_pitch',                  'default': '1.0', 'description': 'I of pitch PID'},
    {'name': 'pid_d_pitch',                  'default': '1.0', 'description': 'D of pitch PID'},
    {'name': 'pid_p_roll',                  'default': '1.0', 'description': 'P of roll PID'},
    {'name': 'pid_i_roll',                  'default': '1.0', 'description': 'I of roll PID'},
    {'name': 'pid_d_roll',                  'default': '1.0', 'description': 'D of roll PID'},


    {'name': 'hold_channel',                  'default': '5', 'description': 'Channel of hold'},
    {'name': 'yaw_channel',                  'default': '4', 'description': 'Channel of hold yaw'},
    {'name': 'pitch_channel',                  'default': '2', 'description': 'Channel of pitch'},
    {'name': 'roll_channel',                  'default': '1', 'description': 'channel of roll'},
    {'name': 'throttle_channel',                  'default': '3', 'description': 'Channel of throttle'},
    {'name': 'hold_transient_duration_ms',                  'default': '100', 'description': 'Count in ms to hold change state'},
]

configurable_parameters = [
    {'name': 'fcu_msp_path',                  'default': '/dev/ttyACM0', 'description': 'path to FCU MultiWii (MSP) serial'},
    {'name': 'use_mag_in_filter',                  'default': "false", 'description': 'path to FCU serial'},

    # rtabmap_ros
    {'name': 'rviz',                  'default': 'false', 'description': 'rtabmap: show rviz'},
    {'name': 'rtabmapviz',                  'default': 'false', 'description': 'rtabmap: show rtabmapviz'},

] + proxy_holder_params


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():
    rtabmap_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch'),
            '/rtabmap.launch.py']),
            launch_arguments={
                "rtabmap_args": "--delete_db_on_start",
                "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                "rgb_topic": "/camera/color/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "approx_sync": "false",
                "frame_id": "camera_color_optical_frame",
                "wait_for_transform": "0.5",
                # "imu_topic": "/imu/data",
                # "wait_imu_to_init": "true",
                "rviz":  LaunchConfiguration("rviz"),
                "rtabmapviz": LaunchConfiguration("rtabmapviz"),
            }.items()
        )
    
    rtabmap_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch'),
            '/rtabmap.launch.py']),
            launch_arguments={
                "rtabmap_args": "--delete_db_on_start",
                "depth_topic": "/depth_camera_controller/depth/image_raw",
                "rgb_topic": "/depth_camera_controller/image_raw",
                "camera_info_topic": "/depth_camera_controller/camera_info",
                "approx_sync": "false",
                "frame_id": "base_footprint",
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                "wait_for_transform": "0.5",
                "imu_topic": "/imu",
                "wait_imu_to_init": "true",
                "rviz":  "true",
                "rtabmapviz": "true",
            }.items()
        )
    
    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(
            get_package_share_directory('foxglove_bridge'), 'launch'),
            '/foxglove_bridge_launch.xml']),
            launch_arguments={
                "port": "8765"
            }.items()
        )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # Node(
        #     package='multiwii_node',
        #     executable='multiwii_node',
        #     parameters=[{
        #         "sub/imu": 0.01,
        #         "sub/motor": 0.1,
        #         "sub/rc": 0.1,
        #         "sub/attitude": 0.1,
        #         "sub/altitude": 0.1,
        #         "sub/analog": 0.1,
        #         "sub/voltage": 1.0,
        #         "sub/current": 1.0,
        #         "sub/battery": 1.0,
        #         "device_path": LaunchConfiguration("fcu_msp_path"),
        #     }]
        # ),
        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     parameters=[{
        #         "use_mag": LaunchConfiguration("use_mag_in_filter"),
        #     }]
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'align_depth.enable': "true"}.items()
            ),
        rtabmap_launch_1,
        foxglove_bridge_launch,
        # Node(
        #     package='proxy_holder',
        #     executable='proxy_holder_node',
        #     parameters=[ { param["name"]: LaunchConfiguration(param["name"]) } for param in proxy_holder_params]
        # )
    ])