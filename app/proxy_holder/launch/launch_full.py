import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory 
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource


configurable_parameters = [
    {'name': 'fcu_msp_path',                  'default': '/dev/ttyACM0', 'description': 'path to FCU MultiWii (MSP) serial'},
    {'name': 'use_mag_in_filter',                  'default': "false", 'description': 'path to FCU serial'},
    {'name': 'rx_serial',                  'default': 'asdasd', 'description': 'path to FCU serial'},
    {'name': 'fcu_serial',                  'default': 'asdasd', 'description': 'path to FCU serial'},
    {'name': 'rviz',                  'default': 'false', 'description': 'path to FCU serial'},
    {'name': 'rtabmapviz',                  'default': 'false', 'description': 'path to FCU serial'},
    {'name': 'wait_for_transform',                  'default': '0.5', 'description': 'path to FCU serial'},
    # {'name': 'fcu_serial',                  'default': 'asdasd', 'description': 'path to FCU serial'},
    # {'name': 'fcu_serial',                  'default': 'asdasd', 'description': 'path to FCU serial'},
]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        Node(
            package='multiwii_node',
            executable='multiwii_node',
            parameters=[{
                "sub/imu": 0.01,
                "sub/motor": 0.1,
                "sub/rc": 0.1,
                "sub/attitude": 0.1,
                "sub/altitude": 0.1,
                "sub/analog": 0.1,
                "sub/voltage": 1.0,
                "sub/current": 1.0,
                "sub/battery": 1.0,
                "device_path": LaunchConfiguration("fcu_msp_path"),
            }]
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            parameters=[{
                "use_mag": LaunchConfiguration("use_mag_in_filter"),
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'align_depth.enable': "true"}.items()
            ),
        IncludeLaunchDescription(
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
                    "wait_for_transform": LaunchConfiguration("wait_for_transform"),
                    "imu_topic": "/imu/data",
                    "wait_imu_to_init": "true",
                    "rviz":  LaunchConfiguration("rviz"),
                    "rtabmapviz": LaunchConfiguration("rtabmapviz"),
                }.items()
            ),
        # Node(
        #     package='proxy_holder',
        #     executable='proxy_holder_node',
        #     name='proxy_holder',
        #     parameters=[{
        #         "rx_serial": LaunchConfiguration("rx_serial"),
        #         "fcu_serial": LaunchConfiguration("fcu_serial"),
        #     }]
        # )
    ])