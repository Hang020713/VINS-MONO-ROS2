import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_pkg_share = get_package_share_directory('config_pkg')
    default_config = os.path.join(
        config_pkg_share,
        'config', 'raspberry_pi_5', 'imx708_px4_vio.yaml')
    vins_folder = config_pkg_share + os.sep

    config_file = LaunchConfiguration('config_file')
    imu_input_topic = LaunchConfiguration('imu_input_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Completed VINS OpenCV YAML config file'),
        DeclareLaunchArgument(
            'imu_input_topic',
            default_value='/imu/data',
            description='sensor_msgs/Imu topic published by the PX4 converter'),
        LogInfo(msg=['Pi 5 VIO config: ', config_file]),
        Node(
            package='vins_estimator',
            executable='imu_clock_bridge',
            name='imu_clock_bridge',
            output='screen',
            parameters=[{
                'input_topic': imu_input_topic,
                'output_topic': '/imu/data_vio',
                'input_clock': 'system',
            }]),
        Node(
            package='feature_tracker',
            executable='feature_tracker',
            name='feature_tracker',
            namespace='feature_tracker',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'vins_folder': vins_folder,
            }]),
        Node(
            package='vins_estimator',
            executable='vins_estimator',
            name='vins_estimator',
            namespace='vins_estimator',
            output='screen',
            parameters=[{'config_file': config_file}]),
    ])
