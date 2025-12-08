from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([            
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        ])
    )

    pointcloud_converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_converter'),
                'launch',
                'launch.py'
            ])
        ])
    )
    
    laserscan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pointcloud_to_laserscan'),
                'launch',
                'livox_launch.py'
            ])
        ])
    )

    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),  # Use the actual package name
                'launch',
                'mapping.launch.py'
            ])
        ])
    )
        
    return LaunchDescription([
        livox_driver,
        pointcloud_converter,
        laserscan,
        odometry,
    ])