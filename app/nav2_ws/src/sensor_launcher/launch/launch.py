from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    static_tf_node_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='screen',
        # Arguments: x, y, z, yaw, pitch, roll, frame_id, child_frame_id
        # Note: Euler angles (yaw, pitch, roll) are in radians.
        # Alternatively, you can use quaternions: x, y, z, qx, qy, qz, qw, frame_id, child_frame_id
        arguments=['0.0', '0.0', '0.06', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
    )

    static_tf_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='screen',
        # Arguments: x, y, z, yaw, pitch, roll, frame_id, child_frame_id
        # Note: Euler angles (yaw, pitch, roll) are in radians.
        # Alternatively, you can use quaternions: x, y, z, qx, qy, qz, qw, frame_id, child_frame_id
        arguments=['0.0', '0.0', '0.45', '0.0', '0.0', '3.14159', 'base_link', 'lidar_link']
    )

    static_tf_node_3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='screen',
        # Arguments: x, y, z, yaw, pitch, roll, frame_id, child_frame_id
        # Note: Euler angles (yaw, pitch, roll) are in radians.
        # Alternatively, you can use quaternions: x, y, z, qx, qy, qz, qw, frame_id, child_frame_id
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'lidar_link', 'livox_frame']
    )

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


    lidar_odometry = GroupAction([
        SetRemap(src='/tf', dst='/tf_fast_lio'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio'),
                    'launch',
                    'mapping.launch.py'
                ])
            ])
        )
    ])

    
    # add kalman filter node
    # Path to your ekf.yaml config file
    ekf_config_path = os.path.join(
        get_package_share_directory('sensor_launcher'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('/odometry/filtered', '/odom')]
        )
        
    return LaunchDescription([
        # static_tf_node_1,
        # static_tf_node_2,
        # static_tf_node_3,
        livox_driver,
        pointcloud_converter,
        laserscan,
        lidar_odometry,
        ekf_node
    ])