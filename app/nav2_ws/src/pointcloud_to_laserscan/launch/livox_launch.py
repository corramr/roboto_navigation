from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_scan', '--child-frame-id', 'livox_frame'
            ]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_scan',
                'transform_tolerance': 0.01,
                'min_height': 0.1,
                'max_height': 0.7,
                'angle_min': -3.14159,    # -180 degrees
                'angle_max': 3.14159,     # +180 degrees
                'angle_increment': 0.004, #0.0087,  # 0.5 degrees
                'scan_time': 0.1,  # NOT IMPO: just metadata info
                'range_min': 0.1,
                'range_max': 10.0,
                'inf_epsilon': 1.0,
                'use_inf': True
            }],
            remappings=[
                ('cloud_in', '/livox/lidar_pc2'),
                ('scan', '/scan')
            ]
        )
    ])