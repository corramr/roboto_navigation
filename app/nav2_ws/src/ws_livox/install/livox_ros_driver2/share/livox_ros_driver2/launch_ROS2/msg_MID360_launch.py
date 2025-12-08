import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

################### user configure parameters ###################
multi_topic   = 0    
data_src      = 0    
publish_freq  = 10.0 
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

# Network configuration
ethernet_interface = 'enx3c18a0255db5'
host_ip = '192.168.1.5'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
# Assuming your config is relative to this launch file
user_config_path = os.path.join(cur_path, '../config/MID360_config.json')

# !!! IMPORTANT: Set the path to the python script we created above !!!
path_to_python_converter = '/home/livox/livox_republisher.py'
#################################################################

def generate_launch_description():
    
    # 1. Parameters for the Driver (xfer_format = 1 for Custom Msg)
    livox_ros2_params_custom = [
        {"xfer_format": 1},  # Force Custom format
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    # 2. Configure network interface
    configure_network = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'ip link set {ethernet_interface} up && '
            f'ip addr flush dev {ethernet_interface} && '
            f'ip addr add {host_ip}/24 dev {ethernet_interface}'
        ],
        name='configure_livox_network',
        output='screen'
    )

    # 3. The Main Driver Node (Hardware Interface)
    livox_driver_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='livox_ros_driver2',
                executable='livox_ros_driver2_node',
                name='livox_lidar_driver',
                output='screen',
                parameters=livox_ros2_params_custom,
                remappings=[
                    ('/livox/lidar', '/livox/lidar'), # Publishes CustomMsg here
                ]
            )
        ]
    )


    return LaunchDescription([
        configure_network,
        livox_driver_node,
    ])