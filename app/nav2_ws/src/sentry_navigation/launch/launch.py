from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

# Import SetRemap
from launch_ros.actions import SetRemap

def generate_launch_description():

    # define 'simulation' argument
    simulation_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='False',
        description='If true, launch only the nav2 stack. If false, launch both stacks.'
    )

    # import launch for nav2 stack
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([            
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'sentry_simulation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_simulator': LaunchConfiguration('use_simulator'), 
            'headless': 'False',
            'use_sim_time': LaunchConfiguration('use_simulator')
        }.items()
    )

    # Wrap the nav2_stack in a GroupAction to apply the remapping globally to this stack
    nav2_group = GroupAction(
        actions=[
            SetRemap(src='cmd_vel', dst='cmd_vel_NAV'),
            nav2_stack
        ]
    )

    # import launching for sensor stack
    sensor_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sensor_launcher'),
                'launch',
                'launch.py'
            ])
        ]),
        condition = UnlessCondition(LaunchConfiguration('use_simulator'))
    )

    # start node to send cmd_vel to microcontroller
    cmd_vel_to_micro = Node(
        package='cmd_vel_serial',
        executable='usb_c_final',
        name='cmd_vel_serial',
        output='log',
        condition=UnlessCondition(LaunchConfiguration('use_simulator'))
    )


    return LaunchDescription([
        simulation_arg,
        nav2_group,   
        sensor_stack,
        cmd_vel_to_micro
    ])