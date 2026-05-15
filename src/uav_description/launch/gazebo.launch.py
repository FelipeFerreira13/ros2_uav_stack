from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('uav_description')

    urdf_file = os.path.join(pkg_share, 'urdf', 'uav.urdf.xacro')

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                # world,
                '-s', 'libgazebo_ros_init.so',      # initializes Gazebo’s ROS interface, especially simulated time and ROS node integration
                '-s', 'libgazebo_ros_factory.so'    # lets ROS spawn models into Gazebo
            ],
            output='screen'
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('gazebo_ros'),
        #             'launch',
        #             'gazebo.launch.py'
        #         )
        #     ),
        #     launch_arguments={
        #         'gui': 'true',
        #         'verbose': 'true',
        #         'gui_required': 'false'
        #     }.items()
        # ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': True
            }]
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'f450_uav'
            ],
            output='screen'
        )
    ])