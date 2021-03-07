import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bee_pkg = FindPackageShare('cablebee').find('cablebee')
    bee_urdf = os.path.join(bee_pkg, 'urdf', 'astrobee.urdf')
    bee_rviz = os.path.join(bee_pkg, 'launch', 'default.rviz')
    with open(bee_urdf, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    rviz_config_file = LaunchConfiguration('rviz_config')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=bee_rviz,
        description='Full path to the RVIZ config file to use')
    
    ld = LaunchDescription([
        declare_rviz_config_file_cmd,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[rsp_params]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen', parameters=[rsp_params, {'use_gui': True}]),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
        ])

    return ld