import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    usb_cam_pkg = FindPackageShare('cablebee').find('cablebee')
    webcam_cal_path = os.path.join(usb_cam_pkg, 'webcamcal', 'ost.yaml')
    widecam_cal_path = os.path.join(usb_cam_pkg, 'widecamcal', 'ost.yaml')

    ld = LaunchDescription([
        # declare_rviz_config_file_cmd,
        Node(package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            namespace='camera',
            parameters=[{"camera_info_url": "file://" + webcam_cal_path, # todo ?
             "camera_name": "webcam",
             "frame_id": "camera"}]),
        Node(name='tf2_ros_odom_bl',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'odom', 'base_link']),
        Node(name='tf2_ros_bl_camera',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_link', 'camera'])
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen'
        # )
        ])

    # bee_rviz = os.path.join(bee_pkg, 'launch', 'default.rviz')

    # rviz_config_file = LaunchConfiguration('rviz_config')
    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     'rviz_config',
    #     default_value=bee_rviz,
    #     description='Full path to the RVIZ config file to use')

    return ld