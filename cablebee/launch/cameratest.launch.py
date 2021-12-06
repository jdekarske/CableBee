import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    cablebee_pkg = FindPackageShare('cablebee').find('cablebee')
    webcam_cal_path = os.path.join(cablebee_pkg, 'webcamcal', 'ost.yaml')
    widecam_cal_path = os.path.join(cablebee_pkg, 'widecamcal', 'ost.yaml')
    rviz_config_file = os.path.join(cablebee_pkg, 'launch', 'cameratest.rviz')

    ld = LaunchDescription([
        Node(package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            namespace='camera',
            parameters=[{"camera_info_url": "file://" + widecam_cal_path, # todo ?
             "camera_name": "widecam",
             "frame_id": "camera",
             "video_device": "/dev/video2"}]),
        Node(name='tf2_ros_odom_bl',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link']),
            # arguments=['1.316', '2.594', '0.952', '3.1415', '0.0', '-1.5708', 'odom', 'base_link']),
        Node(name='tf2_ros_bl_camera',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_link', 'camera']),

        DeclareLaunchArgument('transport', default_value='raw'),
        DeclareLaunchArgument('fiducial_len', default_value='0.198'),
        DeclareLaunchArgument('dictionary', default_value='0'),
        DeclareLaunchArgument('do_pose_estimation', default_value='True'),
        DeclareLaunchArgument('vis_msgs', default_value="False"),
        DeclareLaunchArgument('ignore_fiducials', default_value=""),
        DeclareLaunchArgument('fiducial_len_override', default_value=""),
        # Node(
        #     package='aruco_detect',
        #     namespace='aruco_detect',
        #     executable='aruco_detect',
        #     name='aruco_detect',
        #     parameters=[
        #         {"image_transport":LaunchConfiguration('transport')},
        #         {"publish_images":True},
        #         {"fiducial_len":LaunchConfiguration('fiducial_len')},
        #         {"dictionary":LaunchConfiguration('dictionary')},
        #         {"do_pose_estimation":LaunchConfiguration('do_pose_estimation')},
        #         {"vis_msgs":LaunchConfiguration('vis_msgs')},
        #         {"ignore_fiducials":LaunchConfiguration('ignore_fiducials')},
        #         {"fiducial_len_override":LaunchConfiguration('fiducial_len_override')}
        #     ],
        # ),

        DeclareLaunchArgument('camera', default_value='/camera'),
        DeclareLaunchArgument('use_fiducial_area_as_weight', default_value='false'),
        DeclareLaunchArgument('weighting_scale', default_value='1e9'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_interval', default_value='0.2'),
        DeclareLaunchArgument('future_date_transforms', default_value='0.0'),
        DeclareLaunchArgument('publish_6dof_pose', default_value='false'),
        DeclareLaunchArgument('systematic_error', default_value='0.01'),
        DeclareLaunchArgument('covariance_diagonal', default_value=''),
        DeclareLaunchArgument('read_only_map', default_value='False'),
        # Node(
        #     package='fiducial_slam',
        #     namespace='fiducial_slam',
        #     executable='fiducial_slam_node',
        #     name='fiducial_slam_node',
        #     parameters=[
        #         {"multi_error_theshold": -1.0},
        #         {"map_file":"/.ros/slam/map.txt"},
        #         {"initial_map_file":""},
        #         {"use_fiducial_area_as_weight":LaunchConfiguration('use_fiducial_area_as_weight')},
        #         {"weighting_scale":LaunchConfiguration('weighting_scale')},
        #         {"map_frame":LaunchConfiguration('map_frame')},
        #         {"odom_frame":LaunchConfiguration('odom_frame')},
        #         {"base_frame":LaunchConfiguration('base_frame')},
        #         {"publish_tf":LaunchConfiguration('publish_tf')},
        #         {"tf_publish_interval":LaunchConfiguration('tf_publish_interval')},
        #         {"future_date_transforms":LaunchConfiguration('future_date_transforms')},
        #         {"publish_6dof_pose":LaunchConfiguration('publish_6dof_pose')},
        #         {"read_only_map":LaunchConfiguration('read_only_map')},
        #         {"covariance_diagonal":LaunchConfiguration('covariance_diagonal')}
        #     ],
        #     remappings=[
        #         ('/fiducial_transforms', '/aruco_detect/fiducial_transforms'),
        #     ]
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
        ])

    # bee_rviz = os.path.join(bee_pkg, 'launch', 'default.rviz')

    # rviz_config_file = LaunchConfiguration('rviz_config')
    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     'rviz_config',
    #     default_value=bee_rviz,
    #     description='Full path to the RVIZ config file to use')

    return ld