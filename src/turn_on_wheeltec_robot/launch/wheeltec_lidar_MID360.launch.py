import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    # todo：添加参数
    livox_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("livox_ros_driver2"),
            "launch",
            "msg_MID360_launch.py"
        )
    )

    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_scan',
        output='screen',
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            'min_height': -1.0,
            'max_height': 1.0,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.01,
            'range_min': 0.1,
            'range_max': 50.0,
            'use_inf': True,
        }]
    )

    return LaunchDescription([
        livox_driver,
        # pc_to_scan_node
    ])