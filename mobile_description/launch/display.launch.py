from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_path
def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('mobile_description'),
                             'urdf', 'my_robot_wheel.urdf')
    
    rviz_config_path = os.path.join(get_package_share_path('mobile_description'),
                                    'rviz', 'urdf_config.rviz')
    
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d',rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
