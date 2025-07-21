from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'simple_robot.xml'
        ))
    rviz_config_file = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'rviz', 'my_robot_config.rviz'
        ))

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])