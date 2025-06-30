from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'simple_robot_xacro.urdf.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_file)]), 
                value_type=str
            )}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])