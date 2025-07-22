from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'gazebo_robot_xacro.urdf.xacro')
    controller_config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'controller.yaml')
    
    # gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ])
    )

    # state publisher
    state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_file)]), 
                value_type=str
            )}]
        )
    
    # spawn node
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            # '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
    output='screen'
    )
    
    # Diff Drive Controller
    diff_drive_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
    output='screen'
    )

    # Controller Manager
    controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
        {'robot_description': Command(['xacro', urdf_file])},
        controller_config
        ],
    output='screen'
)

    return LaunchDescription([
        state_publisher,
        spawn_node,
        gazebo,
        controller_manager_node,

        joint_state_broadcaster,
        diff_drive_controller
    ])