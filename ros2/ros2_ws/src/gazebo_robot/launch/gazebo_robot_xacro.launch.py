from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # pkg_my_robot = FindPackageShare('gazebo_robot')
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'gazebo_robot_xacro.urdf.xacro')
    # controller_config = PathJoinSubstitution([pkg_my_robot, 'config', 'controller.yaml'])
    
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
    # joint_state_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Joint Trajectory Controller (or diff_drive_controller etc.)
    # trajectory_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )
    trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        state_publisher,
        spawn_node,
        gazebo,
    #    TimerAction(period=3.0, actions=[joint_state_broadcaster]),
    #     TimerAction(period=4.0, actions=[trajectory_controller])
        joint_state_broadcaster,
        trajectory_controller
    ])