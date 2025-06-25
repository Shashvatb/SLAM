from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    publisher_node = Node(
        package='pub_sub',
        executable='minimal_publisher',
        name='minimal_publisher'
    )

    subscriber_node = Node(
        package='pub_sub',
        executable='minimal_subscriber',
        name='minimal_subscriber'
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node
    ])