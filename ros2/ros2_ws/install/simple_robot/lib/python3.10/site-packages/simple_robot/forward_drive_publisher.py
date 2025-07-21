import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class FakeDrive(Node):
    def __init__(self):
        super().__init__('fake_drive')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update)  # 20Hz
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        # Simulate wheel rotation (angle = speed * time)
        angle = elapsed * 2.0  # 2 rad/s

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [angle, angle]
        self.joint_pub.publish(joint_state)

        # Simulate forward motion (x = velocity * time)
        x = elapsed * 0.1  # 0.1 m/s
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # No rotation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = FakeDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
