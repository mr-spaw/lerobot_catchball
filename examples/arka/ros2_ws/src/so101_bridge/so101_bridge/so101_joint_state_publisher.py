import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig


class SO101JointPublisher(Node):

    def __init__(self):
        super().__init__('so101_joint_state_publisher')

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.timer = self.create_timer(0.05, self.publish_joint_state)

        config = SO100FollowerConfig(
            port="/dev/ttyACM0",
            id="my_awesome_follower_arm",
            use_degrees=True,
        )

        self.robot = SO100Follower(config)
        self.robot.connect()

        self.joint_names = None

    def publish_joint_state(self):
        obs = self.robot.get_observation()

        joint_keys = [k for k in obs.keys() if k.endswith(".pos")]

        if self.joint_names is None:
            self.joint_names = [k.replace(".pos", "") for k in joint_keys]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [obs[k] * 3.14159 / 180.0 for k in joint_keys]  # degrees → radians

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SO101JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
