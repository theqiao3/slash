#!/usr/bin/env python3
"""CMD_VEL 重新映射节点

将一个 Twist 控制命令话题转发到另一个主题，默认把 /cmd_vel 复制到 /tianracer/cmd_vel。
"""

import argparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self, input_topic: str, output_topic: str, queue_size: int = 10) -> None:
        super().__init__('cmd_vel_relay')
        self.input_topic = self.declare_parameter('input_topic', input_topic).value
        self.output_topic = self.declare_parameter('output_topic', output_topic).value
        self.publisher = self.create_publisher(Twist, self.output_topic, queue_size)
        self.subscription = self.create_subscription(
            Twist,
            self.input_topic,
            self._cmd_vel_callback,
            queue_size,
        )
        self.get_logger().info(
            f'转发 {self.input_topic} -> {self.output_topic} (队列={queue_size})'
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.publisher.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description='CMD_VEL 话题重映射器')
    parser.add_argument(
        '--input-topic',
        default='/cmd_vel',
        help='要监听的 Twist 话题，默认 /cmd_vel',
    )
    parser.add_argument(
        '--output-topic',
        default='/tianracer/cmd_vel',
        help='发布到的 Twist 话题，默认 /tianracer/cmd_vel',
    )
    parser.add_argument(
        '--qos',
        type=int,
        default=10,
        help='Publisher/Subscriber 队列大小',
    )
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelRelay(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        queue_size=args.qos,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断，退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
