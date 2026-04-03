#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path

class OdomToPathNode(Node):
    def __init__(self):
        super().__init__('odom_to_path_node')

        self.path = Path()
        self.path.header.frame_id = 'odom'  # 里程计坐标系
        self.max_length = 200

        # 订阅 FAST-LIO2 里程计
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.odom_cb, 10
        )

        # 发布 Path
        self.pub = self.create_publisher(Path, '/odom_path', 10)

    def odom_cb(self, msg):
        pose_stamped = Odometry()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose

        self.path.poses.append(pose_stamped.pose)

        if len(self.path.poses) > self.max_length:
            self.path.poses.pop(0)

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()