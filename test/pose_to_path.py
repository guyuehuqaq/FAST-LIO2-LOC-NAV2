#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PoseToPathNode(Node):
    def __init__(self):
        super().__init__('pose_to_path_node')
        self.path = Path()
        self.path.header.frame_id = 'map'  # 必须是 map
        self.max_length = 200

        # 订阅 NDT 定位结果
        self.sub = self.create_subscription(
            PoseStamped, '/localization_3d', self.pose_cb, 10
        )

        # 发布 Path
        self.pub = self.create_publisher(Path, '/global_path', 10)

    def pose_cb(self, msg):
        self.path.poses.append(msg)
        if len(self.path.poses) > self.max_length:
            self.path.poses.pop(0)

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()