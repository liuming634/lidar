#!/usr/bin/env python3
"""订阅 /livox/lidar 攒 N 秒点云，保存为 PCD 文件。"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class BagToPcd(Node):
    def __init__(self):
        super().__init__('bag_to_pcd')
        self.declare_parameter('duration', 10)
        self.declare_parameter('output', 'config/field.pcd')
        duration = self.get_parameter('duration').value
        self.output = self.get_parameter('output').value

        self.points = []  # [(x, y, z), ...]

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.callback, qos)

        self.timer = self.create_timer(duration, self.finish)
        self.get_logger().info(f'Recording for {duration}s -> {self.output}')

    def callback(self, msg):
        gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        for p in gen:
            self.points.append(p)
        self.get_logger().info(f'Accumulated {len(self.points)} points', throttle_duration_sec=2)

    def finish(self):
        self._save_pcd(self.points, self.output)
        self.get_logger().info(f'Saved {len(self.points)} points to {self.output}')
        rclpy.shutdown()

    @staticmethod
    def _save_pcd(points, path):
        with open(path, 'w') as f:
            f.write(
                '# .PCD v0.7 - Point Cloud Data file format\n'
                'VERSION 0.7\n'
                'FIELDS x y z\n'
                'SIZE 4 4 4\n'
                'TYPE F F F\n'
                'COUNT 1 1 1\n'
                f'WIDTH {len(points)}\n'
                'HEIGHT 1\n'
                'VIEWPOINT 0 0 0 1 0 0 0\n'
                f'POINTS {len(points)}\n'
                'DATA ascii\n'
            )
            for p in points:
                f.write(f'{p[0]} {p[1]} {p[2]}\n')


def main():
    rclpy.init()
    node = BagToPcd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()


if __name__ == '__main__':
    main()
