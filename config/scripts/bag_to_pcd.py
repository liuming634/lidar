#!/usr/bin/env python3
"""
rosbag2 → PCD 点云转换工具

将 ROS 2 bag 中的 sensor_msgs/PointCloud2 数据提取并保存为 PCD 文件。

用法:
  python3 bag_to_pcd.py <input_bag> <topic> [output_dir]
  python3 bag_to_pcd.py <input_bag> --all <output_dir>

记录时只保留点云话题:
  ros2 bag record <your_pointcloud_topic> -o <output_bag>
"""

import argparse
import os
import sys
import struct

import numpy as np
from pypcd import pypcd


def _detect_storage_id(bag_path):
    """自动检测 rosbag2 存储格式 (mcap / sqlite3)。"""
    import rosbag2_py
    # rosbag2_py 提供 get_default_storage_id，但更可靠的是探查目录内容
    try:
        plugins = rosbag2_py.get_registered_readers()
    except AttributeError:
        plugins = []

    # 优先用 mcap（如果存在 .mcap 文件或 mcap 插件可用）
    if os.path.isdir(bag_path):
        has_mcap_file = any(f.endswith(".mcap") for f in os.listdir(bag_path))
        has_db3_file = any(f.endswith(".db3") for f in os.listdir(bag_path))
        if has_mcap_file:
            return "mcap"
        if has_db3_file:
            return "sqlite3"
    elif bag_path.endswith(".mcap"):
        return "mcap"
    elif bag_path.endswith(".db3"):
        return "sqlite3"

    # 根据可用插件猜测
    for p in plugins:
        if "mcap" in p:
            return "mcap"
    return "sqlite3"


def read_ros2_bag(bag_path, topic_filter=None):
    """遍历 rosbag2，提取 PointCloud2 消息。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py import get_message_interfaces
    import rosidl_runtime_py

    storage_id = _detect_storage_id(bag_path)
    print(f"检测到存储格式: {storage_id}")

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {}
    for info in topic_types:
        type_map[info.name] = info.type

    # 收集目标话题列表
    target_topics = []
    all_cloud_topics = []
    for topic_name, topic_type in type_map.items():
        if topic_type == "sensor_msgs/msg/PointCloud2":
            all_cloud_topics.append(topic_name)
            if topic_filter is None or topic_name == topic_filter:
                target_topics.append(topic_name)

    if not target_topics:
        print(f"错误: 话题 '{topic_filter}' 中未找到 PointCloud2 数据", file=sys.stderr)
        print(f"可用的 PointCloud2 话题: {all_cloud_topics}", file=sys.stderr)
        sys.exit(1)

    print(f"目标话题: {target_topics}")

    messages = []
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic in target_topics:
            msg_type = get_message_interfaces.get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            messages.append((topic, timestamp, msg))

    reader.close()
    return messages


def pointcloud2_to_xyz_array(msg):
    """将 sensor_msgs/PointCloud2 解包为 N×3 numpy 数组。"""
    from sensor_msgs_py import point_cloud2
    return point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))


def pointcloud2_to_pcd_xyzi(msg):
    """将 PointCloud2 转为 pypcd 的 PointCloud 对象（含 xyz + intensity）。"""
    from sensor_msgs_py import point_cloud2

    field_names = [f.name for f in msg.fields]
    has_intensity = "intensity" in field_names
    has_ring = "ring" in field_names
    has_time = "time" in field_names

    read_fields = ["x", "y", "z"]
    if has_intensity:
        read_fields.append("intensity")

    points = point_cloud2.read_points_numpy(msg, field_names=read_fields)

    # 构建 pypcd 的元数据
    fields = [
        ("x", np.float32),
        ("y", np.float32),
        ("z", np.float32),
    ]
    if has_intensity:
        fields.append(("intensity", np.float32))

    # 构造结构化数组
    dtype = np.dtype([(name, t) for name, t in fields])
    structured = np.zeros(len(points), dtype=dtype)
    for name, _ in fields:
        structured[name] = points[name]

    pc = pypcd.make_xyzi_point_cloud(structured) if has_intensity else pypcd.make_xyz_point_cloud(structured)
    return pc


def save_pcd(pc, output_path):
    """保存 PointCloud 为 PCD (ASCII 格式)。"""
    pypcd.save_point_cloud(pc, output_path)
    print(f"  已保存: {output_path}")


def convert_bag(args):
    bag_path = args.bag
    output_dir = args.output
    topic = args.topic

    os.makedirs(output_dir, exist_ok=True)

    print(f"读取 bag: {bag_path}")
    messages = read_ros2_bag(bag_path, topic_filter=topic)

    print(f"共找到 {len(messages)} 帧点云数据")

    for i, (topic_name, timestamp, msg) in enumerate(messages):
        pc = pointcloud2_to_pcd_xyzi(msg)
        # 使用时间戳命名，保留可读性
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        filename = f"{sec}_{nsec:09d}.pcd"
        output_path = os.path.join(output_dir, filename)
        save_pcd(pc, output_path)

    print(f"\n完成! 共转换 {len(messages)} 个点云帧 → {output_dir}")


def list_topics(args):
    """列出 bag 中所有话题及其类型。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py import get_message_interfaces

    storage_id = _detect_storage_id(args.bag)
    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    print(f"\nBag 文件: {args.bag}")
    print(f"{'话题':<60} {'类型'}")
    print("-" * 90)
    for info in topics:
        mark = " ← 点云" if info.type == "sensor_msgs/msg/PointCloud2" else ""
        print(f"{info.name:<60} {info.type}{mark}")


def main():
    parser = argparse.ArgumentParser(
        description="ROS 2 bag → PCD 点云转换工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 列出 bag 中的话题
  python3 bag_to_pcd.py --list my_bag/

  # 转换指定话题
  python3 bag_to_pcd.py my_bag/ /radar/points  ./pcd_output/

  # 录制时只保留点云话题
  ros2 bag record /radar/points -o my_bag
        """,
    )

    parser.add_argument("bag", help="输入 rosbag2 目录路径")
    parser.add_argument("topic", nargs="?", default=None, help="点云话题名称 (留空表示转换所有 PointCloud2 话题)")
    parser.add_argument("output", nargs="?", default="./pcd_output", help="PCD 输出目录 (默认: ./pcd_output)")
    parser.add_argument("--list", "-l", action="store_true", help="列出 bag 中的话题并退出")

    args = parser.parse_args()

    if not os.path.isdir(args.bag):
        # 也允许传入 .mcap 文件路径
        if not os.path.isfile(args.bag):
            print(f"错误: 找不到 bag 文件 '{args.bag}'", file=sys.stderr)
            sys.exit(1)

    if args.list:
        list_topics(args)
        return

    if args.topic is None:
        # 未指定话题 — 自动转换所有 PointCloud2 话题
        print("未指定话题，将自动转换所有 PointCloud2 数据")
        args.topic = None

    convert_bag(args)


if __name__ == "__main__":
    main()
