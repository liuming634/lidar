#!/usr/bin/env python3
"""
PCD → YAML 配置文件生成工具

读取场地 PCD 点云文件，自动提取场地范围、高度等参数，
写入对应的 YAML 配置文件中。

用法:
  python3 pcd_to_config.py                          # 使用默认路径 config/RM2025.pcd
  python3 pcd_to_config.py --pcd 路径/场地点云.pcd   # 指定 PCD 文件
  python3 pcd_to_config.py --dry-run                # 仅预览，不写入
"""

import argparse
import os
import sys
import struct
import yaml
import numpy as np

try:
    import lzf
except ImportError:
    lzf = None

# ============================================================
# 配置文件路径（相对于项目根目录）
# ============================================================
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

CONFIG_FILES = {
    "field_params":     os.path.join(PROJECT_ROOT, "config/params/field_params.yaml"),
    "elevation_meta":   os.path.join(PROJECT_ROOT, "config/params/elevation_meta.yaml"),
    "gicp_crop":        os.path.join(PROJECT_ROOT, "config/params/gicp_crop.yaml"),
    "dynamic_crop":     os.path.join(PROJECT_ROOT, "config/params/dynamic_crop.yaml"),
    "map":              os.path.join(PROJECT_ROOT, "config/map.yaml"),
}


def read_pcd_header(file_path):
    """读取 PCD 文件头，返回 (header_dict, data_offset, is_binary, is_compressed)。"""
    header = {}
    with open(file_path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line == "DATA ascii":
                header["data_mode"] = "ascii"
                return header, f.tell(), False, False
            if line.startswith("DATA binary_compressed"):
                header["data_mode"] = "binary_compressed"
                return header, f.tell(), True, True
            if line.startswith("DATA binary"):
                header["data_mode"] = "binary"
                return header, f.tell(), True, False

            parts = line.split(None, 1)
            if len(parts) == 2:
                key, value = parts
                header[key] = value.strip()

    raise ValueError("无法找到 DATA 行，PCD 文件可能已损坏")


def parse_header_fields(header):
    """从 header dict 解析字段信息。"""
    fields = header.get("FIELDS", "").split()
    sizes = [int(s) for s in header.get("SIZE", "").split()]
    types = header.get("TYPE", "").split()
    counts = [int(c) for c in header.get("COUNT", "").split()]

    if not fields:
        raise ValueError("PCD 头缺少 FIELDS 定义")

    # 默认值
    if not sizes:
        sizes = [4] * len(fields)
    if not types:
        types = ["F"] * len(fields)
    if not counts:
        counts = [1] * len(fields)

    num_points = int(header.get("POINTS", 0))
    width = int(header.get("WIDTH", num_points))
    height = int(header.get("HEIGHT", 1))
    if num_points == 0:
        num_points = width * height

    return fields, types, sizes, counts, num_points


def load_pcd(file_path):
    """加载 PCD 文件，返回点云数据 (N×3 numpy array)。"""
    if not os.path.isfile(file_path):
        print(f"错误: 找不到文件 '{file_path}'")
        sys.exit(1)

    print(f"读取 PCD: {file_path}")

    header, data_offset, is_binary, is_compressed = read_pcd_header(file_path)
    fields, types, sizes, counts, num_points = parse_header_fields(header)

    print(f"  点数: {num_points:,}")
    print(f"  字段: {fields}")
    print(f"  模式: {'binary_compressed' if is_compressed else 'binary' if is_binary else 'ascii'}")

    if not is_binary:
        print("  错误: 仅支持 binary/binary_compressed 格式的 PCD")
        sys.exit(1)

    # 读取原始数据
    with open(file_path, "rb") as f:
        f.seek(data_offset)
        raw = f.read()

    if is_compressed:
        if lzf is None:
            print("  错误: 需要 lzf 模块来解压 binary_compressed，请安装: pip install lzf")
            sys.exit(1)
        compressed_size = struct.unpack_from("I", raw, 0)[0]
        uncompressed_size = struct.unpack_from("I", raw, 4)[0]
        compressed_data = raw[8:8 + compressed_size]
        raw = lzf.decompress(compressed_data, uncompressed_size)
        if raw is None or len(raw) != uncompressed_size:
            print("  错误: LZF 解压失败")
            sys.exit(1)

    # 构建 dtype
    dtype_list = []
    byte_offset = 0
    for name, typ, size, count in zip(fields, types, sizes, counts):
        if typ in ("F",):
            np_type = np.float32
        elif typ in ("I",):
            np_type = np.int32
        elif typ in ("U",):
            np_type = np.uint32
        else:
            np_type = np.float32
        # field-major 存储：每个字段的所有数据连续存放
        dtype_list.append((name, np_type))
        byte_offset += size * count

    # binary_compressed 和 binary 都是 field-major: 先所有 x, 再所有 y, 再所有 z
    arr = np.frombuffer(raw, dtype=np.dtype(dtype_list))
    # 每个字段都包含 num_points 个值
    # arr 的 shape 是 (num_points,), 每个元素是一个 tuple
    # 提取 xyz
    xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64)

    # 过滤 NaN / inf
    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid]

    print(f"  有效点: {len(xyz):,}")
    return xyz


def compute_stats(cloud):
    """计算点云统计信息。"""
    x, y, z = cloud[:, 0], cloud[:, 1], cloud[:, 2]
    stats = {
        "x_min": float(x.min()),
        "x_max": float(x.max()),
        "y_min": float(y.min()),
        "y_max": float(y.max()),
        "z_min": float(z.min()),
        "z_max": float(z.max()),
        "field_width":  round(float(x.max() - x.min()), 2),
        "field_height": round(float(y.max() - y.min()), 2),
        "num_points":   len(cloud),
    }
    return stats


def print_stats(stats):
    """打印点云统计信息。"""
    print(f"\n{'='*50}")
    print(f"场地范围统计")
    print(f"{'='*50}")
    print(f"  X: {stats['x_min']:.3f} ~ {stats['x_max']:.3f} m  (宽度: {stats['field_width']:.2f} m)")
    print(f"  Y: {stats['y_min']:.3f} ~ {stats['y_max']:.3f} m  (高度: {stats['field_height']:.2f} m)")
    print(f"  Z: {stats['z_min']:.4f} ~ {stats['z_max']:.4f} m")
    print(f"  点数: {stats['num_points']:,}")


def load_yaml(file_path):
    """加载 YAML 文件，返回 dict。"""
    if not os.path.isfile(file_path):
        print(f"  警告: 文件不存在，将新建 '{file_path}'")
        return {}
    with open(file_path, "r") as f:
        return yaml.safe_load(f) or {}


def write_yaml(file_path, data):
    """将 dict 写入 YAML 文件。"""
    with open(file_path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f"  已写入: {file_path}")


def update_field_params(stats, pcd_rel):
    """更新 field_params.yaml"""
    data = {
        "# 场地尺寸参数 (自动生成)": "",
        "field_width":  stats["field_width"],
        "field_height": stats["field_height"],
    }
    return data


def update_elevation_meta(stats, pcd_rel):
    """更新 elevation_meta.yaml"""
    data = {
        "# 高程图元信息 (自动生成)": "",
        "elevation_image": "config/elevation/2_elevation_gray256.png",
        "field_pcd":       pcd_rel,
        "field_map":       "config/RM2025.png",
        "field_width":     stats["field_width"],
        "field_height":    stats["field_height"],
        "display_scale":   25,
        "z_min":           round(stats["z_min"], 6),
        "z_max":           round(stats["z_max"], 6),
        "no_data":         255,
    }
    return data


def update_gicp_crop(stats, pcd_rel):
    """更新 gicp_crop.yaml (加 2m 边距用于配准)。"""
    margin = 2.0
    data = {
        "# GICP 配准点云裁剪参数 (自动生成，建议人工复核)": "",
        "pcd_path": pcd_rel,
        "crop": {
            "x_min": round(stats["x_min"] - margin, 1),
            "x_max": round(stats["x_max"] + margin, 1),
            "y_min": round(stats["y_min"] - margin, 1),
            "y_max": round(stats["y_max"] + margin, 1),
            "z_max": round(stats["z_max"] + 1.0, 1),
        },
        "voxel_leaf_size":   0.1,
        "fitness_threshold": 0.2,
        "accumulate_frames": 20,
        "grid_degrees":      0.1,
    }
    return data


def update_dynamic_crop(stats, pcd_rel):
    """更新 dynamic_crop.yaml (动态点检测范围，与场地一致)。"""
    data = {
        "# 动态点云裁剪配置 (自动生成，建议人工复核)": "",
        "pcd_path":          pcd_rel,
        "kdtree_threshold":  0.1,
        "accumulate_frames": 3,
        "crop": {
            "x_min": round(stats["x_min"], 1),
            "x_max": round(stats["x_max"], 1),
            "y_min": round(stats["y_min"], 1),
            "y_max": round(stats["y_max"], 1),
            "z_min": round(stats["z_min"], 1),
            "z_max": round(stats["z_max"], 1),
        },
        "exclude_zones": [
            {
                "name": "需根据实际场地调整",
                "x_min": 0,
                "x_max": 0,
                "y_min": 0,
                "y_max": 0,
            }
        ],
    }
    return data


def main():
    parser = argparse.ArgumentParser(description="PCD → YAML 配置文件生成工具")
    parser.add_argument("--pcd", default=os.path.join(PROJECT_ROOT, "config/RM2025.pcd"),
                        help="PCD 文件路径 (默认: config/RM2025.pcd)")
    parser.add_argument("--dry-run", action="store_true",
                        help="仅预览，不写入文件")
    args = parser.parse_args()

    # 加载 PCD
    cloud = load_pcd(args.pcd)
    stats = compute_stats(cloud)
    print_stats(stats)

    # PCD 相对路径（相对于项目根目录）
    pcd_rel = os.path.relpath(args.pcd, PROJECT_ROOT)

    # 生成各配置文件
    updates = {
        "field_params":   update_field_params(stats, pcd_rel),
        "elevation_meta": update_elevation_meta(stats, pcd_rel),
        "gicp_crop":      update_gicp_crop(stats, pcd_rel),
        "dynamic_crop":   update_dynamic_crop(stats, pcd_rel),
    }

    # 预览或写入
    print(f"\n{'='*50}")
    if args.dry_run:
        print(f"预览模式 (--dry-run)，不写入文件")
    else:
        print(f"写入配置文件")
    print(f"{'='*50}")

    for key, data in updates.items():
        file_path = CONFIG_FILES[key]
        print(f"\n--- {key} ({file_path}) ---")
        print(yaml.dump(data, default_flow_style=False, allow_unicode=True, sort_keys=False).strip())
        if not args.dry_run:
            write_yaml(file_path, data)

    if not args.dry_run:
        print(f"\n完成! 已更新 {len(updates)} 个配置文件。")
        print(f"注意: gicp_crop 和 dynamic_crop 的裁剪范围建议人工复核。")
        print(f"      exclude_zones 需要根据实际场地布局手动配置。")


if __name__ == "__main__":
    main()
